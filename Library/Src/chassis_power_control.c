#include "chassis_power_control.h"
#include "math.h"

#define DEBUG_POWER 0

/* 辅助宏 */
#define CLAMP(val, min, max) ((val) > (max) ? (max) : ((val) < (min) ? (min) : (val)))
#define ABS(x) ((x) > 0 ? (x) : -(x))

extern data_referee_t data_referee;
static chassis_power_control_t pc = {0};
extern capacitor_measure_t cap_measure;

/* ==================== 内部辅助函数 ==================== */

/**
 * @brief  计算单个电机功率（物理模型）
 * @param  speed_rpm       电机转速(RPM)
 * @param  current_raw     PID输出电流(C620原始值, -16384~+16384)
 * @return 估算功率(W)
 *
 * 物理模型: P = K_Torque * I(A) * ω(RPM) + R(Ω) * I²(A²) + P_Rest(W)
 * 其中 I(A) = current_raw * M3508_CURRENT_CONVERT
 */
static float motor_power_calc(int16_t speed_rpm, float current_raw)
{
    float I = current_raw * M3508_CURRENT_CONVERT; /* 转换为安培 */
    float P = M3508_K_TORQUE * I * (float)speed_rpm + M3508_MOTOR_R * I * I + M3508_POWER_REST;
    return (P > 0.0f) ? P : 0.0f;
}

/**
 * @brief  分段式可用功率计算
 * @param  max_power   功率上限(W)
 * @param  buffer      当前仿真缓冲能量(J)
 * @return 可用功率(W)
 *
 * 策略:
 *   buffer > 40J:  全功率 + 额外余量（缓冲充足，放心用）
 *   40J > buffer > 10J: 功率随缓冲线性递减
 *   buffer < 10J:  严格限制到功率上限以下（危险区）
 */
static float calc_available_power(float max_power, float buffer)
{
    if (buffer > BUFFER_HIGH)
    {
        /* 缓冲充足: 给足功率，允许短时超限由缓冲吸收 */
        return max_power + (buffer - BUFFER_HIGH) * 1.5f;
    }
    else if (buffer > BUFFER_LOW)
    {
        /* 缓冲中等: 线性递减，从 max_power+30W 到 max_power */
        float ratio = (buffer - BUFFER_LOW) / (BUFFER_HIGH - BUFFER_LOW);
        return max_power + ratio * max_power * 0.25f;
    }
    else if (buffer > BUFFER_MIN)
    {
        /* 缓冲危险: 严格限制到功率上限附近 */
        float ratio = (buffer - BUFFER_MIN) / (BUFFER_LOW - BUFFER_MIN);
        return max_power * (0.7f + 0.3f * ratio); /* 最低70%功率上限 */
    }
    else
    {
        /* 缓冲极限: 最大功率限制，防止断电 */
        float ratio = buffer / BUFFER_MIN;
        return max_power * (0.5f + 0.2f * ratio); /* 最低50%功率上限 */
    }
}

/* ==================== 初始化 ==================== */

void chassis_power_control_init(void)
{
    pc.estimated_power = 0.0f;
    pc.sim_buffer = BUFFER_FULL;
    pc.max_power_limit = CHASSIS_POWER_MAX_DEFAULT;
    pc.available_power = CHASSIS_POWER_MAX_DEFAULT;
    pc.cap_state = 0;
    pc.power_scale = 1.0f;
    pc.power_correction = 0.0f;
    pc.K_Reduction = 1.0f;
    pc.K_I_Reduction_Min = 1.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        pc.motor_power[i] = 0.0f;
        pc.last_pid_out[i] = 0.0f;
        pc.last_final_pid_out[i] = 0.0f;
        pc.K_PI[i] = 0.0f;
        pc.M_PI_val[i] = 0.0f;
    }
    pc.total_motor_power = 0.0f;
    pc.power_predict = 0.0f;
    pc.max_power_record = 0.0f;
    pc.limit_count = 0;
}

/* ==================== 主控函数 ==================== */

uint8_t chassis_power_control_process(pid_type_def chassis_pid[4],
                                      motor_measure_DJ_t motor_measure[4],
                                      int16_t speed_set[4])
{
    uint8_t is_limited = 0;

    /* ================================================================
     * 步骤1: 用原始速度目标计算PID，获取PID内部状态
     * (iout, last_error等，用于后续线性化参数提取)
     * ================================================================ */
    // float saved_iout[4];
    // float saved_last_error[4];

    // for (uint8_t i = 0; i < 4; i++)
    // {
    //     /* 保存调用前的积分项（PID_Calc前的iout是上一拍的积分结果） */
    //     saved_iout[i] = chassis_pid[i].iout;
    //     saved_last_error[i] = chassis_pid[i].last_error;

    //     chassis_pid[i].out = PID_Calc(&chassis_pid[i],
    //                                    motor_measure[i].speed_rpm,
    //                                    speed_set[i], 0);
    // }
    float saved_iout[4];
    float saved_error[4];

    for (uint8_t i = 0; i < 4; i++)
    {
        saved_iout[i] = chassis_pid[i].iout;
        saved_error[i] = chassis_pid[i].error;

        chassis_pid[i].out = PID_Calc(&chassis_pid[i],
                                      motor_measure[i].speed_rpm,
                                      speed_set[i],
                                      0);
    }

    /* ================================================================
     * 第0层: 电流上升率限制（防起步冲击）
     * 限制每拍PID输出的变化量，防止突然给大电流导致功率尖峰
     * ================================================================ */
    // for (uint8_t i = 0; i < 4; i++)
    // {
    //     float delta = chassis_pid[i].out - pc.last_pid_out[i];
    //     if (delta > CURRENT_RISE_LIMIT)
    //         chassis_pid[i].out = pc.last_pid_out[i] + CURRENT_RISE_LIMIT;
    //     else if (delta < -CURRENT_RISE_LIMIT)
    //         chassis_pid[i].out = pc.last_pid_out[i] - CURRENT_RISE_LIMIT;
    //     pc.last_pid_out[i] = chassis_pid[i].out;
    // }

    /* ================================================================
     * 估算当前实际功率 + 仿真缓冲能量
     * 替代 data_referee.PowerHeatData（该数据不可用）
     * ================================================================ */

    /* 1) 用上一拍最终PID输出估算当前实际功率（衰减后的真实值） */
    pc.total_motor_power = 0.0f;
    for (uint8_t i = 0; i < 4; i++)
    {
        pc.motor_power[i] = motor_power_calc(motor_measure[i].speed_rpm,
                                             pc.last_final_pid_out[i]);
        pc.total_motor_power += pc.motor_power[i];
    }
    pc.estimated_power = pc.total_motor_power;

    /* 2) 获取功率上限
     *    GameRobotState(0x0201) 与 PowerHeatData(0x0202) 是不同数据包
     *    chassis_power_limit 通常仍然可靠，但加安全裕度
     */
    float raw_power_limit = (float)data_referee.GameRobotState.chassis_power_limit;
    if (raw_power_limit < 10.0f || raw_power_limit > 250.0f)
        raw_power_limit = CHASSIS_POWER_MAX_DEFAULT;

    /* 乘以安全系数: 用功率上限的98%作为控制目标，留模型误差裕度 */
    pc.max_power_limit = raw_power_limit * POWER_SAFETY_FACTOR;

    /* 3) 仿真缓冲能量: dE/dt = P_limit - P_actual
     *    每拍更新: E += (P_limit - P_estimated) * dt
     *    钳位在 [0, BUFFER_FULL]
     */
    pc.sim_buffer += (raw_power_limit - pc.estimated_power) * CONTROL_PERIOD_S;
    if (pc.sim_buffer > BUFFER_FULL)
        pc.sim_buffer = BUFFER_FULL;
    if (pc.sim_buffer < 0.0f)
        pc.sim_buffer = 0.0f;

    /* 计算可用功率（分段式缓冲能量策略） */
    pc.available_power = calc_available_power(pc.max_power_limit, pc.sim_buffer);

    /* 考虑超级电容爆发 */
    if (pc.cap_state == 1)
        pc.available_power += CAPACITOR_BOOST_POWER;

    /* ================================================================
     * 第1层: 电流超限约束（防单电机超流）
     * 从PID当前状态提取线性化参数 K_PI 和 M_PI_val
     * 预测电流 = K_PI * speed_set + M_PI_val
     * 如果预测电流超限，计算统一衰减系数
     * ================================================================ */
    pc.K_I_Reduction_Min = 1.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        float speed_rpm = (float)motor_measure[i].speed_rpm;
        float Kp = chassis_pid[i].Kp;
        float Kd = chassis_pid[i].Kd;

        /*
         * 从PID内部状态推导线性化参数:
         *
         * PID输出 = pout + iout + dout
         *         = Kp*(set-fdb) + iout + Kd*((set-fdb) - last_error)
         *         = (Kp+Kd)*set + [iout - Kd*last_error - (Kp+Kd)*fdb]
         *
         * 所以:
         *   K_PI = Kp + Kd
         *   M_PI_val = iout - Kd*last_error - K_PI*fdb
         *
         * 注意: 这里使用PID_Calc调用后的状态，iout已包含本拍积分
         */
        pc.K_PI[i] = Kp + Kd;
        pc.M_PI_val[i] = chassis_pid[i].iout - Kd * chassis_pid[i].last_error - pc.K_PI[i] * speed_rpm;

        /* 转换为实际电流(A)单位，用于功率模型 */
        pc.K_PI[i] *= M3508_CURRENT_CONVERT;
        pc.M_PI_val[i] *= M3508_CURRENT_CONVERT;

        /* 预测电流 */
        float predict_I = pc.K_PI[i] * (float)speed_set[i] + pc.M_PI_val[i];
        float I_max = MAX_TOTAL_CURRENT * M3508_CURRENT_CONVERT;

        if (ABS(predict_I) > I_max)
        {
            /* 预测电流超限，计算衰减系数 */
            float sign = (predict_I > 0) ? 1.0f : -1.0f;
            float denom = pc.K_PI[i] * (float)speed_set[i];
            if (ABS(denom) > 0.001f)
            {
                float K_I = (sign * I_max - pc.M_PI_val[i]) / denom;
                K_I = CLAMP(K_I, 0.0f, 1.0f);
                if (K_I < pc.K_I_Reduction_Min)
                    pc.K_I_Reduction_Min = K_I;
            }
        }
    }

    /* 应用电流约束衰减到转速设定值 */
    float adjusted_speed_set[4];
    for (uint8_t i = 0; i < 4; i++)
    {
        adjusted_speed_set[i] = (float)speed_set[i] * pc.K_I_Reduction_Min;
    }

    /* ================================================================
     * 第2层: 功率模型预测衰减（防超功率）
     * 将衰减后的转速设定值代入功率模型
     * 如果预测功率超过可用功率，求K_Reduction进一步衰减
     *
     * 功率模型: P = R * Σ(K_PI² * spd²) * K²
     *              + Σ(spd * (2*R*K_PI*M_PI + K_Torque*K_PI*ω)) * K
     *              + Σ(R*M_PI² + K_Torque*M_PI*ω) + 4*P_Rest
     * ================================================================ */
    float a = 0.0f, b = 0.0f, c = 0.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        float spd = adjusted_speed_set[i];
        float w = (float)motor_measure[i].speed_rpm; /* 当前实际转速 */

        a += M3508_MOTOR_R * spd * spd * pc.K_PI[i] * pc.K_PI[i];
        b += spd * (2.0f * M3508_MOTOR_R * pc.K_PI[i] * pc.M_PI_val[i] + M3508_K_TORQUE * pc.K_PI[i] * w);
        c += M3508_MOTOR_R * pc.M_PI_val[i] * pc.M_PI_val[i] + M3508_K_TORQUE * pc.M_PI_val[i] * w;
    }
    c += 4.0f * M3508_POWER_REST; /* 四个电机的静态损耗 */

    /* 计算当前预测功率 (K=1时) */
    pc.power_predict = a + b + c;

    if (pc.power_predict > pc.available_power)
    {
        is_limited = 1;
        pc.limit_count++;

        /* 求解一元二次方程: a*K² + b*K + (c - P_available) = 0 */
        float disc = b * b - 4.0f * a * (c - pc.available_power);

        if (a < 0.0001f || disc < 0.0f)
        {
            /* a≈0 (所有电机速度设定值为0) 或无实数解
             * 取对称轴作为最优衰减 */
            if (a > 0.0001f)
                pc.K_Reduction = CLAMP(-b / (2.0f * a), 0.0f, 1.0f);
            else
                pc.K_Reduction = 0.0f;
        }
        else
        {
            float sqrt_disc = sqrtf(disc);
            /* 取正根中在[0,1]范围内的解，优先取较大的（保留更多性能） */
            float K1 = (-b + sqrt_disc) / (2.0f * a);
            float K2 = (-b - sqrt_disc) / (2.0f * a);

            if (K1 >= 0.0f && K1 <= 1.0f && K2 >= 0.0f && K2 <= 1.0f)
                pc.K_Reduction = (K1 > K2) ? K1 : K2;
            else if (K1 >= 0.0f && K1 <= 1.0f)
                pc.K_Reduction = K1;
            else if (K2 >= 0.0f && K2 <= 1.0f)
                pc.K_Reduction = K2;
            else
                pc.K_Reduction = 0.0f;
        }

        /* 安全下限: 至少保留10%功率 */
        if (pc.K_Reduction < 0.1f)
            pc.K_Reduction = 0.1f;
    }
    else
    {
        pc.K_Reduction = 1.0f;
    }

    /* ================================================================
     * 第3层: 缓冲能量仿真自校正（替代裁判系统反馈校正）
     * 如果仿真缓冲能量持续低于危险阈值，说明模型可能低估了实际功耗，
     * 累加校正量进一步衰减输出
     * ================================================================ */
    if (pc.sim_buffer < BUFFER_MIN)
    {
        /* 仿真缓冲极低 → 需要额外衰减（模型可能低估功耗） */
        float error = BUFFER_MIN - pc.sim_buffer;
        pc.power_correction += POWER_CORRECT_KP * error / BUFFER_MIN;
        if (pc.power_correction > POWER_CORRECT_MAX)
            pc.power_correction = POWER_CORRECT_MAX;
    }
    else if (pc.sim_buffer > BUFFER_LOW)
    {
        /* 缓冲能量充足 → 快速释放校正量 */
        pc.power_correction *= 0.80f;
        if (pc.power_correction < 0.0f)
            pc.power_correction = 0.0f;
    }

    /* ================================================================
     * 综合衰减并重新计算PID输出
     * 总衰减 = 电流约束 × 功率预测 × 自校正
     *
     * 关键: 先回退第一次PID_Calc的积分累加，
     * 再用衰减后的转速设定值重新计算PID，保证积分项只累加一次
     * ================================================================ */
    float total_scale = pc.K_I_Reduction_Min * pc.K_Reduction * (1.0f - pc.power_correction);
    if (total_scale < 0.1f)
        total_scale = 0.1f;

    pc.power_scale = total_scale;

    /* 用衰减后的转速设定值作为PID的目标值重新计算 */
    // for (uint8_t i = 0; i < 4; i++)
    // {
    //     int16_t limited_speed = (int16_t)((float)speed_set[i] * total_scale);

    //     /* 回退第一次PID_Calc对积分项的累加
    //      * 恢复为PID_Calc调用前的积分值，这样第二次PID_Calc才是正确的积分
    //      */
    //     chassis_pid[i].iout = saved_iout[i];

    //     /* 同时恢复last_error为调用前的值，这样PID_Calc的D项才正确 */
    //     chassis_pid[i].last_error = saved_last_error[i];

    //     /* 重新计算PID: 用衰减后的速度目标 */
    //     chassis_pid[i].out = PID_Calc(&chassis_pid[i],
    //                                   motor_measure[i].speed_rpm,
    //                                   limited_speed, 0);

    //     /* 保存最终输出，供下一拍功率估算使用 */
    //     pc.last_final_pid_out[i] = chassis_pid[i].out;
    // }
    for (uint8_t i = 0; i < 4; i++)
    {
        int16_t limited_speed = (int16_t)((float)speed_set[i] * total_scale);

        chassis_pid[i].iout = saved_iout[i];
        chassis_pid[i].error = saved_error[i];

        chassis_pid[i].out = PID_Calc(&chassis_pid[i],
                                      motor_measure[i].speed_rpm,
                                      limited_speed,
                                      0);

        float delta = chassis_pid[i].out - pc.last_final_pid_out[i];

        if (delta > CURRENT_RISE_LIMIT)
        {
            chassis_pid[i].out = pc.last_final_pid_out[i] + CURRENT_RISE_LIMIT;
        }
        else if (delta < -CURRENT_RISE_LIMIT)
        {
            chassis_pid[i].out = pc.last_final_pid_out[i] - CURRENT_RISE_LIMIT;
        }

        pc.last_final_pid_out[i] = chassis_pid[i].out;
    }

    /* 空闲复位: 所有速度目标接近零时主动恢复状态，防止校正量卡死 */
    float max_speed_set = 0.0f;
    for (uint8_t i = 0; i < 4; i++)
    {
        float abs_spd = (float)speed_set[i];
        if (abs_spd < 0.0f)
            abs_spd = -abs_spd;
        if (abs_spd > max_speed_set)
            max_speed_set = abs_spd;
    }
    if (max_speed_set < 50.0f)
    {
        pc.sim_buffer += 5.0f;
        if (pc.sim_buffer > BUFFER_FULL)
            pc.sim_buffer = BUFFER_FULL;
        pc.power_correction *= 0.5f;
    }

    /* 记录最大功率 */
    if (pc.total_motor_power > pc.max_power_record)
        pc.max_power_record = pc.total_motor_power;

#if DEBUG_POWER
    printf("P:%.1f Pred:%.1f Avail:%.1f K:%.3f KI:%.3f Buf:%.0f Corr:%.3f\n",
           pc.estimated_power,
           pc.power_predict,
           pc.available_power,
           pc.K_Reduction,
           pc.K_I_Reduction_Min,
           pc.sim_buffer,
           pc.power_correction);
#endif

    return is_limited;
}

/* ==================== 辅助接口函数 ==================== */

float chassis_get_available_power(void)
{
    return pc.available_power;
}

void chassis_set_cap_state(uint8_t state)
{
    pc.cap_state = state;
}

/**
 * @brief S曲线加减速计算
 */
float s_curve_speed_calc(float current_speed, float target_speed, float max_speed)
{
    float speed_diff = target_speed - current_speed;
    float speed_ratio = fabsf(current_speed) / max_speed;

    float s_factor;

    if (speed_ratio < S_CURVE_ACCEL_RATIO)
    {
        s_factor = 0.5f + speed_ratio / S_CURVE_ACCEL_RATIO;
    }
    else if (speed_ratio < (1.0f - S_CURVE_DECEL_RATIO))
    {
        s_factor = S_CURVE_MAX_FACTOR;
    }
    else
    {
        s_factor = 0.5f + (1.0f - speed_ratio) / S_CURVE_DECEL_RATIO;
    }

    return speed_diff * s_factor;
}

/**
 * @brief 带S曲线的速度斜坡控制
 */
int16_t speed_ramp_s_curve(int16_t current, int16_t target,
                           int16_t accel_step, int16_t decel_step, int16_t max_speed)
{
    int16_t output = current;
    int16_t diff = target - current;

    if (diff == 0)
        return target;

    float speed_ratio = (float)ABS(current) / (float)ABS(max_speed);
    if (speed_ratio > 1.0f)
        speed_ratio = 1.0f;

    float s_factor;
    if (speed_ratio < 0.3f)
    {
        s_factor = 1.0f + speed_ratio / 0.3f * 0.5f;
    }
    else if (speed_ratio < 0.7f)
    {
        s_factor = 1.5f;
    }
    else
    {
        s_factor = 1.5f - (speed_ratio - 0.7f) / 0.3f * 1.0f;
    }

    int16_t step = (diff > 0) ? accel_step : decel_step;
    int16_t adjusted_step = (int16_t)(step * s_factor);

    if (adjusted_step < 1)
        adjusted_step = 1;
    if (adjusted_step > ABS(diff))
        adjusted_step = ABS(diff);

    if (diff > 0)
        output += adjusted_step;
    else
        output -= adjusted_step;

    return output;
}
