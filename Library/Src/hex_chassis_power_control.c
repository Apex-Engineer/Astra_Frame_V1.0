#include "hex_chassis_power_control.h"
#include "math.h"
#include "string.h"
#include "stdio.h"

extern data_referee_t data_referee;
extern capacitor_measure_t cap_measure;

/* 静态变量:六轮底盘功率控制实例 */
static hex_chassis_power_control_t hex_power_ctrl = {0};

/**
 * @brief 初始化六轮底盘功率控制
 */
void hex_chassis_power_control_init(void)
{
    memset(&hex_power_ctrl, 0, sizeof(hex_chassis_power_control_t));

    hex_power_ctrl.chassis_power = 0.0f;
    hex_power_ctrl.chassis_power_buffer = HEX_CHASSIS_BUFFER_MAX;
    hex_power_ctrl.max_power_limit = HEX_CHASSIS_POWER_MAX_DEFAULT;
    hex_power_ctrl.available_power = HEX_CHASSIS_POWER_MAX_DEFAULT - HEX_CHASSIS_POWER_SAFETY_MARGIN;
    hex_power_ctrl.cap_state = 0;
    hex_power_ctrl.power_scale = 1.0f;

    for (uint8_t i = 0; i < HEX_MOTOR_NUM; i++)
    {
        hex_power_ctrl.motor_power[i] = 0.0f;
    }
    hex_power_ctrl.total_motor_power = 0.0f;
    hex_power_ctrl.limit_count = 0;
    hex_power_ctrl.max_power_record = 0.0f;
}

/**
 * @brief 计算单个电机功率
 * @param speed_rpm 电机转速 (RPM)
 * @param current_pid_output PID输出电流
 * @return 估算的电机功率 (W)
 *
 * 电机功率模型: P = k1*I^2 + k2*w^2 + k3*I*w + constant
 * 简化模型: P = toque_coefficient * I * w + k2 * w^2 + k1 * I^2 + constant
 */
float hex_motor_power_calculate(int16_t speed_rpm, float current_pid_output)
{
    float power = 0.0f;

    // 基础功率模型
    power = HEX_MOTOR_TORQUE_COEFFICIENT * current_pid_output * speed_rpm +
            HEX_MOTOR_POWER_MODEL_K2 * speed_rpm * speed_rpm +
            HEX_MOTOR_POWER_MODEL_K1 * current_pid_output * current_pid_output +
            HEX_MOTOR_POWER_CONSTANT;

    // 如果功率为负(回馈/发电状态),计为0
    if (power < 0.0f)
    {
        power = 0.0f;
    }

    return power;
}

/**
 * @brief 根据目标功率反算允许的最大电流
 * @param speed_rpm 电机当前转速 (RPM)
 * @param target_power 目标功率 (W)
 * @param original_current 原始PID输出电流(用于判断方向)
 * @return 限制后的电流值
 *
 * 使用一元二次方程求解: a*I^2 + b*I + c = 0
 * 其中: a = k1, b = k3*w, c = k2*w^2 + constant - P_target
 */
float hex_motor_current_limit_by_power(int16_t speed_rpm, float target_power, float original_current)
{
    float a = HEX_MOTOR_POWER_MODEL_K1;
    float b = HEX_MOTOR_TORQUE_COEFFICIENT * speed_rpm;
    float c = HEX_MOTOR_POWER_MODEL_K2 * speed_rpm * speed_rpm + HEX_MOTOR_POWER_CONSTANT - target_power;

    float discriminant = b * b - 4.0f * a * c; // ？？？改为2ac试一下
    float limited_current = 0.0f;

    if (discriminant < 0.0f)
    {
        /* 无实数解时缩小电流,避免功率超限 */
        limited_current = original_current * 0.5f;
    }
    else
    {
        float sqrt_disc = sqrtf(discriminant);

        if (original_current > 0.01f)
        {
            // 原电流为正,取正根
            limited_current = (-b + sqrt_disc) / (2.0f * a);
        }
        else if (original_current < -0.01f)
        {
            // 原电流为负,取负根
            limited_current = (-b - sqrt_disc) / (2.0f * a);
        }
        else
        {
            limited_current = 0.0f;
        }
    }

    // 限制在合理范围内 (M3508电机电流范围)//16384对应100%电流,16000对应97.5%电流
    if (limited_current > 16000.0f)
        limited_current = 16000.0f;
    if (limited_current < -16000.0f)
        limited_current = -16000.0f;

    return limited_current;
}

/**
 * @brief 更新可用功率计算 (内部函数)
 *
 * 考虑因素:
 * 1. 裁判系统的功率限制
 * 2. 缓冲能量 (缓冲越大可用功率越多)
 * 3. 超级电容状态
 */
static void hex_update_available_power(void)
{
    // 1. 从裁判系统获取功率数据
    hex_power_ctrl.chassis_power = data_referee.PowerHeatData.chassis_power;
    hex_power_ctrl.chassis_power_buffer = data_referee.PowerHeatData.buffer_energy;
    hex_power_ctrl.max_power_limit = data_referee.GameRobotState.chassis_power_limit;

    // 如果没有获取到功率限制(裁判系统未连接),使用默认值
    if (hex_power_ctrl.max_power_limit < 10.0f || hex_power_ctrl.max_power_limit > 200.0f)
    {
        hex_power_ctrl.max_power_limit = HEX_CHASSIS_POWER_MAX_DEFAULT;
    }

    // 2. 计算可用功率 (考虑缓冲能量:缓冲越大,可用功率越多)
    float buffer_factor = hex_power_ctrl.chassis_power_buffer / HEX_CHASSIS_BUFFER_MAX;
    hex_power_ctrl.available_power = hex_power_ctrl.max_power_limit - HEX_CHASSIS_POWER_SAFETY_MARGIN +
                                     buffer_factor * HEX_CHASSIS_POWER_BUFFER_EXTRA;

    // 3. 考虑超级电容 (如果需要接入超级电容,取消下面注释)
    // if (cap_measure.cap_percent > HEX_CAPACITOR_MIN_PERCENT)
    // {
    //     if (hex_power_ctrl.cap_state)
    //     {
    //         hex_power_ctrl.available_power += HEX_CAPACITOR_BOOST_POWER;
    //     }
    //     else
    //     {
    //         hex_power_ctrl.available_power += HEX_CAPACITOR_BOOST_POWER * 0.1f;
    //     }
    // }

    // 更新最大功率记录
    if (hex_power_ctrl.total_motor_power > hex_power_ctrl.max_power_record)
    {
        hex_power_ctrl.max_power_record = hex_power_ctrl.total_motor_power;
    }
}

/**
 * @brief 计算六轮功率
 * @param motor_measure 电机测量数据数组
 * @param chassis_pid PID数组
 */
static void hex_calculate_motor_powers(motor_measure_DJ_t motor_measure[HEX_MOTOR_NUM],
                                       pid_type_def chassis_pid[HEX_MOTOR_NUM])
{
    hex_power_ctrl.total_motor_power = 0.0f;

    // for (uint8_t i = 0; i < HEX_MOTOR_NUM; i++)
    // {
    //     hex_power_ctrl.motor_power[i] = hex_motor_power_calculate(
    //         motor_measure[i].speed_rpm,
    //         chassis_pid[i].out);
    //     hex_power_ctrl.total_motor_power += hex_power_ctrl.motor_power[i];
    // }
    for (uint8_t i = 0; i < HEX_MOTOR_NUM; i++)
    {
        if (i >= 4) // 2006电机
        {
            hex_power_ctrl.motor_power[i] = HEX_2006_MOTOR_TORQUE_COEFFICIENT * chassis_pid[i].out * motor_measure[i].speed_rpm + HEX_2006_MOTOR_POWER_MODEL_K2 * motor_measure[i].speed_rpm * motor_measure[i].speed_rpm + HEX_2006_MOTOR_POWER_MODEL_K1 * chassis_pid[i].out * chassis_pid[i].out + HEX_2006_MOTOR_POWER_CONSTANT;
        }
        else // M3508电机
        {
            hex_power_ctrl.motor_power[i] = hex_motor_power_calculate(
                motor_measure[i].speed_rpm,
                chassis_pid[i].out);
        }
        hex_power_ctrl.total_motor_power += hex_power_ctrl.motor_power[i];
    }
}

/**
 * @brief 应用功率限制
 * @param chassis_pid PID数组
 * @param motor_measure 电机测量数据数组
 * @return uint8_t 是否进行了限制
 */
static uint8_t hex_apply_power_limit(pid_type_def chassis_pid[HEX_MOTOR_NUM],
                                     motor_measure_DJ_t motor_measure[HEX_MOTOR_NUM])
{
    uint8_t is_limited = 0;

    // 如果总功率超过可用功率,进行功率限制
    if (hex_power_ctrl.total_motor_power > hex_power_ctrl.available_power)
    {
        is_limited = 1;
        hex_power_ctrl.limit_count++;

        // 计算功率缩放系数 (等比例缩放)
        hex_power_ctrl.power_scale = hex_power_ctrl.available_power / hex_power_ctrl.total_motor_power;

        // 对每个电机应用功率限制
        // for (uint8_t i = 0; i < HEX_MOTOR_NUM; i++)
        // {
        //     // 计算该电机允许的目标功率
        //     float motor_target_power = hex_power_ctrl.motor_power[i] * hex_power_ctrl.power_scale;

        //     // 反算允许的电流
        //     float limited_current = hex_motor_current_limit_by_power(
        //         motor_measure[i].speed_rpm,
        //         motor_target_power,
        //         chassis_pid[i].out);

        //     // 应用电流限制
        //     chassis_pid[i].out = limited_current;
        // }
        for (uint8_t i = 0; i < HEX_MOTOR_NUM; i++)
        {
            // 计算该电机允许的目标功率
            float motor_target_power = hex_power_ctrl.motor_power[i] * hex_power_ctrl.power_scale;

            float limited_current;

            if (i >= 4) // 2006电机: 使用2006参数反算电流
            {
                // 一元二次方程: k1*I^2 + k3*w*I + (k2*w^2 + constant - P_target) = 0
                float a = HEX_2006_MOTOR_POWER_MODEL_K1;
                float b = HEX_2006_MOTOR_TORQUE_COEFFICIENT * motor_measure[i].speed_rpm;
                float c = HEX_2006_MOTOR_POWER_MODEL_K2 * motor_measure[i].speed_rpm * motor_measure[i].speed_rpm + HEX_2006_MOTOR_POWER_CONSTANT - motor_target_power;

                float discriminant = b * b - 4.0f * a * c;

                if (discriminant < 0.0f)
                {
                    limited_current = chassis_pid[i].out * 0.5f;
                }
                else
                {
                    float sqrt_disc = sqrtf(discriminant);
                    if (chassis_pid[i].out > 0.01f)
                        limited_current = (-b + sqrt_disc) / (2.0f * a);
                    else if (chassis_pid[i].out < -0.01f)
                        limited_current = (-b - sqrt_disc) / (2.0f * a);
                    else
                        limited_current = 0.0f;
                }

                // M2006电流限制: 最大10000
                if (limited_current > 10000.0f)
                    limited_current = 10000.0f;
                if (limited_current < -10000.0f)
                    limited_current = -10000.0f;
            }
            else // M3508电机: 使用原函数
            {
                limited_current = hex_motor_current_limit_by_power(
                    motor_measure[i].speed_rpm,
                    motor_target_power,
                    chassis_pid[i].out);
            }

            // 应用电流限制
            chassis_pid[i].out = limited_current;
        }
    }
    else
    {
        hex_power_ctrl.power_scale = 1.0f;
    }

    return is_limited;
}

/**
 * @brief 六轮底盘功率控制主函数
 * @param chassis_pid PID数组指针 (长度为6)
 * @param motor_measure 电机测量数据数组 (长度为6)
 * @param speed_set 速度设定值数组 (长度为6)
 * @return uint8_t 是否进行了功率限制 (0=未限制 1=已限制)
 */
uint8_t hex_chassis_power_control_process(pid_type_def chassis_pid[HEX_MOTOR_NUM],
                                          motor_measure_DJ_t motor_measure[HEX_MOTOR_NUM],
                                          int16_t speed_set[HEX_MOTOR_NUM])
{
    (void)speed_set; // 预留参数,当前实现未使用,避免编译警告

    // 1. 更新可用功率
    hex_update_available_power();

    // 2. 计算各电机功率
    hex_calculate_motor_powers(motor_measure, chassis_pid);

    // 3. 应用功率限制
    return hex_apply_power_limit(chassis_pid, motor_measure);
}

/**
 * @brief 带指针数组的功率控制处理函数 (更灵活的接口)
 * @param hex_ctrl 六轮电机控制结构体指针
 * @return uint8_t 是否进行了功率限制
 */
uint8_t hex_chassis_power_control_process_ex(hex_motor_control_t *hex_ctrl)
{
    if (hex_ctrl == NULL)
    {
        return 0;
    }

    // 1. 更新可用功率
    hex_update_available_power();

    // 2. 计算各电机功率
    hex_power_ctrl.total_motor_power = 0.0f;
    // for (uint8_t i = 0; i < HEX_MOTOR_NUM; i++)
    // {
    //     hex_power_ctrl.motor_power[i] = hex_motor_power_calculate(
    //         hex_ctrl->motor[i]->speed_rpm,
    //         hex_ctrl->pid[i]->out);
    //     hex_power_ctrl.total_motor_power += hex_power_ctrl.motor_power[i];
    // }
    for (uint8_t i = 0; i < HEX_MOTOR_NUM; i++)
    {
        if (i >= 4) // 2006电机
        {
            hex_power_ctrl.motor_power[i] = HEX_2006_MOTOR_TORQUE_COEFFICIENT * hex_ctrl->pid[i]->out * hex_ctrl->motor[i]->speed_rpm + HEX_2006_MOTOR_POWER_MODEL_K2 * hex_ctrl->motor[i]->speed_rpm * hex_ctrl->motor[i]->speed_rpm + HEX_2006_MOTOR_POWER_MODEL_K1 * hex_ctrl->pid[i]->out * hex_ctrl->pid[i]->out + HEX_2006_MOTOR_POWER_CONSTANT;
        }
        else // M3508电机
        {
            hex_power_ctrl.motor_power[i] = hex_motor_power_calculate(
                hex_ctrl->motor[i]->speed_rpm,
                hex_ctrl->pid[i]->out);
        }
        hex_power_ctrl.total_motor_power += hex_power_ctrl.motor_power[i];
    }

    // 3. 应用功率限制
    uint8_t is_limited = 0;
    if (hex_power_ctrl.total_motor_power > hex_power_ctrl.available_power)
    {
        is_limited = 1;
        hex_power_ctrl.limit_count++;

        hex_power_ctrl.power_scale = hex_power_ctrl.available_power / hex_power_ctrl.total_motor_power;

        // for (uint8_t i = 0; i < HEX_MOTOR_NUM; i++)
        // {
        //     float motor_target_power = hex_power_ctrl.motor_power[i] * hex_power_ctrl.power_scale;

        //     float limited_current = hex_motor_current_limit_by_power(
        //         hex_ctrl->motor[i]->speed_rpm,
        //         motor_target_power,
        //         hex_ctrl->pid[i]->out);

        //     hex_ctrl->pid[i]->out = limited_current;
        //     hex_ctrl->limited_current[i] = limited_current;
        // }
        for (uint8_t i = 0; i < HEX_MOTOR_NUM; i++)
        {
            float motor_target_power = hex_power_ctrl.motor_power[i] * hex_power_ctrl.power_scale;

            float limited_current;

            if (i >= 4) // 2006电机: 使用2006参数反算电流
            {
                float a = HEX_2006_MOTOR_POWER_MODEL_K1;
                float b = HEX_2006_MOTOR_TORQUE_COEFFICIENT * hex_ctrl->motor[i]->speed_rpm;
                float c = HEX_2006_MOTOR_POWER_MODEL_K2 * hex_ctrl->motor[i]->speed_rpm * hex_ctrl->motor[i]->speed_rpm + HEX_2006_MOTOR_POWER_CONSTANT - motor_target_power;

                float discriminant = b * b - 4.0f * a * c;

                if (discriminant < 0.0f)
                {
                    limited_current = hex_ctrl->pid[i]->out * 0.5f;
                }
                else
                {
                    float sqrt_disc = sqrtf(discriminant);
                    if (hex_ctrl->pid[i]->out > 0.01f)
                        limited_current = (-b + sqrt_disc) / (2.0f * a);
                    else if (hex_ctrl->pid[i]->out < -0.01f)
                        limited_current = (-b - sqrt_disc) / (2.0f * a);
                    else
                        limited_current = 0.0f;
                }

                if (limited_current > 10000.0f)
                    limited_current = 10000.0f;
                if (limited_current < -10000.0f)
                    limited_current = -10000.0f;
            }
            else // M3508电机: 使用原函数
            {
                limited_current = hex_motor_current_limit_by_power(
                    hex_ctrl->motor[i]->speed_rpm,
                    motor_target_power,
                    hex_ctrl->pid[i]->out);
            }

            hex_ctrl->pid[i]->out = limited_current;
            hex_ctrl->limited_current[i] = limited_current;
        }
    }
    else
    {
        hex_power_ctrl.power_scale = 1.0f;
        for (uint8_t i = 0; i < HEX_MOTOR_NUM; i++)
        {
            hex_ctrl->limited_current[i] = hex_ctrl->pid[i]->out;
        }
    }

    return is_limited;
}

/**
 * @brief 获取当前可用功率
 * @return float 可用功率值 (W)
 */
float hex_chassis_get_available_power(void)
{
    return hex_power_ctrl.available_power;
}

/**
 * @brief 获取当前总电机功率
 * @return float 总功率值 (W)
 */
float hex_chassis_get_total_motor_power(void)
{
    return hex_power_ctrl.total_motor_power;
}

/**
 * @brief 设置超级电容状态
 * @param state 0=正常 1=爆发模式
 */
void hex_chassis_set_cap_state(uint8_t state)
{
    hex_power_ctrl.cap_state = state;
}

/**
 * @brief 获取功率控制状态
 * @param ctrl 功率控制结构体指针 (用于外部查看状态)
 * @return uint8_t 操作状态 0=成功
 */
uint8_t hex_chassis_get_power_ctrl_state(hex_chassis_power_control_t *ctrl)
{
    if (ctrl == NULL)
    {
        return 1;
    }

    memcpy(ctrl, &hex_power_ctrl, sizeof(hex_chassis_power_control_t));
    return 0;
}

/**
 * @brief S曲线加减速计算
 * @param current_speed 当前速度
 * @param target_speed 目标速度
 * @param max_speed 最大速度
 * @return float S曲线调整后的速度增量
 *
 * S曲线原理:
 * - 低速阶段:加速较慢 (防止打滑)
 * - 中速阶段:加速最快
 * - 高速阶段:加速减慢 (接近目标速度时平滑过渡)
 */
float hex_s_curve_speed_calc(float current_speed, float target_speed, float max_speed)
{
    float speed_diff = target_speed - current_speed;
    float speed_ratio = fabsf(current_speed) / max_speed;

    // S曲线因子:0.5 ~ 1.5 之间变化
    // 低速时因子小(缓启动),中速时因子大,高速接近目标时因子小
    float s_factor;

    if (speed_ratio < HEX_S_CURVE_ACCEL_RATIO)
    {
        // 加速段:从0.5逐渐增加到1.5
        s_factor = 0.5f + speed_ratio / HEX_S_CURVE_ACCEL_RATIO;
    }
    else if (speed_ratio < (1.0f - HEX_S_CURVE_DECEL_RATIO))
    {
        // 匀速段:最大加速因子
        s_factor = HEX_S_CURVE_MAX_FACTOR;
    }
    else
    {
        // 减速段:逐渐减小到0.5
        s_factor = 0.5f + (1.0f - speed_ratio) / HEX_S_CURVE_DECEL_RATIO;
    }

    // 根据S曲线因子调整速度增量
    return speed_diff * s_factor;
}

/**
 * @brief 带S曲线的速度斜坡控制
 * @param current 当前速度
 * @param target 目标速度
 * @param accel_step 加速度步进
 * @param decel_step 减速度步进
 * @param max_speed 最大速度
 * @return int16_t 下一时刻速度
 */
int16_t hex_speed_ramp_s_curve(int16_t current, int16_t target,
                               int16_t accel_step, int16_t decel_step,
                               int16_t max_speed)
{
    int16_t output = current;
    int16_t diff = target - current;

    if (diff == 0)
    {
        return target;
    }

    // 计算速度比率
    float speed_ratio = (float)fabs(current) / (float)fabs(max_speed);
    if (speed_ratio > 1.0f)
        speed_ratio = 1.0f;

    // 计算S曲线因子
    float s_factor;
    if (speed_ratio < 0.3f)
    {
        s_factor = 1.0f + speed_ratio / 0.3f * 0.5f; // 1.0~1.5
    }
    else if (speed_ratio < 0.7f)
    {
        s_factor = 1.5f; // 最大加速因子
    }
    else
    {
        s_factor = 1.5f - (speed_ratio - 0.7f) / 0.3f * 1.0f;
    }

    // 根据方向选择加速或减速步进
    int16_t step = (diff > 0) ? accel_step : decel_step;
    int16_t adjusted_step = (int16_t)(step * s_factor);

    // 限制步进范围
    if (adjusted_step < 1)
        adjusted_step = 1;
    if (adjusted_step > abs(diff))
        adjusted_step = abs(diff);

    // 应用步进
    if (diff > 0)
        output += adjusted_step;
    else
        output -= adjusted_step;

    return output;
}

/**
 * @brief 六轮底盘独立速度斜坡控制
 * @param current 当前速度数组 (长度6)
 * @param target 目标速度数组 (长度6)
 * @param accel_step 加速度步进
 * @param decel_step 减速度步进
 * @param max_speed 最大速度
 * @param output 输出速度数组 (长度6)
 */
void hex_chassis_speed_ramp_control(int16_t current[HEX_MOTOR_NUM],
                                    int16_t target[HEX_MOTOR_NUM],
                                    int16_t accel_step, int16_t decel_step,
                                    int16_t max_speed,
                                    int16_t output[HEX_MOTOR_NUM])
{
    for (uint8_t i = 0; i < HEX_MOTOR_NUM; i++)
    {
        output[i] = hex_speed_ramp_s_curve(current[i], target[i],
                                           accel_step, decel_step,
                                           max_speed);
    }
}

/**
 * @brief 获取单个电机的功率占比
 * @param motor_idx 电机索引 (0-5)
 * @return float 功率占比 (0.0~1.0)
 */
float hex_motor_get_power_ratio(uint8_t motor_idx)
{
    if (motor_idx >= HEX_MOTOR_NUM)
    {
        return 0.0f;
    }

    if (hex_power_ctrl.total_motor_power < 0.01f)
    {
        return 0.0f;
    }

    return hex_power_ctrl.motor_power[motor_idx] / hex_power_ctrl.total_motor_power;
}

/**
 * @brief 六轮底盘功率统计 (用于调试)
 * @param info 信息字符串缓冲区
 * @param len 缓冲区长度
 */
void hex_chassis_power_debug_info(char *info, uint16_t len)
{
    if (info == NULL || len < 200)
    {
        return;
    }

    snprintf(info, len,
             "P:%.1f/%.1fW Buf:%.0f Scale:%.2f L:%lu",
             hex_power_ctrl.total_motor_power,
             hex_power_ctrl.available_power,
             hex_power_ctrl.chassis_power_buffer,
             hex_power_ctrl.power_scale,
             hex_power_ctrl.limit_count);
}
