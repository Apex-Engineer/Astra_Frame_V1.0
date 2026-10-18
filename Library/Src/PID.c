#include "PID.h"

/**
 * @brief  PID计算
 * @param  pid  PID结构体地址
 * @param  Kp  P
 * @param  Ki  I
 * @param  Kd  D
 * @param  i_max  I最大值
 * @param  max_out  最大输出值
 */
void PID_init(pid_type_def *pid, float Kp, float Ki, float Kd, float i_max, float max_out)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->i_max = i_max;
    pid->out_max = max_out;
    pid->error = pid->last_error = pid->last_last_error = 0.0f;
}

/**
 * @brief  PID计算（位置式PID）
 * @param  pid  PID结构体地址
 * @param  ref  实际值
 * @param  set  预期值
 * @param  max  反馈最大值，如无限制则为0
 * @retval PID计算结果
 */
float PID_Calc(pid_type_def *pid, float ref, float set, int max)
{
    pid->fdb = ref;
    pid->set = set;
    pid->last_error = pid->error;
    pid->error = set - ref;

    if (max != 0)
    {
        if (pid->error >= (max >> 1))
        {
            pid->error -= max;
        }
        if (pid->error < -(max >> 1))
        {
            pid->error += max;
        }
    }

    pid->pout = pid->error * pid->Kp;
    pid->iout += pid->error * pid->Ki;
    pid->dout = (pid->error - pid->last_error) * pid->Kd;

    if (pid->iout >= pid->i_max)
    {
        pid->iout = pid->i_max;
    }
    if (pid->iout <= -pid->i_max)
    {
        pid->iout = -pid->i_max;
    }

    pid->out = pid->pout + pid->iout + pid->dout;

    if (pid->out >= pid->out_max)
    {
        pid->out = pid->out_max;
    }
    if (pid->out <= -pid->out_max)
    {
        pid->out = -pid->out_max;
    }
    return pid->out;
}

/**
 * @brief  PID计算（增量式PID）
 * @param  pid  PID结构体地址
 * @param  ref  实际值
 * @param  set  预期值
 * @param  max  反馈最大值，如无限制则为0
 * @retval PID计算结果
 */
float PID_Calc_Inc(pid_type_def *pid, float ref, float set, int max)
{
    pid->fdb = ref;
    pid->set = set;
    pid->last_last_error = pid->last_error;
    pid->last_error = pid->error;
    pid->error = set - ref;

    // 角度环绕处理（与位置式一致）
    if (max != 0)
    {
        if (pid->error >= (max >> 1))
        {
            pid->error -= max;
        }
        if (pid->error < -(max >> 1))
        {
            pid->error += max;
        }
    }

    float delta_u = pid->Kp * (pid->error - pid->last_error) + pid->Ki * pid->error +
                    pid->Kd * (pid->error - 2 * pid->last_error + pid->last_last_error);
    pid->out += delta_u;

    // 输出限幅
    if (pid->out > pid->out_max)
    {
        pid->out = pid->out_max;
    }
    if (pid->out < -pid->out_max)
    {
        pid->out = -pid->out_max;
    }
    return pid->out;
}

/* ============================================================
 *  模糊 PID 控制器
 *
 *  结构: e, ec → 模糊推理 → ΔKp, ΔKi, ΔKd → 修正 PID 参数 → PID 输出
 *
 *  复用 Fuzzy.c 中的模糊推理核心:
 *    - Fuzzy_init_ex(): 使用自定义规则表初始化三个模糊调参器
 *    - Fuzzy_Calc_Raw(): 直接传入 e, ec, 不更新模糊器内部状态
 * ============================================================ */

/* Kp 整定规则表: 大误差时增大 Kp 加快响应, 小误差时减小 Kp 抑制抖动
 *        EC: NB  NM  NS  ZO  PS  PM  PB
 *  E: */
static const int FUZZY_PID_RULE_KP[7][7] = {
    {6, 6, 5, 5, 4, 3, 3}, // NB
    {6, 5, 5, 4, 4, 3, 2}, // NM
    {5, 5, 4, 4, 3, 2, 2}, // NS
    {5, 4, 4, 3, 2, 2, 1}, // ZO
    {4, 4, 3, 2, 2, 1, 1}, // PS
    {4, 3, 2, 2, 1, 1, 0}, // PM
    {3, 3, 2, 1, 1, 0, 0}, // PB
};

/* Ki 整定规则表: 小误差时增大 Ki 消除静差, 大误差时减小 Ki 防止积分饱和
 *        EC: NB  NM  NS  ZO  PS  PM  PB
 *  E: */
static const int FUZZY_PID_RULE_KI[7][7] = {
    {0, 0, 1, 1, 2, 3, 3}, // NB
    {0, 1, 1, 2, 3, 3, 4}, // NM
    {1, 1, 2, 3, 3, 4, 4}, // NS
    {1, 2, 3, 3, 4, 5, 5}, // ZO
    {2, 3, 3, 4, 4, 5, 5}, // PS
    {3, 3, 4, 5, 5, 6, 6}, // PM
    {3, 4, 4, 5, 6, 6, 6}, // PB
};

/* Kd 整定规则表: 误差中等且变化快时增大 Kd 抑制超调, 稳态时适度 Kd 抑制抖动
 *        EC: NB  NM  NS  ZO  PS  PM  PB
 *  E: */
static const int FUZZY_PID_RULE_KD[7][7] = {
    {4, 4, 3, 3, 2, 1, 1}, // NB
    {4, 3, 3, 2, 2, 1, 1}, // NM
    {3, 3, 2, 2, 1, 1, 2}, // NS
    {3, 2, 2, 1, 1, 2, 3}, // ZO
    {2, 2, 1, 1, 2, 3, 3}, // PS
    {2, 1, 1, 2, 3, 3, 4}, // PM
    {1, 1, 2, 3, 3, 4, 4}, // PB
};

/**
 * @brief  初始化模糊 PID 控制器
 * @param  fpid     模糊 PID 结构体
 * @param  Kp0      PID 初始 Kp
 * @param  Ki0      PID 初始 Ki
 * @param  Kd0      PID 初始 Kd
 * @param  i_max    积分上限
 * @param  max_out  PID 输出上限
 * @param  Kp_max   Kp 最大修正量 (模糊论域 ±6 映射到 ±Kp_max)
 * @param  Ki_max   Ki 最大修正量
 * @param  Kd_max   Kd 最大修正量
 * @param  Ke       误差量化因子
 * @param  Kec      误差变化率量化因子
 *
 * @note   模糊输出论域为 [-6, 6], 因此 Ku = Kx_max / 6
 */
void Fuzzy_PID_init(fuzzy_pid_type_def *fpid, float Kp0, float Ki0, float Kd0, float i_max, float max_out, float Kp_max,
                    float Ki_max, float Kd_max, float Ke, float Kec)
{
    // 初始化基础 PID
    PID_init(&fpid->pid, Kp0, Ki0, Kd0, i_max, max_out);

    // 保存初始参数和修正上限
    fpid->Kp0 = Kp0;
    fpid->Ki0 = Ki0;
    fpid->Kd0 = Kd0;
    fpid->Kp_max = Kp_max;
    fpid->Ki_max = Ki_max;
    fpid->Kd_max = Kd_max;
    fpid->delta_kp = 0.0f;
    fpid->delta_ki = 0.0f;
    fpid->delta_kd = 0.0f;

    // 初始化三个模糊调参器, 复用 Fuzzy.c 的推理核心
    // 输出限幅 = Kx_max, Ku = Kx_max / 6 (把模糊论域 ±6 映射到实际修正量范围)
    Fuzzy_init_ex(&fpid->fuzzy_kp, Ke, Kec, Kp_max / 6.0f, Kp_max, FUZZY_PID_RULE_KP);
    Fuzzy_init_ex(&fpid->fuzzy_ki, Ke, Kec, Ki_max / 6.0f, Ki_max, FUZZY_PID_RULE_KI);
    Fuzzy_init_ex(&fpid->fuzzy_kd, Ke, Kec, Kd_max / 6.0f, Kd_max, FUZZY_PID_RULE_KD);
}

/**
 * @brief  模糊 PID 计算 (位置式)
 * @param  fpid  模糊 PID 结构体
 * @param  ref   实际值
 * @param  set   设定值
 * @param  max   反馈最大值 (如角度环绕), 无限制传 0
 * @retval PID 计算结果
 *
 * @note   先根据 e, ec 用模糊推理在线调整 Kp/Ki/Kd, 再用位置式 PID 计算输出
 */
float Fuzzy_PID_Calc(fuzzy_pid_type_def *fpid, float ref, float set, int max)
{
    // 1. 计算当前误差 (先做环绕处理, 与 PID_Calc 保持一致)
    float error = set - ref;
    if (max != 0)
    {
        if (error >= (max >> 1))
        {
            error -= max;
        }
        if (error < -(max >> 1))
        {
            error += max;
        }
    }

    // 2. 计算误差变化率 (使用 pid 结构体中保存的上次误差)
    float error_change = error - fpid->pid.last_error;

    // 3. 复用 Fuzzy.c 核心, 计算三个 PID 参数的修正量
    //    传入的是物理误差, 由模糊器内部乘以 Ke/Kec 量化
    fpid->delta_kp = Fuzzy_Calc_Raw(&fpid->fuzzy_kp, error, error_change);
    fpid->delta_ki = Fuzzy_Calc_Raw(&fpid->fuzzy_ki, error, error_change);
    fpid->delta_kd = Fuzzy_Calc_Raw(&fpid->fuzzy_kd, error, error_change);

    // 4. 在线修正 PID 参数
    fpid->pid.Kp = fpid->Kp0 + fpid->delta_kp;
    fpid->pid.Ki = fpid->Ki0 + fpid->delta_ki;
    fpid->pid.Kd = fpid->Kd0 + fpid->delta_kd;

    // 5. 用更新后的参数执行位置式 PID
    return PID_Calc(&fpid->pid, ref, set, max);
}