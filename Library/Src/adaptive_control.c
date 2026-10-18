#include "adaptive_control.h"

/* ============================================================================
 *                        自适应PID控制器实现
 * ============================================================================ */

/**
 * @brief  初始化自适应PID控制器
 *
 *  设置初始PID参数、自适应参数范围、学习率等
 *  所有内部状态清零，参数恢复到初始值
 */
void AdaptivePID_Init(adaptive_pid_type_def *apid, float Kp, float Ki, float Kd, float i_max, float out_max, float dt)
{
    /* ---- 保存初始参数（自适应的中心点） ---- */
    apid->Kp_init = Kp;
    apid->Ki_init = Ki;
    apid->Kd_init = Kd;

    apid->Kp = Kp;
    apid->Ki = Ki;
    apid->Kd = Kd;

    /* ---- PID状态清零 ---- */
    apid->pout = 0.0f;
    apid->iout = 0.0f;
    apid->dout = 0.0f;

    apid->error = 0.0f;
    apid->last_error = 0.0f;
    apid->last_last_error = 0.0f;
    apid->error_integral = 0.0f;

    apid->set = 0.0f;
    apid->fdb = 0.0f;
    apid->out = 0.0f;

    /* ---- 限幅 ---- */
    apid->i_max = i_max;
    apid->out_max = out_max;

    /* ---- 自适应参数（默认值） ---- */
    apid->gamma_p = ADAP_PID_DEFAULT_GAMMA_P;
    apid->gamma_i = ADAP_PID_DEFAULT_GAMMA_I;
    apid->gamma_d = ADAP_PID_DEFAULT_GAMMA_D;
    apid->dead_zone = ADAP_PID_DEFAULT_DEAD_ZONE;

    /* ---- 参数范围（基于初始值的倍数） ---- */
    apid->Kp_max = Kp * ADAP_PID_DEFAULT_KP_RANGE;
    apid->Kp_min = Kp / ADAP_PID_DEFAULT_KP_RANGE;
    apid->Ki_max = Ki * ADAP_PID_DEFAULT_KI_RANGE;
    apid->Ki_min = Ki / ADAP_PID_DEFAULT_KI_RANGE;
    apid->Kd_max = Kd * ADAP_PID_DEFAULT_KD_RANGE;
    apid->Kd_min = Kd / ADAP_PID_DEFAULT_KD_RANGE;

    /* ---- 参数变化率限幅 ---- */
    apid->dKp_max = ADAP_PID_DEFAULT_DKP_MAX;
    apid->dKi_max = ADAP_PID_DEFAULT_DKI_MAX;
    apid->dKd_max = ADAP_PID_DEFAULT_DKD_MAX;

    /* ---- 控制周期 ---- */
    apid->dt = (dt > 0.0f) ? dt : ADAP_PID_DT_DEFAULT;

    /* ---- 默认启用自适应 ---- */
    apid->adaptive_enable = 1;
}

/**
 * @brief  自适应PID计算（位置式）
 *
 *  【MIT规则离散化推导】
 *
 *  假设被控对象近似为一阶：y(k) = a·y(k-1) + b·u(k-1)
 *  控制器：u(k) = Kp·e(k) + Ki·Σe(j) + Kd·Δe(k)
 *
 *  MIT规则要求参数向性能指标 J = ½e² 的负梯度方向调整：
 *    Δθ = -γ · ∂J/∂θ = γ · e · ∂y/∂θ
 *
 *  对于Kp：
 *    ∂y(k)/∂Kp = b · ∂u(k-1)/∂Kp = b · e(k-1)
 *    =>  ΔKp = γp · e(k) · e(k-1)        （b吸收到γp中）
 *
 *  对于Ki：
 *    ∂y(k)/∂Ki = b · Σe(j)
 *    =>  ΔKi = γi · e(k) · Σe(j)
 *
 *  对于Kd：
 *    ∂y(k)/∂Kd = b · Δe(k-1)
 *    =>  ΔKd = γd · e(k) · Δe(k-1)
 *
 *  【物理意义】
 *  - e(k)·e(k-1) > 0：误差持续同号（系统响应慢）→ Kp增大
 *  - e(k)·e(k-1) < 0：误差变号（超调/振荡）→ Kp减小
 *  - e(k)·Σe(j) > 0：存在稳态误差 → Ki增大
 *  - e(k)·Δe(k-1) 大：误差变化剧烈（振荡）→ Kd增大
 */
float AdaptivePID_Calc(adaptive_pid_type_def *apid, float ref, float set, int max)
{
    /* ========== Step 1: 误差计算（同标准PID） ========== */
    apid->fdb = ref;
    apid->set = set;

    /* 移位误差历史 */
    apid->last_last_error = apid->last_error;
    apid->last_error = apid->error;
    apid->error = set - ref;

    /* 角度环绕处理 */
    if (max != 0)
    {
        if (apid->error >= (max >> 1))
        {
            apid->error -= max;
        }
        if (apid->error < -(max >> 1))
        {
            apid->error += max;
        }
    }

    /* ========== Step 2: 误差积分（带抗饱和） ========== */
    /* 仅在输出未饱和时积分，防止积分饱和 */
    float new_integral = apid->error_integral + apid->error * apid->dt;
    if (new_integral > apid->i_max / (apid->Ki + 1e-6f))
    {
        new_integral = apid->i_max / (apid->Ki + 1e-6f);
    }
    if (new_integral < -apid->i_max / (apid->Ki + 1e-6f))
    {
        new_integral = -apid->i_max / (apid->Ki + 1e-6f);
    }
    apid->error_integral = new_integral;

    /* ========== Step 3: 标准PID计算 ========== */
    apid->pout = apid->error * apid->Kp;
    apid->iout = apid->error_integral * apid->Ki;
    apid->dout = (apid->error - apid->last_error) * apid->Kd / apid->dt;

    /* 积分限幅 */
    if (apid->iout > apid->i_max)
    {
        apid->iout = apid->i_max;
    }
    if (apid->iout < -apid->i_max)
    {
        apid->iout = -apid->i_max;
    }

    apid->out = apid->pout + apid->iout + apid->dout;

    /* 输出限幅 */
    if (apid->out >= apid->out_max)
    {
        apid->out = apid->out_max;
    }
    else if (apid->out <= -apid->out_max)
    {
        apid->out = -apid->out_max;
    }

    /* ========== Step 4: MIT规则自适应参数调整 ========== */
    if (apid->adaptive_enable && (fabsf(apid->error) > apid->dead_zone))
    {
        /* ---- 计算灵敏度信号（MIT规则的核心） ---- */
        /* 灵敏度 ≈ 输出对参数的偏导，用历史误差近似 */

        /* Kp灵敏度：∂y/∂Kp ∝ e(k-1) */
        float sens_p = apid->last_error;

        /* Ki灵敏度：∂y/∂Ki ∝ ∫e(t)dt */
        float sens_i = apid->error_integral;

        /* Kd灵敏度：∂y/∂Kd ∝ Δe(k-1) = e(k-1) - e(k-2) */
        float sens_d = apid->last_error - apid->last_last_error;

        /* ---- MIT规则：参数变化 = 学习率 × 当前误差 × 灵敏度 ---- */
        float dKp = apid->gamma_p * apid->error * sens_p;
        float dKi = apid->gamma_i * apid->error * sens_i;
        float dKd = apid->gamma_d * apid->error * sens_d;

        /* ---- 变化率限幅（防止参数突变） ---- */
        dKp = adaptive_control_clampf(dKp, -apid->dKp_max, apid->dKp_max);
        dKi = adaptive_control_clampf(dKi, -apid->dKi_max, apid->dKi_max);
        dKd = adaptive_control_clampf(dKd, -apid->dKd_max, apid->dKd_max);

        /* ---- 更新参数 ---- */
        apid->Kp += dKp;
        apid->Ki += dKi;
        apid->Kd += dKd;

        /* ---- 范围限幅（防止参数发散） ---- */
        apid->Kp = adaptive_control_clampf(apid->Kp, apid->Kp_min, apid->Kp_max);
        apid->Ki = adaptive_control_clampf(apid->Ki, apid->Ki_min, apid->Ki_max);
        apid->Kd = adaptive_control_clampf(apid->Kd, apid->Kd_min, apid->Kd_max);
    }

    return apid->out;
}

/**
 * @brief  自适应PID计算（增量式）
 *
 *  与位置式的区别：
 *  - 位置式：u(k) = Kp·e(k) + Ki·Σe(j) + Kd·Δe(k)
 *  - 增量式：Δu(k) = Kp·Δe(k) + Ki·e(k) + Kd·Δ²e(k)
 *            u(k) = u(k-1) + Δu(k)
 *
 *  增量式的优点：
 *  - 无积分饱和问题
 *  - 参数变化时输出更平滑
 *  - 更适合执行器是积分特性（如步进电机）的系统
 *
 *  自适应调整律与位置式相同（MIT规则）
 */
float AdaptivePID_Calc_Inc(adaptive_pid_type_def *apid, float ref, float set, int max)
{
    /* ========== Step 1: 误差计算（同位置式） ========== */
    apid->fdb = ref;
    apid->set = set;

    apid->last_last_error = apid->last_error;
    apid->last_error = apid->error;
    apid->error = set - ref;

    /* 角度环绕处理 */
    if (max != 0)
    {
        if (apid->error >= (max >> 1))
        {
            apid->error -= max;
        }
        if (apid->error < -(max >> 1))
        {
            apid->error += max;
        }
    }

    /* ========== Step 2: 误差积分（用于Ki自适应和输出） ========== */
    apid->error_integral += apid->error * apid->dt;

    /* ========== Step 3: 增量式PID计算 ========== */
    /* Δu = Kp·(e(k)-e(k-1)) + Ki·e(k)·dt + Kd·(e(k)-2e(k-1)+e(k-2))/dt */
    float delta_e = apid->error - apid->last_error;
    float delta2_e = apid->error - 2.0f * apid->last_error + apid->last_last_error;

    float delta_u = apid->Kp * delta_e + apid->Ki * apid->error * apid->dt + apid->Kd * delta2_e / apid->dt;

    apid->out += delta_u;

    /* 输出限幅 */
    apid->out = adaptive_control_clampf(apid->out, -apid->out_max, apid->out_max);

    /* 保存输出分量（用于调试/观察） */
    apid->pout = apid->Kp * delta_e;
    apid->iout = apid->Ki * apid->error * apid->dt;
    apid->dout = apid->Kd * delta2_e / apid->dt;

    /* ========== Step 4: MIT规则自适应参数调整 ========== */
    if (apid->adaptive_enable && (fabsf(apid->error) > apid->dead_zone))
    {
        /* 灵敏度信号 */
        float sens_p = apid->last_error;
        float sens_i = apid->error_integral;
        float sens_d = apid->last_error - apid->last_last_error;

        /* MIT规则 */
        float dKp = apid->gamma_p * apid->error * sens_p;
        float dKi = apid->gamma_i * apid->error * sens_i;
        float dKd = apid->gamma_d * apid->error * sens_d;

        /* 变化率限幅 */
        dKp = adaptive_control_clampf(dKp, -apid->dKp_max, apid->dKp_max);
        dKi = adaptive_control_clampf(dKi, -apid->dKi_max, apid->dKi_max);
        dKd = adaptive_control_clampf(dKd, -apid->dKd_max, apid->dKd_max);

        /* 更新参数 */
        apid->Kp += dKp;
        apid->Ki += dKi;
        apid->Kd += dKd;

        /* 范围限幅 */
        apid->Kp = adaptive_control_clampf(apid->Kp, apid->Kp_min, apid->Kp_max);
        apid->Ki = adaptive_control_clampf(apid->Ki, apid->Ki_min, apid->Ki_max);
        apid->Kd = adaptive_control_clampf(apid->Kd, apid->Kd_min, apid->Kd_max);
    }

    return apid->out;
}

/**
 * @brief  在线修改自适应学习率
 */
void AdaptivePID_SetAdaptiveGain(adaptive_pid_type_def *apid, float gamma_p, float gamma_i, float gamma_d)
{
    apid->gamma_p = gamma_p;
    apid->gamma_i = gamma_i;
    apid->gamma_d = gamma_d;
}

/**
 * @brief  在线修改参数变化范围
 */
void AdaptivePID_SetParamRange(adaptive_pid_type_def *apid, float Kp_range, float Ki_range, float Kd_range)
{
    if (Kp_range > 1.0f)
    {
        apid->Kp_max = apid->Kp_init * Kp_range;
        apid->Kp_min = apid->Kp_init / Kp_range;
    }
    if (Ki_range > 1.0f)
    {
        apid->Ki_max = apid->Ki_init * Ki_range;
        apid->Ki_min = apid->Ki_init / Ki_range;
    }
    if (Kd_range > 1.0f)
    {
        apid->Kd_max = apid->Kd_init * Kd_range;
        apid->Kd_min = apid->Kd_init / Kd_range;
    }
}

/**
 * @brief  使能/禁用自适应
 */
void AdaptivePID_Enable(adaptive_pid_type_def *apid, uint8_t enable)
{
    apid->adaptive_enable = enable;
}

/**
 * @brief  重置自适应PID内部状态
 */
void AdaptivePID_Reset(adaptive_pid_type_def *apid)
{
    apid->error = 0.0f;
    apid->last_error = 0.0f;
    apid->last_last_error = 0.0f;
    apid->error_integral = 0.0f;
    apid->pout = 0.0f;
    apid->iout = 0.0f;
    apid->dout = 0.0f;
    apid->out = 0.0f;
}

/**
 * @brief  将参数恢复到初始值
 */
void AdaptivePID_RestoreDefaults(adaptive_pid_type_def *apid)
{
    apid->Kp = apid->Kp_init;
    apid->Ki = apid->Ki_init;
    apid->Kd = apid->Kd_init;
    AdaptivePID_Reset(apid);
}

/* ============================================================================
 *                        MRAC控制器实现
 * ============================================================================ */

/**
 * @brief  初始化MRAC控制器
 *
 *  参考模型：τ_m · dy_m/dt + y_m = K_m · r
 *  离散化（欧拉法）：y_m(k+1) = y_m(k) + dt · [-(1/τ_m)·y_m(k) + (K_m/τ_m)·r(k)]
 */
void MRAC_Init(mrac_type_def *mrac, float tau_m, float K_m, float gamma1, float gamma2, float dt, float out_max)
{
    /* ---- 参考模型参数 ---- */
    mrac->tau_m = (tau_m > 0.0f) ? tau_m : MRAC_DEFAULT_TAU_M;
    mrac->K_m = K_m;

    /* ---- 参考模型状态清零 ---- */
    mrac->ym = 0.0f;
    mrac->ym_dot = 0.0f;

    /* ---- 控制器参数（初始值） ---- */
    /* theta1的初始猜测：假设系统增益K≈K_m，则theta1*≈K_m/K≈1 */
    mrac->theta1 = 1.0f;
    /* theta2的初始猜测：假设系统极点a≈1/τ_m，则theta2*≈(a_m-a)/K≈0 */
    mrac->theta2 = 0.0f;

    /* ---- 自适应增益 ---- */
    mrac->gamma1 = gamma1;
    mrac->gamma2 = gamma2;

    /* ---- 参数限幅 ---- */
    mrac->theta1_max = MRAC_DEFAULT_THETA1_MAX;
    mrac->theta1_min = MRAC_DEFAULT_THETA1_MIN;
    mrac->theta2_max = MRAC_DEFAULT_THETA2_MAX;
    mrac->theta2_min = MRAC_DEFAULT_THETA2_MIN;

    /* ---- 输入/输出 ---- */
    mrac->set = 0.0f;
    mrac->fdb = 0.0f;
    mrac->out = 0.0f;
    mrac->out_max = out_max;
    mrac->error = 0.0f;

    /* ---- 控制周期 ---- */
    mrac->dt = (dt > 0.0f) ? dt : MRAC_DT_DEFAULT;

    /* ---- σ修正（默认关闭） ---- */
    mrac->sigma = 0.0f;
}

/**
 * @brief  MRAC计算主函数
 *
 *  【算法流程】
 *  1. 更新参考模型输出 y_m
 *  2. 计算跟踪误差 e = y_m - y_p
 *  3. 用李雅普诺夫调整律更新 θ₁, θ₂
 *  4. 计算控制量 u = θ₁·r - θ₂·y_p
 *  5. 输出限幅
 *
 *  【李雅普诺夫稳定性设计】
 *
 *  被控对象（一阶）：τ·ẏ_p + y_p = K·u
 *  改写为：ẏ_p = -a·y_p + b·u，其中 a=1/τ, b=K/τ
 *
 *  参考模型：τ_m·ẏ_m + y_m = K_m·r
 *  改写为：ẏ_m = -a_m·y_m + b_m·r，其中 a_m=1/τ_m, b_m=K_m/τ_m
 *
 *  控制器：u = θ₁·r - θ₂·y_p
 *
 *  代入被控对象：
 *    ẏ_p = -a·y_p + b·(θ₁·r - θ₂·y_p)
 *        = -(a + b·θ₂)·y_p + b·θ₁·r
 *
 *  跟踪误差 e = y_m - y_p：
 *    ė = ẏ_m - ẏ_p
 *      = (-a_m·y_m + b_m·r) - (-(a+b·θ₂)·y_p + b·θ₁·r)
 *      = -a_m·y_m + b_m·r + (a+b·θ₂)·y_p - b·θ₁·r
 *      = -a_m·(y_m - y_p) - a_m·y_p + (a+b·θ₂)·y_p + (b_m - b·θ₁)·r
 *      = -a_m·e + (a - a_m + b·θ₂)·y_p + (b_m - b·θ₁)·r
 *
 *  最优参数（使ė = -a_m·e）：
 *    θ₁* = b_m / b = K_m / K
 *    θ₂* = (a_m - a) / b = (K_m/τ_m - K/τ) · (τ/K)
 *
 *  定义参数误差：θ̃₁ = θ₁* - θ₁,  θ̃₂ = θ₂* - θ₂
 *
 *  李雅普诺夫函数：
 *    V = ½·e² + (b/2γ₁)·θ̃₁² + (b/2γ₂)·θ̃₂²
 *
 *  求导：
 *    V̇ = e·ė - (b/γ₁)·θ̃₁·θ̇₁ - (b/γ₂)·θ̃₂·θ̇₂
 *      = e·[-a_m·e + b·(θ₂-θ₂*)·y_p + b·(θ₁*-θ₁)·r] - (b/γ₁)·θ̃₁·θ̇₁ - (b/γ₂)·θ̃₂·θ̇₂
 *      = -a_m·e² + b·θ̃₂·e·y_p + b·θ̃₁·e·r - (b/γ₁)·θ̃₁·θ̇₁ - (b/γ₂)·θ̃₂·θ̇₂
 *      = -a_m·e² + b·θ̃₁·(e·r - θ̇₁/γ₁) + b·θ̃₂·(e·y_p - θ̇₂/γ₂)
 *
 *  选择调整律使括号内为零：
 *    θ̇₁ = γ₁·e·r
 *    θ̇₂ = γ₂·e·y_p
 *
 *  则 V̇ = -a_m·e² ≤ 0，全局渐近稳定！
 *
 *  【σ修正（鲁棒性增强）】
 *  标准MRAC在无持续激励时参数可能漂移。σ修正在调整律中加入泄漏项：
 *    θ̇₁ = γ₁·e·r - σ·(θ₁ - θ₁₀)
 *    θ̇₂ = γ₂·e·y_p - σ·(θ₂ - θ₂₀)
 *  其中θ₁₀, θ₂₀是参数的期望中心值。σ>0时参数会趋向中心值，防止无界漂移。
 */
float MRAC_Calc(mrac_type_def *mrac, float ref, float set, int max)
{
    /* ========== Step 1: 保存输入 ========== */
    mrac->fdb = ref; /* y_p */
    mrac->set = set; /* r */

    /* ========== Step 2: 运行参考模型 ========== */
    /* 连续模型：τ_m·dy_m/dt + y_m = K_m·r
     * 离散化（欧拉法）：y_m(k+1) = y_m(k) + dt·[dy_m/dt]
     *                  dy_m/dt = (K_m·r - y_m) / τ_m
     */
    float ym_dot = (mrac->K_m * set - mrac->ym) / mrac->tau_m;
    mrac->ym += ym_dot * mrac->dt;
    mrac->ym_dot = ym_dot;

    /* ========== Step 3: 计算跟踪误差 ========== */
    /* e = y_m - y_p（参考模型输出 - 实际输出） */
    mrac->error = mrac->ym - ref;

    /* 角度环绕处理（如果需要） */
    if (max != 0)
    {
        if (mrac->error >= (max >> 1))
        {
            mrac->error -= max;
        }
        if (mrac->error < -(max >> 1))
        {
            mrac->error += max;
        }
    }

    /* ========== Step 4: 李雅普诺夫调整律（参数自适应） ========== */
    /* 标准MRAC调整律 */
    float dtheta1 = mrac->gamma1 * mrac->error * set;
    float dtheta2 = mrac->gamma2 * mrac->error * ref;

    /* σ修正（鲁棒性增强） */
    if (mrac->sigma > 0.0f)
    {
        /* 泄漏项：将参数拉向初始值 */
        dtheta1 -= mrac->sigma * (mrac->theta1 - 1.0f);
        dtheta2 -= mrac->sigma * (mrac->theta2 - 0.0f);
    }

    /* 更新参数 */
    mrac->theta1 += dtheta1 * mrac->dt;
    mrac->theta2 += dtheta2 * mrac->dt;

    /* 参数限幅 */
    mrac->theta1 = adaptive_control_clampf(mrac->theta1, mrac->theta1_min, mrac->theta1_max);
    mrac->theta2 = adaptive_control_clampf(mrac->theta2, mrac->theta2_min, mrac->theta2_max);

    /* ========== Step 5: 计算控制量 ========== */
    /* u = θ₁·r - θ₂·y_p */
    mrac->out = mrac->theta1 * set - mrac->theta2 * ref;

    /* 输出限幅 */
    mrac->out = adaptive_control_clampf(mrac->out, -mrac->out_max, mrac->out_max);

    return mrac->out;
}

/**
 * @brief  在线修改参考模型参数
 */
void MRAC_SetReferenceModel(mrac_type_def *mrac, float tau_m, float K_m)
{
    if (tau_m > 0.0f)
    {
        mrac->tau_m = tau_m;
    }
    mrac->K_m = K_m;
}

/**
 * @brief  在线修改自适应增益
 */
void MRAC_SetAdaptiveGain(mrac_type_def *mrac, float gamma1, float gamma2)
{
    mrac->gamma1 = gamma1;
    mrac->gamma2 = gamma2;
}

/**
 * @brief  在线修改σ修正因子
 */
void MRAC_SetSigma(mrac_type_def *mrac, float sigma)
{
    mrac->sigma = sigma;
}

/**
 * @brief  重置MRAC内部状态
 */
void MRAC_Reset(mrac_type_def *mrac)
{
    mrac->ym = 0.0f;
    mrac->ym_dot = 0.0f;
    mrac->error = 0.0f;
    mrac->out = 0.0f;
    mrac->theta1 = 1.0f;
    mrac->theta2 = 0.0f;
}

/* ============================================================================
 *                        增益调度PID控制器实现
 * ============================================================================ */

/**
 * @brief  初始化增益调度PID
 */
void GainSchedulePID_Init(gain_schedule_pid_type_def *gspid, const float Kp_table[GAIN_SCHEDULE_MODES],
                          const float Ki_table[GAIN_SCHEDULE_MODES], const float Kd_table[GAIN_SCHEDULE_MODES],
                          const float thresholds[GAIN_SCHEDULE_MODES - 1], float out_max, float i_max)
{
    uint8_t i;

    /* 复制参数表 */
    for (i = 0; i < GAIN_SCHEDULE_MODES; i++)
    {
        gspid->Kp_table[i] = Kp_table[i];
        gspid->Ki_table[i] = Ki_table[i];
        gspid->Kd_table[i] = Kd_table[i];
    }

    /* 复制调度阈值 */
    for (i = 0; i < GAIN_SCHEDULE_MODES - 1; i++)
    {
        gspid->schedule_thresholds[i] = thresholds[i];
    }

    /* 初始模式 */
    gspid->current_mode = 0;
    gspid->schedule_var = 0.0f;

    /* 平滑过渡参数 */
    gspid->transition_rate = 0.2f; /* 每周期最多变化20% */
    gspid->Kp_smooth = Kp_table[0];
    gspid->Ki_smooth = Ki_table[0];
    gspid->Kd_smooth = Kd_table[0];

    /* PID状态清零 */
    gspid->pout = 0.0f;
    gspid->iout = 0.0f;
    gspid->dout = 0.0f;
    gspid->error = 0.0f;
    gspid->last_error = 0.0f;
    gspid->last_last_error = 0.0f;
    gspid->set = 0.0f;
    gspid->fdb = 0.0f;
    gspid->out = 0.0f;
    gspid->out_max = out_max;
    gspid->i_max = i_max;
}

/**
 * @brief  增益调度PID计算
 *
 *  【调度逻辑】
 *  根据调度变量 schedule_var 的值选择对应的PID参数组：
 *    schedule_var < threshold[0]       → 模式0
 *    threshold[0] ≤ schedule_var < threshold[1]  → 模式1
 *    threshold[1] ≤ schedule_var < threshold[2]  → 模式2
 *    schedule_var ≥ threshold[2]       → 模式3
 *
 *  模式切换时参数平滑过渡，防止输出跳变。
 */
float GainSchedulePID_Calc(gain_schedule_pid_type_def *gspid, float ref, float set, float schedule_var, int max)
{
    uint8_t new_mode;
    float target_Kp, target_Ki, target_Kd;

    /* ========== Step 1: 确定当前模式 ========== */
    gspid->schedule_var = schedule_var;

    if (schedule_var < gspid->schedule_thresholds[0])
    {
        new_mode = 0;
    }
    else if (schedule_var < gspid->schedule_thresholds[1])
    {
        new_mode = 1;
    }
    else if (schedule_var < gspid->schedule_thresholds[2])
    {
        new_mode = 2;
    }
    else
    {
        new_mode = 3;
    }

    gspid->current_mode = new_mode;

    /* ========== Step 2: 参数平滑过渡 ========== */
    /* 获取目标参数 */
    target_Kp = gspid->Kp_table[new_mode];
    target_Ki = gspid->Ki_table[new_mode];
    target_Kd = gspid->Kd_table[new_mode];

    /* 平滑过渡：每周期最多变化 transition_rate 比例 */
    float Kp_diff = target_Kp - gspid->Kp_smooth;
    float Ki_diff = target_Ki - gspid->Ki_smooth;
    float Kd_diff = target_Kd - gspid->Kd_smooth;

    float max_Kp_step = fabsf(gspid->Kp_smooth) * gspid->transition_rate + 0.1f;
    float max_Ki_step = fabsf(gspid->Ki_smooth) * gspid->transition_rate + 0.01f;
    float max_Kd_step = fabsf(gspid->Kd_smooth) * gspid->transition_rate + 0.01f;

    gspid->Kp_smooth += adaptive_control_clampf(Kp_diff, -max_Kp_step, max_Kp_step);
    gspid->Ki_smooth += adaptive_control_clampf(Ki_diff, -max_Ki_step, max_Ki_step);
    gspid->Kd_smooth += adaptive_control_clampf(Kd_diff, -max_Kd_step, max_Kd_step);

    /* ========== Step 3: 标准PID计算 ========== */
    gspid->fdb = ref;
    gspid->set = set;

    gspid->last_last_error = gspid->last_error;
    gspid->last_error = gspid->error;
    gspid->error = set - ref;

    /* 角度环绕处理 */
    if (max != 0)
    {
        if (gspid->error >= (max >> 1))
        {
            gspid->error -= max;
        }
        if (gspid->error < -(max >> 1))
        {
            gspid->error += max;
        }
    }

    /* PID计算（使用平滑后的参数） */
    gspid->pout = gspid->error * gspid->Kp_smooth;
    gspid->iout += gspid->error * gspid->Ki_smooth;
    gspid->dout = (gspid->error - gspid->last_error) * gspid->Kd_smooth;

    /* 积分限幅 */
    gspid->iout = adaptive_control_clampf(gspid->iout, -gspid->i_max, gspid->i_max);

    gspid->out = gspid->pout + gspid->iout + gspid->dout;

    /* 输出限幅 */
    gspid->out = adaptive_control_clampf(gspid->out, -gspid->out_max, gspid->out_max);

    return gspid->out;
}

/**
 * @brief  在线修改某一模式的参数
 */
void GainSchedulePID_SetModeParams(gain_schedule_pid_type_def *gspid, uint8_t mode, float Kp, float Ki, float Kd)
{
    if (mode < GAIN_SCHEDULE_MODES)
    {
        gspid->Kp_table[mode] = Kp;
        gspid->Ki_table[mode] = Ki;
        gspid->Kd_table[mode] = Kd;
    }
}

/**
 * @brief  重置增益调度PID内部状态
 */
void GainSchedulePID_Reset(gain_schedule_pid_type_def *gspid)
{
    gspid->error = 0.0f;
    gspid->last_error = 0.0f;
    gspid->last_last_error = 0.0f;
    gspid->pout = 0.0f;
    gspid->iout = 0.0f;
    gspid->dout = 0.0f;
    gspid->out = 0.0f;
    gspid->Kp_smooth = gspid->Kp_table[gspid->current_mode];
    gspid->Ki_smooth = gspid->Ki_table[gspid->current_mode];
    gspid->Kd_smooth = gspid->Kd_table[gspid->current_mode];
}
