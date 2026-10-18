#include "adrc.h"

/* ======================== 内部函数声明 ======================== */

static void adrc_td_update(adrc_type_def *adrc, float target);
static void adrc_eso_update(adrc_type_def *adrc, float y, float u);
static float adrc_lsef_calc(adrc_type_def *adrc);
static float adrc_fhan(float v1, float v2, float r, float h);

/* ======================== 初始化 ======================== */

/**
 * @brief  初始化 ADRC 控制器
 *
 *  参数选择指导：
 *  ┌─────────┬──────────────────────────────────────────────────┐
 *  │ 参数    │ 作用与调参建议                                    │
 *  ├─────────┼──────────────────────────────────────────────────┤
 *  │ w0      │ ESO带宽，越大观测越快但越敏感噪声                  │
 *  │         │ 速度环: 30~80，位置环: 10~40                      │
 *  │         │ 规则: w0 ≈ 3~5 × wc                              │
 *  ├─────────┼──────────────────────────────────────────────────┤
 *  │ wc      │ 控制器带宽，越大响应越快但可能震荡                  │
 *  │         │ 速度环: 10~30，位置环: 3~15                       │
 *  ├─────────┼──────────────────────────────────────────────────┤
 *  │ b0      │ 控制增益估计，影响ESO对扰动的估计精度              │
 *  │         │ b0 太大→补偿不足，b0 太小→过度补偿震荡             │
 *  │         │ DJ电机速度环: 500~5000（需实测标定）               │
 *  │         │ DM电机位置环: 1~50                                │
 *  ├─────────┼──────────────────────────────────────────────────┤
 *  │ r       │ TD跟踪因子，越大跟踪越快                          │
 *  │         │ 需要快速响应: 10000~20000                         │
 *  │         │ 需要平滑过渡: 1000~5000                           │
 *  ├─────────┼──────────────────────────────────────────────────┤
 *  │ h       │ 控制周期（秒），必须与实际调用周期一致              │
 *  │         │ 10ms 周期 → h = 0.01                             │
 *  └─────────┴──────────────────────────────────────────────────┘
 */
void ADRC_Init(adrc_type_def *adrc, float w0, float wc, float b0, float r, float h, float out_max)
{
    /* TD 参数 */
    adrc->td_r = r;
    adrc->td_h = h;
    adrc->td_v1 = 0.0f;
    adrc->td_v2 = 0.0f;

    /* ESO 参数（b0 加最小值保护） */
    adrc->eso_w0 = w0;
    adrc->eso_b0 = (fabsf(b0) >= ADRC_B0_MIN) ? b0 : ADRC_B0_MIN;
    adrc->eso_beta1 = 3.0f * w0;
    adrc->eso_beta2 = 3.0f * w0 * w0;
    adrc->eso_beta3 = w0 * w0 * w0;
    adrc->eso_z1 = 0.0f;
    adrc->eso_z2 = 0.0f;
    adrc->eso_z3 = 0.0f;

    /* LSEF 参数 */
    adrc->lsef_wc = wc;
    adrc->lsef_kp = wc * wc;
    adrc->lsef_kd = 2.0f * wc;

    /* 输出限幅 */
    adrc->out_max = out_max;

    /* 状态清零 */
    adrc->set = 0.0f;
    adrc->fdb = 0.0f;
    adrc->out = 0.0f;
    adrc->error = 0.0f;
    adrc->disturbance = 0.0f;
}

/* ======================== TD（跟踪微分器） ======================== */

/**
 * @brief  最速跟踪微分器（韩京清 fhan 函数）
 *
 *  功能：给定目标值 target，安排一个最速过渡过程 td_v1，
 *        同时提取其微分信号 td_v2
 *
 *  优点：设定值突变时不会产生冲击，而是按"最速曲线"平滑过渡
 */
static void adrc_td_update(adrc_type_def *adrc, float target)
{
    float v1 = adrc->td_v1;
    float v2 = adrc->td_v2;
    float r = adrc->td_r;
    float h = adrc->td_h;

    /* 使用 fhan 函数计算最速控制量 */
    float fh = adrc_fhan(v1 - target, v2, r, h);

    /* Euler 积分更新 */
    adrc->td_v1 = v1 + h * v2;
    adrc->td_v2 = v2 + h * fh;
}

/**
 * @brief  韩京清最速控制函数 fhan（标准 sqrt 版）
 * @param  v1  位置误差
 * @param  v2  速度误差
 * @param  r   快速跟踪因子
 * @param  h   步长
 * @retval 最速控制量
 *
 *  这是 ADRC 的核心函数之一，实现了时间最优控制律。
 *  使用 sqrt 计算切换曲线参数，保证相平面轨迹的连续性。
 *  数学推导见韩京清《自抗扰控制技术》
 */
static float adrc_fhan(float v1, float v2, float r, float h)
{
    float d = r * h;
    float d0 = h * d; /* d0 = r * h^2 */
    float y = v1 + h * v2;
    float a, fhan_val;
    float a1;

    /* 标准版：使用 sqrt 计算切换曲线参数 */
    a1 = sqrtf(d * d + 8.0f * r * fabsf(y));

    if (fabsf(y) <= d0)
    {
        a = v2 + y / h;
    }
    else
    {
        if (y > 0.0f)
            a = v2 + (a1 - d) / 2.0f;
        else
            a = v2 - (a1 - d) / 2.0f;
    }

    /* 饱和限幅 */
    if (a > d)
        a = d;
    if (a < -d)
        a = -d;

    /* 输出计算 */
    if (fabsf(a) <= d)
    {
        fhan_val = -r * a / d;
    }
    else
    {
        if (a > 0.0f)
            fhan_val = -r;
        else
            fhan_val = r;
    }

    return fhan_val;
}

/* ======================== LESO（线性扩张状态观测器） ======================== */

/**
 * @brief  三阶线性扩张状态观测器更新
 * @param  adrc  ADRC 结构体指针
 * @param  y     系统输出测量值（反馈值）
 * @param  u     上一拍控制输入
 *
 *  系统模型: y'' = f + b0*u  （f 为总扰动，b0 为控制增益）
 *  扩张状态: x1=y, x2=y', x3=f（总扰动）
 *
 *  LESO 方程:
 *    e  = z1 - y
 *    z1 = z1 + h*(z2 - beta1*e)
 *    z2 = z2 + h*(z3 - beta2*e + b0*u)
 *    z3 = z3 + h*(-beta3*e)
 *
 *  其中极点配置在 -w0 处:
 *    beta1 = 3*w0
 *    beta2 = 3*w0^2
 *    beta3 = w0^3
 *
 *  注意：调用前需确保 z1 已被解包到 y 附近（见 ADRC_Calc 中的过圈处理）
 */
static void adrc_eso_update(adrc_type_def *adrc, float y, float u)
{
    float h = adrc->td_h;
    float b0 = adrc->eso_b0;
    float b1 = adrc->eso_beta1;
    float b2 = adrc->eso_beta2;
    float b3 = adrc->eso_beta3;

    float e = adrc->eso_z1 - y;

    adrc->eso_z1 = adrc->eso_z1 + h * (adrc->eso_z2 - b1 * e);
    adrc->eso_z2 = adrc->eso_z2 + h * (adrc->eso_z3 - b2 * e + b0 * u);
    adrc->eso_z3 = adrc->eso_z3 + h * (-b3 * e);
}

/* ======================== LSEF（线性状态误差反馈） ======================== */

/**
 * @brief  线性状态误差反馈 + 扰动补偿
 * @param  adrc  ADRC 结构体指针
 * @retval 控制量 u
 *
 *  误差定义:
 *    e1 = td_v1 - z1  （位置跟踪误差）
 *    e2 = td_v2 - z2  （速度跟踪误差）
 *
 *  控制律:
 *    u0 = kp*e1 + kd*e2
 *    u  = u0 - z3/b0   （扰动补偿）
 *
 *  其中 kp = wc^2, kd = 2*wc
 */
static float adrc_lsef_calc(adrc_type_def *adrc)
{
    float e1 = adrc->td_v1 - adrc->eso_z1;
    float e2 = adrc->td_v2 - adrc->eso_z2;

    /* 线性状态误差反馈 */
    float u0 = adrc->lsef_kp * e1 + adrc->lsef_kd * e2;

    /* 扰动补偿（b0 零值保护，防止除零导致输出 ±inf） */
    float b0 = adrc->eso_b0;
    if (fabsf(b0) < ADRC_B0_MIN)
        b0 = (b0 >= 0.0f) ? ADRC_B0_MIN : -ADRC_B0_MIN;
    float u = u0 - adrc->eso_z3 / b0;

    return u;
}

/* ======================== ADRC 主计算函数 ======================== */

/**
 * @brief  ADRC 计算主函数
 * @param  adrc  ADRC 结构体指针
 * @param  ref   反馈实际值
 * @param  set   目标设定值
 * @param  max   反馈最大值（用于过圈处理，0=不处理）
 * @retval 控制输出值
 *
 *  调用流程：
 *  1. 保存本次设定值和反馈值
 *  2. 过圈误差计算（与 PID_Calc 兼容）
 *  3. 过圈状态解包：将 ESO 的 z1 和 TD 的 v1 调整到与 ref 同一"圈"
 *  4. TD: 安排过渡过程（使用过圈校正后的目标值）
 *  5. ESO: 观测系统状态和总扰动
 *  6. LSEF: 计算控制量并补偿扰动
 *  7. 输出限幅
 *
 *  过圈处理说明（max != 0 时）：
 *    编码器输出范围 [0, max)，当从 max-1 跳到 0 时会产生"过圈"。
 *    ADRC 的内部状态（TD 的 v1、ESO 的 z1）具有记忆性，不能像 PID
 *    那样只处理误差。本函数采用"状态解包"策略：
 *    - 将 z1 调整到 ref 附近（使观测误差 e = z1 - ref 为最短路径）
 *    - 将 v1 调整到 z1 附近（使 LSEF 的 e1 = v1 - z1 为最短路径）
 *    - 将 set 调整到 z1 附近（使 TD 的跟踪目标连续）
 *    这保证了所有内部状态始终在同一"圈"内工作。
 */
float ADRC_Calc(adrc_type_def *adrc, float ref, float set, int max)
{
    /* 保存输入 */
    adrc->fdb = ref;
    adrc->set = set;

    /* ---- 过圈误差计算（与 PID_Calc 兼容） ---- */
    float error = set - ref;
    if (max != 0)
    {
        if (error >= (float)(max >> 1))
            error -= (float)max;
        if (error < -(float)(max >> 1))
            error += (float)max;
    }
    adrc->error = error;

    /* ---- 过圈状态解包 ---- */
    if (max != 0)
    {
        float half_max = (float)(max >> 1);
        float fmax = (float)max;

        /* 将 ESO 的 z1 解包到 ref 附近，保证观测误差取最短路径 */
        float e_z1 = adrc->eso_z1 - ref;
        if (e_z1 >= half_max)
            adrc->eso_z1 -= fmax;
        else if (e_z1 < -half_max)
            adrc->eso_z1 += fmax;

        /* 将 TD 的 v1 解包到 z1 附近，保证 LSEF 误差取最短路径 */
        float e_v1 = adrc->td_v1 - adrc->eso_z1;
        if (e_v1 >= half_max)
            adrc->td_v1 -= fmax;
        else if (e_v1 < -half_max)
            adrc->td_v1 += fmax;
    }

    /* ---- Step 1: TD 安排过渡过程 ---- */
    /* 将目标设定值解包到 z1 附近，使 TD 的跟踪目标与当前状态在同一"圈" */
    float target = set;
    if (max != 0)
    {
        float half_max = (float)(max >> 1);
        float fmax = (float)max;
        float diff = target - adrc->eso_z1;
        if (diff >= half_max)
            target -= fmax;
        else if (diff < -half_max)
            target += fmax;
    }
    adrc_td_update(adrc, target);

    /* ---- Step 2: ESO 观测系统状态和总扰动 ---- */
    /* 用上一拍的输出 u 作为 ESO 的输入 */
    adrc_eso_update(adrc, ref, adrc->out);

    /* ---- Step 3: LSEF 计算控制量 ---- */
    float u = adrc_lsef_calc(adrc);

    /* ---- Step 4: 输出限幅 ---- */
    if (u > adrc->out_max)
        u = adrc->out_max;
    if (u < -adrc->out_max)
        u = -adrc->out_max;

    adrc->out = u;

    /* 保存诊断信息 */
    adrc->disturbance = adrc->eso_z3;

    return u;
}

/* ======================== 在线参数修改 ======================== */

void ADRC_SetESOBandwidth(adrc_type_def *adrc, float w0)
{
    if (w0 <= 0.0f)
        return; /* w0 必须 > 0，否则 ESO 增益全零，观测器失效 */
    adrc->eso_w0 = w0;
    adrc->eso_beta1 = 3.0f * w0;
    adrc->eso_beta2 = 3.0f * w0 * w0;
    adrc->eso_beta3 = w0 * w0 * w0;
}

void ADRC_SetControllerBandwidth(adrc_type_def *adrc, float wc)
{
    if (wc <= 0.0f)
        return; /* wc 必须 > 0，否则 kp=kd=0，控制器无输出 */
    adrc->lsef_wc = wc;
    adrc->lsef_kp = wc * wc;
    adrc->lsef_kd = 2.0f * wc;
}

void ADRC_SetB0(adrc_type_def *adrc, float b0)
{
    /* b0 零值/极小值保护，防止扰动补偿除零 */
    adrc->eso_b0 = (fabsf(b0) >= ADRC_B0_MIN) ? b0 : ADRC_B0_MIN;
}

void ADRC_SetOutMax(adrc_type_def *adrc, float out_max)
{
    adrc->out_max = out_max;
}

void ADRC_Reset(adrc_type_def *adrc)
{
    adrc->td_v1 = 0.0f;
    adrc->td_v2 = 0.0f;
    adrc->eso_z1 = 0.0f;
    adrc->eso_z2 = 0.0f;
    adrc->eso_z3 = 0.0f;
    adrc->out = 0.0f;
    adrc->error = 0.0f;
    adrc->disturbance = 0.0f;
}
