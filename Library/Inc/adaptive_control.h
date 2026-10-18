#ifndef __ADAPTIVE_CONTROL_H__
#define __ADAPTIVE_CONTROL_H__

#include "main.h"
#include <stdint.h>
#include <math.h>

/* ======================== 自适应PID 参数宏 ======================== */

/* 默认自适应增益（学习率）—— 越小收敛越慢但越稳定 */
#define ADAP_PID_DEFAULT_GAMMA_P 0.0005f  /* Kp自适应速率 */
#define ADAP_PID_DEFAULT_GAMMA_I 0.00005f /* Ki自适应速率（比Kp慢10倍，防止积分饱和） */
#define ADAP_PID_DEFAULT_GAMMA_D 0.00005f /* Kd自适应速率（Kd变化要更谨慎） */

/* 参数变化死区 —— 误差小于此值不触发自适应 */
#define ADAP_PID_DEFAULT_DEAD_ZONE 50.0f /* RPM，约3%的最大转速 */

/* 参数变化限幅 —— 每周期最大变化量 */
#define ADAP_PID_DEFAULT_DKP_MAX 0.05f  /* 每周期Kp最大变化 */
#define ADAP_PID_DEFAULT_DKI_MAX 0.005f /* 每周期Ki最大变化 */
#define ADAP_PID_DEFAULT_DKD_MAX 0.005f /* 每周期Kd最大变化 */

/* 参数范围限制（相对于初始值的倍数） */
#define ADAP_PID_DEFAULT_KP_RANGE 3.0f /* Kp在 [init/3, init*3] 之间 */
#define ADAP_PID_DEFAULT_KI_RANGE 5.0f /* Ki在 [init/5, init*5] 之间 */
#define ADAP_PID_DEFAULT_KD_RANGE 3.0f /* Kd在 [init/3, init*3] 之间 */

/* 输出限幅默认值 */
#define ADAP_PID_OUT_MAX_DEFAULT 16384.0f
#define ADAP_PID_I_MAX_DEFAULT 4000.0f

/* 控制周期默认值（秒） */
#define ADAP_PID_DT_DEFAULT 0.01f /* 10ms，对应100Hz */

/* ======================== MRAC 参数宏 ======================== */

/* 参考模型默认值 —— 一阶模型: tau_m * dy/dt + y = K_m * r */
#define MRAC_DEFAULT_TAU_M 0.02f /* 参考模型时间常数（期望响应速度） */
#define MRAC_DEFAULT_K_M 1.0f    /* 参考模型稳态增益（单位增益） */

/* MRAC自适应增益 */
#define MRAC_DEFAULT_GAMMA1 0.001f /* 前馈参数 theta1 学习率 */
#define MRAC_DEFAULT_GAMMA2 0.001f /* 反馈参数 theta2 学习率 */

/* 参数限幅 */
#define MRAC_DEFAULT_THETA1_MAX 5.0f
#define MRAC_DEFAULT_THETA1_MIN 0.01f
#define MRAC_DEFAULT_THETA2_MAX 5.0f
#define MRAC_DEFAULT_THETA2_MIN -2.0f

/* 输出限幅 */
#define MRAC_OUT_MAX_DEFAULT 16384.0f

/* 控制周期 */
#define MRAC_DT_DEFAULT 0.01f

/* ======================== 增益调度PID 参数宏 ======================== */

/* 调度模式数量 */
#define GAIN_SCHEDULE_MODES 4

/* ======================== 结构体定义 ======================== */

/**
 * @brief  自适应PID控制器结构体
 *
 *  核心原理：基于MIT规则（梯度下降）在线调整PID增益
 *
 *  调整律（离散化MIT规则）：
 *    ΔKp = γp · e(k) · e(k-1)        // 误差持续同号→Kp增大，变号→Kp减小
 *    ΔKi = γi · e(k) · ∫e(t)dt      // 稳态误差存在→Ki增大
 *    ΔKd = γd · e(k) · Δe(k-1)      // 振荡时→Kd增大
 *
 *  适用场景：
 *    - 电机负载变化（机器人抓取不同重量物体）
 *    - 摩擦系数变化（温度、磨损）
 *    - 可以直接替换现有PID，接口完全兼容
 */
typedef struct
{
    /* ---- PID基础参数（可变） ---- */
    float Kp; /* 当前比例增益（实时调整） */
    float Ki; /* 当前积分增益（实时调整） */
    float Kd; /* 当前微分增益（实时调整） */

    float Kp_init; /* Kp初始值（自适应中心点） */
    float Ki_init; /* Ki初始值 */
    float Kd_init; /* Kd初始值 */

    /* ---- PID输出分量 ---- */
    float pout; /* 比例输出 */
    float iout; /* 积分输出 */
    float dout; /* 微分输出 */

    /* ---- 误差序列 ---- */
    float error;           /* 当前误差 e(k) */
    float last_error;      /* 上次误差 e(k-1) */
    float last_last_error; /* 上上次误差 e(k-2) */
    float error_integral;  /* 误差积分 ∫e(t)dt */

    /* ---- 输入/输出 ---- */
    float set;     /* 目标设定值 */
    float fdb;     /* 反馈实际值 */
    float out_max; /* 输出限幅 */
    float i_max;   /* 积分限幅 */
    float out;     /* 最终输出 */

    /* ---- 自适应参数 ---- */
    float gamma_p;   /* Kp学习率 */
    float gamma_i;   /* Ki学习率 */
    float gamma_d;   /* Kd学习率 */
    float dead_zone; /* 死区（小误差不调整） */

    /* ---- 参数范围限制 ---- */
    float Kp_max, Kp_min; /* Kp变化范围 */
    float Ki_max, Ki_min; /* Ki变化范围 */
    float Kd_max, Kd_min; /* Kd变化范围 */

    /* ---- 参数变化率限幅 ---- */
    float dKp_max; /* 每周期Kp最大变化量 */
    float dKi_max; /* 每周期Ki最大变化量 */
    float dKd_max; /* 每周期Kd最大变化量 */

    /* ---- 控制周期 ---- */
    float dt; /* 控制周期（秒） */

    /* ---- 使能标志 ---- */
    uint8_t adaptive_enable; /* 0=禁用自适应（固定PID），1=启用自适应 */

} adaptive_pid_type_def;

/**
 * @brief  模型参考自适应控制（MRAC）结构体
 *
 *  核心原理：李雅普诺夫稳定性设计
 *
 *  被控对象假设：τ·dy/dt + y = K·u  （一阶系统）
 *  参考模型：τ_m·dy_m/dt + y_m = K_m·r
 *  控制器：u = θ₁·r - θ₂·y
 *
 *  调整律（保证稳定）：
 *    dθ₁/dt = γ₁ · e · r          // e = y_m - y，跟踪误差
 *    dθ₂/dt = -γ₂ · e · y
 *
 *  稳定性证明：
 *    取李雅普诺夫函数 V = ½e² + (K/2γ₁)(θ₁*-θ₁)² + (K/2γ₂)(θ₂*-θ₂)²
 *    可证 dV/dt = -(1/τ_m)e² ≤ 0，全局渐近稳定
 *
 *  适用场景：
 *    - 需要严格稳定性保证的场合
 *    - 系统参数大范围变化
 *    - 替代PID用于速度环/位置环
 */
typedef struct
{
    /* ---- 参考模型参数 ---- */
    float tau_m; /* 参考模型时间常数 τ_m（越小响应越快） */
    float K_m;   /* 参考模型稳态增益 K_m */

    /* ---- 参考模型状态 ---- */
    float ym;     /* 参考模型输出 y_m */
    float ym_dot; /* 参考模型输出微分（用于二阶参考模型扩展） */

    /* ---- 控制器参数（自适应） ---- */
    float theta1; /* 前馈增益 θ₁ */
    float theta2; /* 反馈增益 θ₂ */

    /* ---- 自适应增益 ---- */
    float gamma1; /* θ₁学习率 γ₁ */
    float gamma2; /* θ₂学习率 γ₂ */

    /* ---- 参数限幅 ---- */
    float theta1_max, theta1_min;
    float theta2_max, theta2_min;

    /* ---- 输入/输出 ---- */
    float set;     /* 设定值 r */
    float fdb;     /* 反馈值 y_p */
    float out;     /* 控制输出 u */
    float out_max; /* 输出限幅 */

    /* ---- 误差 ---- */
    float error; /* 跟踪误差 e = y_m - y_p */

    /* ---- 控制周期 ---- */
    float dt; /* 控制周期（秒） */

    /* ---- 扩展：鲁棒性修改 ---- */
    float sigma; /* σ修正因子（防止参数漂移，典型值0.001~0.01） */

} mrac_type_def;

/**
 * @brief  增益调度PID结构体
 *
 *  核心原理：根据系统工况（如速度、负载）切换不同PID参数组
 *
 *  与自适应PID的区别：
 *    - 自适应PID：参数连续微调，适合慢变系统
 *    - 增益调度：根据已知工况离散切换，适合有明显工作模式的系统
 *
 *  适用场景：
 *    - 底盘低速/高速模式切换
 *    - 机械臂不同位姿（转动惯量不同）
 *    - 发射机构不同射速模式
 */
typedef struct
{
    /* ---- 多组PID参数 ---- */
    float Kp_table[GAIN_SCHEDULE_MODES]; /* 各模式Kp */
    float Ki_table[GAIN_SCHEDULE_MODES]; /* 各模式Ki */
    float Kd_table[GAIN_SCHEDULE_MODES]; /* 各模式Kd */

    /* ---- 调度变量 ---- */
    float schedule_var;                                 /* 当前调度变量值 */
    float schedule_thresholds[GAIN_SCHEDULE_MODES - 1]; /* 调度阈值 */

    /* ---- 当前选用的参数索引 ---- */
    uint8_t current_mode; /* 0 ~ GAIN_SCHEDULE_MODES-1 */

    /* ---- 模式切换平滑（防止参数跳变） ---- */
    float transition_rate;                 /* 参数过渡速率（每周期最大变化比例） */
    float Kp_smooth, Ki_smooth, Kd_smooth; /* 平滑后的当前参数 */

    /* ---- PID状态（同标准PID） ---- */
    float pout, iout, dout;
    float error, last_error, last_last_error;
    float set, fdb;
    float out_max, i_max;
    float out;

} gain_schedule_pid_type_def;

/* ======================== 自适应PID 函数声明 ======================== */

/**
 * @brief  初始化自适应PID控制器
 * @param  apid      自适应PID结构体指针
 * @param  Kp        初始比例增益
 * @param  Ki        初始积分增益
 * @param  Kd        初始微分增益
 * @param  i_max     积分限幅
 * @param  out_max   输出限幅
 * @param  dt        控制周期（秒），如 10ms = 0.01f
 * @note   自适应参数使用默认值，如需自定义请调用 AdaptivePID_SetAdaptiveGain()
 */
void AdaptivePID_Init(adaptive_pid_type_def *apid, float Kp, float Ki, float Kd, float i_max, float out_max, float dt);

/**
 * @brief  自适应PID计算（位置式，接口与PID_Calc完全兼容）
 * @param  apid   自适应PID结构体指针
 * @param  ref    反馈实际值
 * @param  set    目标设定值
 * @param  max    反馈最大值（用于过圈处理，0=不处理）
 * @retval 控制输出值
 * @note   每调用一次，内部会根据MIT规则自动微调Kp/Ki/Kd
 */
float AdaptivePID_Calc(adaptive_pid_type_def *apid, float ref, float set, int max);

/**
 * @brief  自适应PID计算（增量式，接口与PID_Calc_Inc兼容）
 * @param  apid   自适应PID结构体指针
 * @param  ref    反馈实际值
 * @param  set    目标设定值
 * @param  max    反馈最大值（用于过圈处理，0=不处理）
 * @retval 控制输出值
 */
float AdaptivePID_Calc_Inc(adaptive_pid_type_def *apid, float ref, float set, int max);

/**
 * @brief  在线修改自适应学习率
 * @param  apid      自适应PID结构体指针
 * @param  gamma_p   Kp学习率（典型值 0.0001~0.001，越小越保守）
 * @param  gamma_i   Ki学习率（典型值 gamma_p 的 0.1 倍）
 * @param  gamma_d   Kd学习率（典型值 gamma_p 的 0.1 倍）
 * @note   学习率过大可能导致参数振荡甚至发散
 */
void AdaptivePID_SetAdaptiveGain(adaptive_pid_type_def *apid, float gamma_p, float gamma_i, float gamma_d);

/**
 * @brief  在线修改参数变化范围
 * @param  apid       自适应PID结构体指针
 * @param  Kp_range   Kp相对于初始值的变化倍数（如3.0f表示 [init/3, init*3]）
 * @param  Ki_range   Ki相对于初始值的变化倍数
 * @param  Kd_range   Kd相对于初始值的变化倍数
 */
void AdaptivePID_SetParamRange(adaptive_pid_type_def *apid, float Kp_range, float Ki_range, float Kd_range);

/**
 * @brief  使能/禁用自适应
 * @param  apid     自适应PID结构体指针
 * @param  enable   0=禁用（固定为初始PID参数），1=启用自适应
 * @note   调试时建议先禁用自适应，确认基础PID工作正常后再启用
 */
void AdaptivePID_Enable(adaptive_pid_type_def *apid, uint8_t enable);

/**
 * @brief  重置自适应PID内部状态
 * @param  apid   自适应PID结构体指针
 * @note   切换控制模式或重新启动时调用，防止历史状态干扰
 */
void AdaptivePID_Reset(adaptive_pid_type_def *apid);

/**
 * @brief  将参数恢复到初始值
 * @param  apid   自适应PID结构体指针
 */
void AdaptivePID_RestoreDefaults(adaptive_pid_type_def *apid);

/* ======================== MRAC 函数声明 ======================== */

/**
 * @brief  初始化MRAC控制器
 * @param  mrac     MRAC结构体指针
 * @param  tau_m    参考模型时间常数 τ_m（越小响应越快，典型值 0.01~0.05s）
 * @param  K_m      参考模型稳态增益（通常设为1.0f，单位增益）
 * @param  gamma1   前馈参数学习率（典型值 0.0005~0.005）
 * @param  gamma2   反馈参数学习率（典型值 0.0005~0.005）
 * @param  dt       控制周期（秒）
 * @param  out_max  输出限幅
 * @note   调参顺序：先设 tau_m 定义期望响应，再调整 gamma1/gamma2
 */
void MRAC_Init(mrac_type_def *mrac, float tau_m, float K_m, float gamma1, float gamma2, float dt, float out_max);

/**
 * @brief  MRAC计算主函数（一个控制周期调用一次）
 * @param  mrac   MRAC结构体指针
 * @param  ref    反馈实际值 y_p
 * @param  set    目标设定值 r
 * @param  max    反馈最大值（用于过圈处理，0=不处理）
 * @retval 控制输出值 u
 * @note   内部自动运行参考模型和参数自适应
 */
float MRAC_Calc(mrac_type_def *mrac, float ref, float set, int max);

/**
 * @brief  在线修改参考模型参数
 * @param  mrac    MRAC结构体指针
 * @param  tau_m   新的时间常数（越小→响应越快）
 * @param  K_m     新的稳态增益
 */
void MRAC_SetReferenceModel(mrac_type_def *mrac, float tau_m, float K_m);

/**
 * @brief  在线修改自适应增益
 * @param  mrac      MRAC结构体指针
 * @param  gamma1    前馈学习率
 * @param  gamma2    反馈学习率
 */
void MRAC_SetAdaptiveGain(mrac_type_def *mrac, float gamma1, float gamma2);

/**
 * @brief  在线修改 σ修正因子（鲁棒性增强）
 * @param  mrac    MRAC结构体指针
 * @param  sigma   σ值（0=标准MRAC，0.001~0.01=鲁棒MRAC）
 * @note   σ修正可以防止参数在无持续激励时漂移，但会降低收敛精度
 */
void MRAC_SetSigma(mrac_type_def *mrac, float sigma);

/**
 * @brief  重置MRAC内部状态
 * @param  mrac   MRAC结构体指针
 */
void MRAC_Reset(mrac_type_def *mrac);

/* ======================== 增益调度PID 函数声明 ======================== */

/**
 * @brief  初始化增益调度PID
 * @param  gspid      增益调度PID结构体指针
 * @param  Kp_table   Kp参数表（4个模式的Kp值）
 * @param  Ki_table   Ki参数表
 * @param  Kd_table   Kd参数表
 * @param  thresholds 调度阈值（3个阈值，将调度变量分为4个区间）
 * @param  out_max    输出限幅
 * @param  i_max      积分限幅
 */
void GainSchedulePID_Init(gain_schedule_pid_type_def *gspid, const float Kp_table[GAIN_SCHEDULE_MODES],
                          const float Ki_table[GAIN_SCHEDULE_MODES], const float Kd_table[GAIN_SCHEDULE_MODES],
                          const float thresholds[GAIN_SCHEDULE_MODES - 1], float out_max, float i_max);

/**
 * @brief  增益调度PID计算
 * @param  gspid           增益调度PID结构体指针
 * @param  ref             反馈实际值
 * @param  set             目标设定值
 * @param  schedule_var    调度变量（如当前速度、负载估计值等）
 * @param  max             反馈最大值（用于过圈处理，0=不处理）
 * @retval 控制输出值
 */
float GainSchedulePID_Calc(gain_schedule_pid_type_def *gspid, float ref, float set, float schedule_var, int max);

/**
 * @brief  在线修改某一模式的参数
 * @param  gspid   增益调度PID结构体指针
 * @param  mode    模式索引（0~3）
 * @param  Kp      新的Kp值
 * @param  Ki      新的Ki值
 * @param  Kd      新的Kd值
 */
void GainSchedulePID_SetModeParams(gain_schedule_pid_type_def *gspid, uint8_t mode, float Kp, float Ki, float Kd);

/**
 * @brief  重置增益调度PID内部状态
 * @param  gspid   增益调度PID结构体指针
 */
void GainSchedulePID_Reset(gain_schedule_pid_type_def *gspid);

/* ======================== 工具函数 ======================== */

/**
 * @brief  浮点限幅函数
 * @param  x      输入值
 * @param  min    下限
 * @param  max    上限
 * @retval 限幅后的值
 */
static inline float adaptive_control_clampf(float x, float min, float max)
{
    if (x > max)
        return max;
    if (x < min)
        return min;
    return x;
}

/**
 * @brief  一阶低通滤波器
 * @param  input     当前输入
 * @param  last_out  上次输出
 * @param  alpha     滤波系数 (0~1)，越大越跟隨输入
 * @retval 滤波后输出
 */
static inline float adaptive_control_lpf(float input, float last_out, float alpha)
{
    return alpha * input + (1.0f - alpha) * last_out;
}

#endif /* __ADAPTIVE_CONTROL_H__ */
