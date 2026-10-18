#ifndef __ADRC_H__
#define __ADRC_H__

#include "main.h"
#include <stdint.h>
#include <math.h>

/* ======================== ADRC 参数宏 ======================== */

/* TD（跟踪微分器）默认参数 */
#define ADRC_TD_DEFAULT_R 5000.0f /* 快速跟踪因子，越大跟踪越快但噪声放大 */
#define ADRC_TD_DEFAULT_H0 0.01f  /* TD 滤波因子，h0 >= 控制周期(秒) */

/* LESO（线性扩张状态观测器）默认参数 */
#define ADRC_ESO_DEFAULT_W0 30.0f /* ESO 带宽参数 w0，越大观测越快但噪声敏感 */
                                  /* 三阶ESO极点: -w0, -w0, -w0 */
                                  /* beta1 = 3*w0, beta2 = 3*w0^2, beta3 = w0^3 */

/* LSEF（线性状态误差反馈）默认参数 */
#define ADRC_LSEF_DEFAULT_WC 15.0f /* 控制器带宽 wc */
                                   /* kp = wc^2, kd = 2*wc */

/* 输出限幅默认值 */
#define ADRC_OUT_MAX_DEFAULT 16384.0f

/* b0 最小值保护（防止扰动补偿除零，典型 b0 远大于此值） */
#define ADRC_B0_MIN 1e-4f

/* ======================== ADRC 结构体 ======================== */

/**
 * @brief  ADRC 控制器结构体（二阶系统，三阶 LESO）
 *
 *  适用对象: 速度环（输入=电流，输出=转速）
 *            位置环（输入=速度，输出=位置）
 *
 *  调参顺序: ESO带宽w0 → 控制器带宽wc → TD跟踪因子r
 */
typedef struct
{
    /* ---- TD（跟踪微分器）状态 ---- */
    float td_v1; /* TD 跟踪信号（安排过渡过程后的设定值） */
    float td_v2; /* TD 跟踪信号的微分（设定值的变化率） */
    float td_r;  /* 快速跟踪因子 */
    float td_h;  /* 滤波因子（同时作为 TD 和 ESO 的积分步长，一般等于控制周期） */

    /* ---- LESO（线性扩张状态观测器）状态 ---- */
    float eso_z1;    /* ESO 观测的系统输出 y */
    float eso_z2;    /* ESO 观测的系统输出微分 y' */
    float eso_z3;    /* ESO 观测的总扰动 f */
    float eso_b0;    /* 系统控制增益估计值 b0（>= ADRC_B0_MIN） */
    float eso_beta1; /* ESO 增益1 = 3 * w0 */
    float eso_beta2; /* ESO 增益2 = 3 * w0^2 */
    float eso_beta3; /* ESO 增益3 = w0^3 */
    float eso_w0;    /* ESO 带宽参数 */

    /* ---- LSEF（线性状态误差反馈）参数 ---- */
    float lsef_kp; /* 比例增益 = wc^2 */
    float lsef_kd; /* 微分增益 = 2 * wc */
    float lsef_wc; /* 控制器带宽参数 */

    /* ---- 输入/输出 ---- */
    float set;     /* 目标设定值 */
    float fdb;     /* 反馈实际值 */
    float out;     /* 控制输出 */
    float out_max; /* 输出限幅 */

    /* ---- 诊断信息 ---- */
    float error;       /* 跟踪误差（过圈校正后） */
    float disturbance; /* ESO 观测到的总扰动 z3 */

} adrc_type_def;

/* ======================== 函数声明 ======================== */

/**
 * @brief  初始化 ADRC 控制器
 * @param  adrc     ADRC 结构体指针
 * @param  w0       ESO 带宽参数（典型值 10~100，必须 > 0）
 * @param  wc       控制器带宽参数（典型值 3~50，wc < w0，必须 > 0）
 * @param  b0       系统控制增益估计（>= ADRC_B0_MIN，b0 越大→同样输出产生的响应越大）
 * @param  r        TD 快速跟踪因子（典型值 1000~20000）
 * @param  h        控制周期（秒），如 10ms = 0.01f
 * @param  out_max  输出限幅
 */
void ADRC_Init(adrc_type_def *adrc, float w0, float wc, float b0, float r, float h, float out_max);

/**
 * @brief  ADRC 计算主函数（一个控制周期调用一次）
 * @param  adrc     ADRC 结构体指针
 * @param  ref      反馈实际值（编码器转速/位置）
 * @param  set      目标设定值
 * @param  max      反馈最大值（用于过圈处理，0=不处理）
 * @retval 控制输出值
 */
float ADRC_Calc(adrc_type_def *adrc, float ref, float set, int max);

/**
 * @brief  在线修改 ESO 带宽（w0 必须 > 0）
 */
void ADRC_SetESOBandwidth(adrc_type_def *adrc, float w0);

/**
 * @brief  在线修改控制器带宽（wc 必须 > 0）
 */
void ADRC_SetControllerBandwidth(adrc_type_def *adrc, float wc);

/**
 * @brief  在线修改系统控制增益 b0（自动钳位到 >= ADRC_B0_MIN）
 */
void ADRC_SetB0(adrc_type_def *adrc, float b0);

/**
 * @brief  在线修改输出限幅
 */
void ADRC_SetOutMax(adrc_type_def *adrc, float out_max);

/**
 * @brief  重置 ADRC 内部状态（切换模式时调用）
 */
void ADRC_Reset(adrc_type_def *adrc);

#endif /* __ADRC_H__ */
