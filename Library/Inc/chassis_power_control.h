#ifndef __CHASSIS_POWER_CONTROL_H__
#define __CHASSIS_POWER_CONTROL_H__

#include "main.h"
#include "motor.h"
#include "PID.h"
#include "referee.h"

/* ==================== M3508电机物理参数 ==================== */

/**
 * 电流转换系数: C620电调原始值 → 实际电流(A)
 * C620量程: -16384~+16384 对应 -20A~+20A
 */
#define M3508_CURRENT_CONVERT (20.0f / 16384.0f)

/**
 * M3508电机内阻(Ω) — 含线阻
 * 参考值: 约0.23Ω
 */
#define M3508_MOTOR_R 0.2304f

/**
 * M3508转矩系数 — 包含 2π/60 转换
 * K_Torque * I(A) * ω(RPM) → 功率(W)
 * 物理值: 转矩常数0.114 N·m/A × 2π/60 ≈ 0.0119
 * 经实车拟合校正: 0.001538
 */
#define M3508_K_TORQUE 0.001538f

/**
 * 单电机静态损耗功率(W)
 * 包括铁损、风摩损、控制器损耗等
 * 四电机合计约12W
 */
#define M3508_POWER_REST 3.0f

/* ==================== 功率控制参数 ==================== */

/* 控制周期(秒) — 对应 Chassis_Task 中的 osDelay(10) */
#define CONTROL_PERIOD_S 0.01f

/* 功率安全系数 — 配合准确功率估算，仅需微弱裕度 */
#define POWER_SAFETY_FACTOR 0.98f

/* 缓冲能量阈值 (裁判系统满缓冲60J) */
#define BUFFER_FULL 60.0f
#define BUFFER_HIGH 40.0f /* 缓冲充足阈值 */
#define BUFFER_LOW 10.0f  /* 缓冲危险阈值 — 开始严格限功率 */
#define BUFFER_MIN 5.0f   /* 缓冲极限阈值 — 最严格限功率 */

/* 电流上升率限制 (防起步功率尖峰) */
#define CURRENT_RISE_LIMIT 1500.0f /* 每拍(10ms)最大电流变化量(C620原始值) */

/* 最大电流上限 (C620安全限制) */
#define MAX_TOTAL_CURRENT 16000.0f /* 单电机最大电流(原始值) */

/* 缓冲能量仿真自校正（替代裁判系统反馈闭环校正） */
#define POWER_CORRECT_KP 0.3f  /* 功率校正比例系数 */
#define POWER_CORRECT_MAX 0.5f /* 最大校正量(不超过50%削减) */

/* 默认功率上限 */
#define CHASSIS_POWER_MAX_DEFAULT (120.0f)

/* 超级电容参数 */
#define CAPACITOR_BOOST_POWER (200.0f)
#define CAPACITOR_MIN_PERCENT (5.0f)

/* S曲线加减速参数 */
#define S_CURVE_ACCEL_RATIO (0.3f)
#define S_CURVE_DECEL_RATIO (0.3f)
#define S_CURVE_MAX_FACTOR (1.5f)

/* ==================== 数据结构 ==================== */

typedef struct
{
    float cap_voltage;   // 电容电压
    float cap_current;   // 电容电流
    float cap_power;     // 电容功率
    uint8_t cap_percent; // 电容电量百分比
} capacitor_measure_t;

/**
 * @brief 底盘功率控制结构体
 */
typedef struct
{
    /* ---- 功率估算(替代裁判系统PowerHeatData) ---- */
    float estimated_power; // 模型估算当前底盘功率(W)
    float sim_buffer;      // 仿真缓冲能量(J)
    float max_power_limit; // 功率上限(W) — 来自GameRobotState + 安全裕度
    float available_power; // 计算后的可用功率(W)

    /* ---- 功率模型预测 ---- */
    float motor_power[4];    // 各电机功率估算(W)
    float total_motor_power; // 总电机功率(W)
    float power_predict;     // 功率预测值(W)

    /* ---- PI线性化参数 ---- */
    float K_PI[4];     // PI线性化斜率: i_out = K_PI * speed_set + M_PI
    float M_PI_val[4]; // PI线性化截距

    /* ---- 衰减系数 ---- */
    float K_Reduction;       // 功率衰减系数 (0~1)
    float K_I_Reduction_Min; // 电流超限衰减系数 (0~1)
    float power_scale;       // 总衰减系数(综合结果)

    /* ---- 电流上升率限制 ---- */
    float last_pid_out[4];       // 上一拍PID输出（上升率限制后）
    float last_final_pid_out[4]; // 上一拍最终PID输出（总衰减后，用于功率估算）

    /* ---- 缓冲能量仿真自校正(替代裁判系统反馈校正) ---- */
    float power_correction; // 累计功率校正量

    /* ---- 超级电容 ---- */
    uint8_t cap_state; // 超级电容状态: 0=正常 1=爆发

    /* ---- 调试 ---- */
    float max_power_record; // 历史最大功率记录
    uint32_t limit_count;   // 功率限制触发次数统计
} chassis_power_control_t;

/* ==================== 函数声明 ==================== */

void chassis_power_control_init(void);

/**
 * @brief 底盘功率控制主函数（四层防护，不依赖PowerHeatData）
 * @param chassis_pid PID数组指针（速度环）
 * @param motor_measure 电机测量数据数组
 * @param speed_set 速度设定值数组
 * @return 是否进行了功率限制
 *
 * 四层防护:
 * 1. 电流上升率限制（防起步冲击）
 * 2. 电流超限约束（防单电机超流）
 * 3. 功率模型预测衰减（防超功率）
 * 4. 缓冲能量仿真自校正（替代裁判系统反馈校正）
 */
uint8_t chassis_power_control_process(pid_type_def chassis_pid[4], motor_measure_DJ_t motor_measure[4],
                                      int16_t speed_set[4]);

float chassis_get_available_power(void);
void chassis_set_cap_state(uint8_t state);
float s_curve_speed_calc(float current_speed, float target_speed, float max_speed);
int16_t speed_ramp_s_curve(int16_t current, int16_t target, int16_t accel_step, int16_t decel_step, int16_t max_speed);

#endif /* __CHASSIS_POWER_CONTROL_H__ */
