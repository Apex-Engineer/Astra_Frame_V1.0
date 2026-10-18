#ifndef __HEX_CHASSIS_POWER_CONTROL_H__
#define __HEX_CHASSIS_POWER_CONTROL_H__

#include "main.h"
#include "motor.h"
#include "PID.h"
#include "referee.h"
#include "chassis_power_control.h"

/* ========== 六轮底盘功率控制配置 ========== */

/* 电机数量定义 */
#define HEX_MOTOR_NUM               (6)             // 六轮底盘电机数量

/* 电机功率模型参数 - 基于西交利物浦开源方案 */
/* 这些参数需要通过实际电机测试数据进行拟合调整 */
#define HEX_MOTOR_POWER_MODEL_K1        (1.23e-07f)     // 电流平方项系数 a
#define HEX_MOTOR_POWER_MODEL_K2        (1.453e-07f)    // 转速平方项系数 k2
// #define HEX_MOTOR_TORQUE_COEFFICIENT    (1.99688994e-6f) // 转矩系数 (20/16384)*(0.3)*(187/3591)/9.55???
#define HEX_MOTOR_TORQUE_COEFFICIENT    (2.43316593e-6f) // 转矩系数 (20/16384)*(0.3)*(1/15.76)/9.55???
#define HEX_MOTOR_POWER_CONSTANT        (4.081f)        // 常数项 

#define HEX_2006_MOTOR_POWER_MODEL_K1        (3.69e-7 )     // 2006电机电流平方项系数X  K1_2006 ≈ 3~5 × K1_3508 = 3.69e-7 ~ 6.15e-7
#define HEX_2006_MOTOR_POWER_MODEL_K2        (2.9e-7)    // 2006电机转速平方项系数X  K2_2006 ≈ 2~3 × K2_3508 = 2.9e-7 ~ 4.36e-7
#define HEX_2006_MOTOR_TORQUE_COEFFICIENT    (5.23560209e-7f) // 2006电机转矩系数 (10/10000)*(0.18)*(1/36)/9.55=5.23560209E - 7
#define HEX_2006_MOTOR_POWER_CONSTANT        (2.0f)        // 常数项 X


/* 功率控制参数 */
#define HEX_CHASSIS_POWER_MAX_DEFAULT   (120.0f)        // 默认最大功率
#define HEX_CHASSIS_BUFFER_MAX          (120.0f)        // 缓冲能量最大值
#define HEX_CHASSIS_POWER_SAFETY_MARGIN (5.0f)          // 功率安全余量
#define HEX_CHASSIS_POWER_BUFFER_EXTRA  (30.0f)         // 缓冲满时额外可用功率
#define HEX_CAPACITOR_BOOST_POWER       (200.0f)        // 超级电容爆发功率
#define HEX_CAPACITOR_MIN_PERCENT       (5.0f)          // 电容最小可用百分比

/* S曲线加减速参数 */
#define HEX_S_CURVE_ACCEL_RATIO         (0.3f)          // 加速段比例
#define HEX_S_CURVE_DECEL_RATIO         (0.3f)          // 减速段比例
#define HEX_S_CURVE_MAX_FACTOR          (1.5f)          // S曲线最大加速因子

/* ========== 六轮底盘电机编号定义 ========== */
/* 六轮布局: 左前LF 左中LM 左后LB 右前RF 右中RM 右后RB */
/* 索引: 0     1     2     3     4     5 */
typedef enum
{
    HEX_MOTOR_LF = 0,   // 左前轮 Left Front
    HEX_MOTOR_LM = 1,   // 左中轮 Left Middle
    HEX_MOTOR_LB = 2,   // 左后轮 Left Back
    HEX_MOTOR_RF = 3,   // 右前轮 Right Front
    HEX_MOTOR_RM = 4,   // 右中轮 Right Middle
    HEX_MOTOR_RB = 5,   // 右后轮 Right Back
} hex_motor_index_e;

/**
 * @brief 六轮底盘功率控制结构体
 */
typedef struct
{
    float chassis_power;                // 当前底盘功率 (来自裁判系统)
    float chassis_power_buffer;         // 缓冲能量
    float max_power_limit;              // 最大功率限制
    float available_power;              // 计算后的可用功率

    float motor_power[HEX_MOTOR_NUM];   // 各电机功率估算 [6个电机]
    float total_motor_power;            // 总电机功率

    uint8_t cap_state;                  // 超级电容状态: 0=正常 1=爆发
    float power_scale;                  // 功率缩放系数 (限制时<1)

    uint32_t limit_count;               // 功率限制触发次数统计
    float max_power_record;             // 历史最大功率记录
} hex_chassis_power_control_t;

/**
 * @brief 六轮电机控制数据结构
 */
typedef struct
{
    pid_type_def *pid[HEX_MOTOR_NUM];               // PID控制器指针数组
    motor_measure_DJ_t *motor[HEX_MOTOR_NUM];       // 电机测量数据指针数组
    int16_t speed_set[HEX_MOTOR_NUM];               // 速度设定值数组
    float limited_current[HEX_MOTOR_NUM];           // 限制后的电流值
} hex_motor_control_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化六轮底盘功率控制
 */
void hex_chassis_power_control_init(void);

/**
 * @brief 计算单个电机功率
 * @param speed_rpm 电机转速 (RPM)
 * @param current_pid_output PID输出电流
 * @return 估算的电机功率 (W)
 *
 * 电机功率模型: P = k1*I^2 + k2*w^2 + k3*I*w + constant
 * 其中 k3 为转矩系数
 */
float hex_motor_power_calculate(int16_t speed_rpm, float current_pid_output);

/**
 * @brief 根据目标功率反算允许的最大电流
 * @param speed_rpm 电机当前转速 (RPM)
 * @param target_power 目标功率 (W)
 * @param original_current 原始PID输出电流 (用于判断方向)
 * @return 限制后的电流值
 *
 * 使用一元二次方程求解: a*I^2 + b*I + c = 0
 * 其中: a = k1, b = k3*w, c = k2*w^2 + constant - P_target
 */
float hex_motor_current_limit_by_power(int16_t speed_rpm, float target_power, float original_current);

/**
 * @brief 六轮底盘功率控制主函数
 * @param chassis_pid PID数组指针 (长度为6)
 * @param motor_measure 电机测量数据数组 (长度为6)
 * @param speed_set 速度设定值数组 (长度为6)
 * @return uint8_t 是否进行了功率限制 (0=未限制 1=已限制)
 *
 * 处理流程:
 * 1. 从裁判系统读取功率数据
 * 2. 计算可用功率 (考虑缓冲能量)
 * 3. 计算每个电机的实际功率
 * 4. 判断是否需要限制,需要则反算并应用电流限制
 */
uint8_t hex_chassis_power_control_process(pid_type_def chassis_pid[HEX_MOTOR_NUM],
                                          motor_measure_DJ_t motor_measure[HEX_MOTOR_NUM],
                                          int16_t speed_set[HEX_MOTOR_NUM]);

/**
 * @brief 带指针数组的功率控制处理函数 (更灵活的接口)
 * @param hex_ctrl 六轮电机控制结构体指针
 * @return uint8_t 是否进行了功率限制
 */
uint8_t hex_chassis_power_control_process_ex(hex_motor_control_t *hex_ctrl);

/**
 * @brief 获取当前可用功率
 * @return float 可用功率值 (W)
 */
float hex_chassis_get_available_power(void);

/**
 * @brief 获取当前总电机功率
 * @return float 总功率值 (W)
 */
float hex_chassis_get_total_motor_power(void);

/**
 * @brief 设置超级电容状态
 * @param state 0=正常 1=爆发模式
 */
void hex_chassis_set_cap_state(uint8_t state);

/**
 * @brief 获取功率控制状态
 * @param ctrl 功率控制结构体指针 (用于外部查看状态)
 * @return uint8_t 操作状态 0=成功
 */
uint8_t hex_chassis_get_power_ctrl_state(hex_chassis_power_control_t *ctrl);

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
float hex_s_curve_speed_calc(float current_speed, float target_speed, float max_speed);

/**
 * @brief 带S曲线的速度斜坡控制
 * @param current 当前速度
 * @param target 目标速度
 * @param accel_step 加速度步进
 * @param decel_step 减速度步进
 * @param max_speed 最大速度 (用于S曲线计算)
 * @return int16_t 下一时刻速度
 */
int16_t hex_speed_ramp_s_curve(int16_t current, int16_t target,
                                int16_t accel_step, int16_t decel_step,
                                int16_t max_speed);

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
                                    int16_t output[HEX_MOTOR_NUM]);

/**
 * @brief 获取单个电机的功率占比
 * @param motor_idx 电机索引 (0-5)
 * @return float 功率占比 (0.0~1.0)
 */
float hex_motor_get_power_ratio(uint8_t motor_idx);

/**
 * @brief 六轮底盘功率统计 (用于调试)
 * @param info 信息字符串缓冲区
 * @param len 缓冲区长度
 */
void hex_chassis_power_debug_info(char *info, uint16_t len);

#endif /* __HEX_CHASSIS_POWER_CONTROL_H__ */
