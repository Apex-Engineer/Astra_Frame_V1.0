#ifndef __CUSTOM_CONTROL_H__
#define __CUSTOM_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== Includes ======================== */
#include "main.h"
#include "PID.h"
#include "motor.h"
#include "bsp_can.h"
#include "referee.h"
#include "Data.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "math.h"
#include "core_cm4.h"
#include <stdio.h>
#include <string.h>

/* ======================== 枚举与常量 ======================== */

/**
 * @brief 机械臂关节数量（不含夹爪）
 *        pos[0~4] → J6/J5/J4/J3/J1,  pos[5] → Pitch1
 */
#define ARM_GRIPPER_IDX     6       /* 夹爪在 arm_motor_measure 中的索引 */

/**
 * @brief 机械臂电机 CAN ID 映射表
 *        索引顺序: [0]=J6, [1]=J5, [2]=J4, [3]=J3, [4]=J1, [5]=Pitch1
 *        与 data_motor.arm_motor_measure[] 数组下标对应
 */
typedef enum {
    ARM_MOTOR_J6      = 0,   /* 0x01 J6         */
    ARM_MOTOR_J5      = 1,   /* 0x03 J5 (与J6并联) */
    ARM_MOTOR_J4      = 2,   /* 0x05 J4/Roll    */
    ARM_MOTOR_J3      = 3,   /* 0x07 J3/Pitch   */
    ARM_MOTOR_J1      = 4,   /* 0x09 J1         */
    ARM_MOTOR_PITCH1  = 5,   /* 0x13 Pitch1     */
    ARM_MOTOR_COUNT   = 6
} ArmMotorIndex_e;

/* 电机 CAN 发送 ID（MIT 模式） */
#define ARM_CAN_ID_J6           0x01
#define ARM_CAN_ID_J5           0x03
#define ARM_CAN_ID_J4           0x05
#define ARM_CAN_ID_J3           0x07
#define ARM_CAN_ID_J1           0x09
#define ARM_CAN_ID_PITCH1       0x13
#define ARM_CAN_ID_GRIPPER      0x11

/* 电机 CAN 接收 ID（零点设置/反馈） */
#define ARM_CAN_RX_ID_J6        0x201
#define ARM_CAN_RX_ID_J5        0x203
#define ARM_CAN_RX_ID_J4        0x205
#define ARM_CAN_RX_ID_J3        0x207
#define ARM_CAN_RX_ID_J1        0x209
#define ARM_CAN_RX_ID_PITCH1    0x213
#define ARM_CAN_RX_ID_GRIPPER   0x211


#define ARM_TX_BUF_COUNT  2

#define MAX_PRESET_STEPS    5  // 所有预设的最大步数（取最大值）


/**
 * @brief 机械臂 CAN ID 查找表
 *        索引 = ArmMotorIndex_e，值为 MIT 控制时的发送 ID
 */
static const uint16_t ARM_MIT_CAN_ID[ARM_MOTOR_COUNT] = {
    ARM_CAN_ID_J6,      /* [0] */
    ARM_CAN_ID_J5,      /* [1] */
    ARM_CAN_ID_J4,      /* [2] */
    ARM_CAN_ID_J3,      /* [3] */
    ARM_CAN_ID_J1,      /* [4] */
    ARM_CAN_ID_PITCH1   /* [5] */
};

/**
 * @brief 机械臂零点设置 CAN ID 查找表（每个电机发两帧：MIT ID + 反馈 ID）
 */
static const uint16_t ARM_ZEROSET_CAN_ID[ARM_MOTOR_COUNT][2] = {
    { ARM_CAN_ID_J6,     ARM_CAN_RX_ID_J6     },
    { ARM_CAN_ID_J5,     ARM_CAN_RX_ID_J5     },
    { ARM_CAN_ID_J4,     ARM_CAN_RX_ID_J4     },
    { ARM_CAN_ID_J3,     ARM_CAN_RX_ID_J3     },
    { ARM_CAN_ID_J1,     ARM_CAN_RX_ID_J1     },
    { ARM_CAN_ID_PITCH1, ARM_CAN_RX_ID_PITCH1 },
};

/**
 * @brief 各关节 MIT 控制参数
 */
typedef struct {
    float kp;       /* 位置刚度 */
    float kd;       /* 速度阻尼 */
    float kp_lock;  /* 锁定时位置刚度 */
    float kd_lock;  /* 锁定时速度阻尼 */
} ArmMITParams_t;

/* ======================== 阻抗控制参数 ======================== */

/**
 * @brief  单关节阻抗控制参数
 *         当 |torque_feedback| > torque_limit 时，目标位置沿力方向让步
 */
typedef struct {
    float torque_limit;        /* 触发让步的力矩阈值 (N·m) */
    float compliance_gain;     /* 让步增益: 每超限1N·m每秒让步多少度 (deg/(N·m·s)) */
    float restore_rate;        /* 无碰撞时偏移恢复速率 (1/s), 越大恢复越快 */
    float max_offset;          /* 最大让步偏移量 (deg), 防止偏移过大 */
} ImpedanceParams_t;

/**
 * @brief  单关节阻抗控制运行状态
 */
typedef struct {
    float compliance_offset;   /* 当前让步偏移量 (deg) */
    uint8_t collision;         /* 当前是否检测到碰撞 */
} ImpedanceState_t;

/* 默认阻抗参数查找表, 索引 = ArmMotorIndex_e */
/*               torque_limit  compliance_gain  restore_rate  max_offset */
static const ImpedanceParams_t ARM_IMPEDANCE_PARAMS[ARM_MOTOR_COUNT] = {
    {  1.0f,         30.0f,           2.0f,         30.0f  },  /* [0] J6       - 小臂, 力矩小 */
    {  1.0f,         30.0f,           2.0f,         30.0f  },  /* [1] J5       - 小臂, 力矩小 */
    {  2.0f,         20.0f,           2.0f,         20.0f  },  /* [2] J4/Roll  - 中等力矩 */
    {  2.0f,         20.0f,           2.0f,         20.0f  },  /* [3] J3/Pitch - 中等力矩 */
    {  1.5f,         20.0f,           2.0f,         30.0f  },  /* [4] J1       - 中等力矩 */
    {  2.0f,         20.0f,           2.0f,         20.0f  },  /* [5] Pitch1   - 大臂, 力矩大 */
};


/* 默认 MIT 参数查找表，索引 = ArmMotorIndex_e */
static const ArmMITParams_t ARM_MIT_PARAMS[ARM_MOTOR_COUNT] = {
    /*  kp     kd    kp_lock  kd_lock  */
    {  2.0f,  0.2f,  2.0f,    0.2f   },  /* [0] J6       */
    {  2.0f,  0.2f,  2.0f,    0.2f   },  /* [1] J5       */
    { 50.0f,  5.0f, 50.0f,    5.0f   },  /* [2] J4/Roll  */
    { 70.0f,  10.0f, 50.0f,    5.0f   },  /* [3] J3/Pitch */
    { 50.0f,  5.0f, 50.0f,    5.0f   },  /* [4] J1       */
    {110.0f, 10.0f, 100.0f,  10.0f   },  /* [5] Pitch1   */
};

/* 夹爪 MIT 参数 */
#define GRIPPER_KP          2.0f
#define GRIPPER_KD          0.1f
#define GRIPPER_KP_HOLD     0.0f
#define GRIPPER_KD_HOLD     0.0f

/**
 * @brief 机械臂角度限位（度）
 */
#define ARM_POS_LIMIT       700.0f
#define PITCH_MAX_LIMIT     180.0f//360.0f
#define ROLL_MAX_LIMIT      360.0f//720.0f

/* 夹爪限位 */
#define GRIPPER_POS_LIMIT   140000.0f
#define GRIPPER_STEP        5.0f

/* 梯形减速参数 */
#define DECEL_ZONE          30.0f    /* 减速区间（度） */
#define CRUISE_VEL          0.1f     /* 巡航速度 */
#define POS_ARRIVE_THRESH   5.0f     /* 到位判断阈值（度） */
#define PRESET_STEP_SIZE    2.0f     //0.8

/* Pitch1 / J3 手动步进量（预设模式下 Z/X 和 C/V 微调用） */
#define PITCH1_STEP_VT13    1.05f
#define J3_STEP_VT13        1.05f
#define J4_STEP_VT13        1.05f
#define J1_STEP_VT13        1.05f
#define J6J5_PITCH_STEP_VT13 1.05f
#define J6J5_YAW_STEP_VT13 1.05f

/* 重力补偿系数 */
#define GRAVITY_COMP_COEFF  1.65f
#define GRAVITY_COMP_ARM    0.365f
#define GRAVITY_COMP_ANGLE  30.0f

/* Pitch1-J1 联动保护限位（角度后续微调） */
#define PITCH1_J1_PROTECT_THRESHOLD  -50.0f   /* Pitch1 触发保护的角度阈值（度） */
#define J1_PROTECT_LIMIT             10.0f 
#define J1_PROTECT_CANCEL_ANGLE      80.0f    /* J1 超过此角度时取消联动保护（度） */
/* 初始化延时 */
#define ARM_INIT_DELAY_MS   10

// /**
//  * @brief Ctrl 组合键每个有4个预设步骤，按第5次退出
//  */
// #define MAX_PRESET_STEPS    5//3

/**
 * @brief 机械臂工作模式
 */
typedef enum {
    ARM_MODE_NORMAL = 0,    /* 正常遥控模式 */
    ARM_MODE_PRESET,        /* 预设位姿模式 */
    ARM_MODE_HOMING,        /* 回零模式 */
    ARM_MODE_LOCKED,        /* B键锁定模式 */
    ARM_MODE_COUNT
} ArmMode_e;

/**
 * @brief 预设位姿编号
 *        Ctrl+Z/X/C:
 *        Shift+Z/X/C: 
 *        按不同组合键 → 切换到新预设
 */
typedef enum {
    PRESET_NONE = 0,        /* 正常模式 */
    PRESET_CTRL_Z,          /* Ctrl+Z:  取左上 */
    PRESET_CTRL_X,          /* Ctrl+X:  存车左 */
    PRESET_CTRL_C,          /* Ctrl+C:  取车左 */
    PRESET_SHIFT_Z,         /* Shift+Z: 取右上 */
    PRESET_SHIFT_X,         /* Shift+X: 存车右 */
    PRESET_SHIFT_C,         /* Shift+C: 取车右 */
    PRESET_COUNT            /* = 7 */
} PresetId_e;


/**
 * @brief 机械臂运行状态上下文
 */
typedef struct {
    /* 模式状态 */
    ArmMode_e   mode;               /* 当前工作模式 */
    uint8_t     initialized;        /* 初始化完成标志 */
    uint8_t     system_ready;       /* 系统就绪（裁判系统供电） */

    /* 锁定模式 */
    uint8_t     arm_locked;         /* 机械臂是否锁定 */
    uint8_t     last_b_key;         /* B键上一次状态（边沿检测） */
    uint8_t     last_f_key;         /* 【新增】F键上一次状态（边沿检测） */
    float       locked_pos[ARM_MOTOR_COUNT]; /* 锁定时各关节角度 */
    float       homing_target[ARM_MOTOR_COUNT]; /* 【新增】回零目标角度 */
    uint8_t     homing_phase;                   /* 【新增】回零阶段：0=J1先回零, 1=全部回零 */

    /* 预设模式 */
    PresetId_e  preset_id;          /* 当前预设编号 */
    uint8_t     preset_step;        /* 当前预设步骤 (0-based)
                                     *   Ctrl组合键: 0/1/2 对应3个预设位置，按第4次退出
                                     *   Shift组合键: 0 对应1个预设位置，按第2次退出
                                     */

    /* 组合键边沿检测（每个预设组合键独立检测） */
    uint8_t     last_ctrl_z_key;    /* Ctrl+Z  */
    uint8_t     last_ctrl_x_key;    /* Ctrl+X  */
    uint8_t     last_ctrl_c_key;    /* Ctrl+C  */
    uint8_t     last_shift_z_key;   /* Shift+Z */
    uint8_t     last_shift_x_key;   /* Shift+X */
    uint8_t     last_shift_c_key;   /* Shift+C */

    float       preset_target[ARM_MOTOR_COUNT]; /* 预设目标角度 */
    float       pos_error[ARM_MOTOR_COUNT];     /* 位置误差 */
    float       vel_cmd[ARM_MOTOR_COUNT];       /* 速度指令 */
    uint8_t     preset_arrived;     /* 预设位姿是否到达 */

    float       interp_pos[ARM_MOTOR_COUNT]; /* 【新增】插值位置，每拍向目标靠近一步 */

    /* 预设模式下手动微调偏移量 */
    float       j1_manual_offset;   /* J1 手动偏移（Shift+Z/X控制，进入预设时归零） */
    float       j3_manual_offset;   /* J3 手动偏移（C/V控制，进入预设时归零） */
    float       j4_manual_offset;   /* J4 手动偏移（B/G控制，进入预设时归零） */
    float       j6j5_pitch_manual_offset; // pitch 微调偏移
    float       j6j5_yaw_manual_offset; //yaw 微调偏移
    float       pitch1_manual_offset; /* Pitch1 手动偏移（Z/X控制，进入预设时归零） */

    /* 夹爪 */
    float       gripper_pos;        /* 夹爪当前角度 */

    /* Pitch1
     *   正常模式: 由外部 pos[5] 输入驱动
     *   预设模式: 由预设目标值驱动，Z/X 可微调
     */
    float       pitch1_pos;         /* Pitch1 当前角度（正常模式跟踪 pos[5]） */

    /* 重力补偿 */
    float       gravity_torque;     /* 计算出的重力补偿力矩 */
        /* 阻抗控制 */
    uint8_t         impedance_enabled;                    /* 阻抗控制总开关 */
    ImpedanceState_t impedance_state[ARM_MOTOR_COUNT];   /* 各关节阻抗状态 */

} ArmContext_t;


/**
 * @brief  机械臂电机扭矩上报数据段（CmdID 0x0309，固定30字节）
 *         [torque×6] 占 24 字节，[reserved×6] 填 0
 */
typedef struct __attribute__((packed)) {
    float torque[ARM_MOTOR_COUNT];   /* 24字节: [J6, J5, J4, J3, J1, Pitch1] 扭矩(N·m) */
    uint8_t reserved[6];             /* 6字节: 填0，对齐到30字节 */
} ArmTorqueReport_t;


// custom_control.c — 新增每个预设的实际步数表
static const uint8_t PRESET_MAX_STEPS[PRESET_COUNT] = {
    0,  // PRESET_NONE
    4,  // PRESET_CTRL_Z:  取左上, step 0~3, 按第5次退出
    4,  // PRESET_CTRL_X:  存车左, step 0~3, 按第5次退出
    5,  // PRESET_CTRL_C:  取车左, step 0~4, 按第6次退出
    4,  // PRESET_SHIFT_Z: 取右上, step 0~3, 按第5次退出
    4,  // PRESET_SHIFT_X: 存车右, step 0~3, 按第5次退出
    5,  // PRESET_SHIFT_C: 取车右, step 0~4, 按第6次退出
};

/* ======================== 函数声明 ======================== */

/**
 * @brief  机械臂控制主任务（供 FreeRTOS 调用）
 */
void arm_control_task(void);

/**
 * @brief  机械臂电机初始化（使能 + 零点设置）
 * @retval 0: 成功, -1: 失败
 */
int8_t arm_init(void);

/**
 * @brief  机械臂电机重新初始化（检测掉线后调用）
 */
void arm_reinit(void);

/* ---- 模式处理函数 ---- */
void arm_mode_normal_handler(ArmContext_t *ctx);
void arm_mode_preset_handler(ArmContext_t *ctx);
void arm_mode_locked_handler(ArmContext_t *ctx);

/* ---- 辅助函数 ---- */
float   clampf(float val, float min_val, float max_val);
void    arm_send_mit(ArmMotorIndex_e idx, float pos, float vel, float tau);
void    arm_custom_send_mit(ArmMotorIndex_e idx, float pos, float vel, float tau);
float   arm_j1_protection_clamp(float j1_target);

void    arm_send_mit_locked(ArmMotorIndex_e idx, float pos, float tau);
void ZeroCrossing(ArmMotorIndex_e idx , float pos , float p_max ,float p_min);
void    gripper_control(ArmContext_t *ctx);
void    j3_manual_control(ArmContext_t *ctx);
void    j4_manual_control(ArmContext_t *ctx);
void    j1_manual_control(ArmContext_t *ctx);
void    j6j5_manual_control(ArmContext_t *ctx);
void    arm_check_motor_online(ArmContext_t *ctx);
void    arm_preset_update_target(ArmContext_t *ctx);
uint8_t arm_preset_check_arrived(ArmContext_t *ctx);
void    arm_update_preset_keys(ArmContext_t *ctx);
float   arm_impedance_filter(ArmMotorIndex_e idx, float target_pos);
void    arm_impedance_reset(ArmContext_t *ctx);


#ifdef __cplusplus
}
#endif

#endif /* __CUSTOM_CONTROL_H__ */
