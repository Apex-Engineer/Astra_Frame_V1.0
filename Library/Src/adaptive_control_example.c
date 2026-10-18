/**
 * @file    adaptive_control_example.c
 * @brief   自适应控制在当前工程中的集成示例
 * @author  基于 Astra_enginner_V1.0 工程
 *
 * 本文件展示如何将自适应PID、MRAC、增益调度PID集成到当前底盘/机械臂控制中
 * 实际使用时，将对应代码片段复制到 Chassis.c / custom_control.c 中
 */

#include "adaptive_control.h"
#include "PID.h"
#include "Data.h"
#include "motor.h"

/* ============================================================================
 *                        方案一：自适应PID替换底盘速度环
 * ============================================================================ */

/* 定义自适应PID实例（替换原有的 chassis_pid_vel） */
static adaptive_pid_type_def chassis_adaptive_pid_vel[4];

/**
 * @brief  初始化底盘自适应PID（在 Chassis_PID_Init 中调用）
 *
 *  替换原有的 PID_init(&chassis_pid_vel[i], 10.5f, 0.15f, 0.35f, 4000.0f, 16384.0f);
 */
void Chassis_AdaptivePID_Init(void)
{
    for (int i = 0; i < 4; i++)
    {
        /* 初始化参数与原有PID一致 */
        AdaptivePID_Init(&chassis_adaptive_pid_vel[i],
                         10.5f,      /* Kp初始值 */
                         0.15f,      /* Ki初始值 */
                         0.35f,      /* Kd初始值 */
                         4000.0f,    /* 积分限幅 */
                         16384.0f,   /* 输出限幅（M3508最大电流） */
                         0.01f);     /* 控制周期 10ms */

        /* 【可选】自定义自适应参数 */
        /* 学习率：越小越保守，越大收敛越快但可能振荡 */
        AdaptivePID_SetAdaptiveGain(&chassis_adaptive_pid_vel[i],
                                     0.0005f,   /* gamma_p：Kp调整速度 */
                                     0.00005f,  /* gamma_i：Ki调整速度（慢10倍） */
                                     0.00005f); /* gamma_d：Kd调整速度 */

        /* 参数变化范围：Kp可以在 [3.5, 31.5] 之间变化 */
        AdaptivePID_SetParamRange(&chassis_adaptive_pid_vel[i],
                                   3.0f,   /* Kp范围倍数 */
                                   5.0f,   /* Ki范围倍数 */
                                   3.0f);  /* Kd范围倍数 */

        /* 调试阶段先禁用自适应，确认基础PID正常后再启用 */
        /* AdaptivePID_Enable(&chassis_adaptive_pid_vel[i], 1); */
        AdaptivePID_Enable(&chassis_adaptive_pid_vel[i], 0);  /* 默认禁用，调试时手动开启 */
    }
}

/**
 * @brief  底盘速度环控制（在 Chassis_Task 中替换原有PID调用）
 *
 *  替换原有的：
 *      PID_Calc(&chassis_pid_vel[i],
 *               data_motor.Chassis_motor_measure[i].speed_rpm,
 *               chassis_speed_set[i], 0);
 */
void Chassis_AdaptivePID_Control(void)
{
    extern int16_t chassis_speed_set[4];
    extern data_motor_t data_motor;
    extern int16_t chassis_speed[4];

    for (int i = 0; i < 4; i++)
    {
        /* 自适应PID计算（接口与 PID_Calc 完全一致） */
        float current_output = AdaptivePID_Calc(
            &chassis_adaptive_pid_vel[i],
            data_motor.Chassis_motor_measure[i].speed_rpm,  /* 实际转速 */
            chassis_speed_set[i],                            /* 目标转速 */
            0);                                              /* 无角度环绕 */

        /* 发送到电机 */
        chassis_speed[i] = (int16_t)current_output;
    }

    /* CAN发送（与原有逻辑一致） */
    DJ_124_ctrl(chassis_speed[0], chassis_speed[1], chassis_speed[2], chassis_speed[3]);
}

/**
 * @brief  通过遥控器按键切换自适应使能（示例）
 *
 *  例如：长按某个按键开启/关闭自适应
 */
void Chassis_AdaptivePID_Toggle(uint8_t enable)
{
    for (int i = 0; i < 4; i++)
    {
        AdaptivePID_Enable(&chassis_adaptive_pid_vel[i], enable);
    }
}

/**
 * @brief  观察自适应参数变化（调试用，通过串口或UI输出）
 */
void Chassis_AdaptivePID_Debug(void)
{
    /* 将当前Kp/Ki/Kd值发送到UI或串口，观察自适应效果 */
    /* 例如：当机器人抓取重物时，Kp应该自动增大 */
    /*
    for (int i = 0; i < 4; i++)
    {
        printf("Motor%d: Kp=%.3f Ki=%.4f Kd=%.3f\n", i,
               chassis_adaptive_pid_vel[i].Kp,
               chassis_adaptive_pid_vel[i].Ki,
               chassis_adaptive_pid_vel[i].Kd);
    }
    */
}

/* ============================================================================
 *                        方案二：MRAC替换底盘速度环
 * ============================================================================ */

/* 定义MRAC实例 */
static mrac_type_def chassis_mrac_vel[4];

/**
 * @brief  初始化底盘MRAC（替代方案）
 *
 *  MRAC不需要像PID那样精确整定参数，主要调参考模型和自适应增益。
 *  参考模型定义了你期望的系统响应特性。
 */
void Chassis_MRAC_Init(void)
{
    for (int i = 0; i < 4; i++)
    {
        /* 参考模型：tau_m = 0.02s → 期望50ms内达到稳态 */
        /* K_m = 1.0 → 单位增益（输出跟踪输入） */
        MRAC_Init(&chassis_mrac_vel[i],
                  0.02f,      /* tau_m：期望响应时间常数 */
                  1.0f,       /* K_m：稳态增益 */
                  0.001f,     /* gamma1：前馈学习率 */
                  0.001f,     /* gamma2：反馈学习率 */
                  0.01f,      /* dt：10ms控制周期 */
                  16384.0f);  /* out_max：M3508最大电流 */

        /* 【可选】启用σ修正（增强鲁棒性，防止参数漂移） */
        MRAC_SetSigma(&chassis_mrac_vel[i], 0.005f);
    }
}

/**
 * @brief  底盘MRAC速度环控制
 */
void Chassis_MRAC_Control(void)
{
    extern int16_t chassis_speed_set[4];
    extern data_motor_t data_motor;
    extern int16_t chassis_speed[4];

    for (int i = 0; i < 4; i++)
    {
        float current_output = MRAC_Calc(
            &chassis_mrac_vel[i],
            data_motor.Chassis_motor_measure[i].speed_rpm,
            chassis_speed_set[i],
            0);

        chassis_speed[i] = (int16_t)current_output;
    }

    DJ_124_ctrl(chassis_speed[0], chassis_speed[1], chassis_speed[2], chassis_speed[3]);
}

/**
 * @brief  根据当前工况在线调整MRAC参考模型
 *
 *  例如：低速时希望响应平滑（tau_m大），高速时希望响应快速（tau_m小）
 */
void Chassis_MRAC_SetMode(float current_speed)
{
    for (int i = 0; i < 4; i++)
    {
        if (fabsf(current_speed) < 2000.0f)
        {
            /* 低速模式：响应更平滑 */
            MRAC_SetReferenceModel(&chassis_mrac_vel[i], 0.03f, 1.0f);
        }
        else if (fabsf(current_speed) < 8000.0f)
        {
            /* 中速模式 */
            MRAC_SetReferenceModel(&chassis_mrac_vel[i], 0.02f, 1.0f);
        }
        else
        {
            /* 高速模式：响应更快速 */
            MRAC_SetReferenceModel(&chassis_mrac_vel[i], 0.015f, 1.0f);
        }
    }
}

/* ============================================================================
 *                    方案三：增益调度PID用于机械臂
 * ============================================================================ */

/* 定义增益调度PID实例（用于机械臂某关节） */
static gain_schedule_pid_type_def arm_gain_schedule_pid;

/**
 * @brief  机械臂增益调度PID初始化
 *
 *  机械臂不同位姿时转动惯量不同，需要不同的PID参数。
 *  用机械臂伸展程度（如末端距离）作为调度变量。
 */
void Arm_GainSchedulePID_Init(void)
{
    /* 4种模式的PID参数 */
    /* 模式0：完全收缩（转动惯量最小，Kp可以大） */
    /* 模式1：半收缩 */
    /* 模式2：半伸展 */
    /* 模式3：完全伸展（转动惯量最大，Kp要小防止振荡） */
    float Kp_table[4] = {15.0f, 12.0f, 8.0f, 5.0f};
    float Ki_table[4] = {0.2f,  0.15f, 0.1f, 0.05f};
    float Kd_table[4] = {0.5f,  0.6f,  0.8f, 1.0f};

    /* 调度阈值：用机械臂某关节角度（度）作为调度变量 */
    float thresholds[3] = {30.0f, 60.0f, 90.0f};

    GainSchedulePID_Init(&arm_gain_schedule_pid,
                         Kp_table, Ki_table, Kd_table,
                         thresholds,
                         30.0f,    /* 输出限幅（DM电机MIT模式力矩限制） */
                         10.0f);   /* 积分限幅 */
}

/**
 * @brief  机械臂增益调度控制
 * @param  joint_angle  当前关节角度（作为调度变量）
 */
float Arm_GainSchedulePID_Control(float ref, float set, float joint_angle)
{
    return GainSchedulePID_Calc(
        &arm_gain_schedule_pid,
        ref,            /* 反馈值（如编码器位置） */
        set,            /* 目标值 */
        joint_angle,    /* 调度变量（关节角度） */
        0);             /* 无角度环绕 */
}

/* ============================================================================
 *                        方案四：混合策略
 * ============================================================================ */

/**
 * @brief  混合策略：增益调度 + 自适应微调
 *
 *  先用增益调度切换到大致合适的参数范围，
 *  再用自适应PID在该范围内微调。
 *
 *  适用场景：机械臂 —— 大范围的位姿变化用增益调度，
 *                     小范围的负载/摩擦变化用自适应微调
 */
typedef struct
{
    gain_schedule_pid_type_def gspid;   /* 增益调度（大范围粗调） */
    adaptive_pid_type_def apid;          /* 自适应PID（小范围细调） */
    uint8_t strategy;                    /* 0=仅增益调度, 1=混合 */
} hybrid_control_t;

static hybrid_control_t arm_hybrid_ctrl[6];  /* 6关节 */

void Arm_Hybrid_Init(void)
{
    /* 初始化略... */
}

/**
 * @brief  混合控制计算
 *
 *  流程：
 *    1. 增益调度PID计算 → 中间输出 u_gs
 *    2. 用 u_gs 作为自适应PID的"设定值"，进一步微调
 *    3. 最终输出 = 自适应PID的输出
 *
 *  或者更简单的方案：增益调度选择初始Kp/Ki/Kd，自适应PID在此基础上微调
 */
float Arm_Hybrid_Control(int joint_id, float ref, float set, float schedule_var)
{
    /* 增益调度选择基础参数 */
    /* ... */

    /* 自适应PID在基础参数上微调 */
    /* ... */

    return 0.0f;  /* 占位 */
}

/* ============================================================================
 *                        Makefile 修改
 * ============================================================================ */

/**
 * 在 Makefile 中添加新文件编译：
 *
 * C_SOURCES += Library/Src/adaptive_control.c
 *
 * 在 INCLUDE_PATH 中添加头文件路径（如果已有则无需修改）：
 *
 * C_INCLUDES += -ILibrary/Inc
 */

/* ============================================================================
 *                        调参指南
 * ============================================================================ */

/**
 * 【自适应PID调参步骤】
 *
 * Step 1: 确认基础PID工作正常
 *   - 先禁用自适应（AdaptivePID_Enable(..., 0)）
 *   - 用初始Kp/Ki/Kd确认系统能稳定跟踪
 *
 * Step 2: 逐步启用自适应
 *   - 先只调整Kp（gamma_i = gamma_d = 0）
 *   - 观察Kp的变化是否朝着正确的方向
 *   - 例如：抓取重物后响应变慢 → Kp应该自动增大
 *
 * Step 3: 调整学习率
 *   - gamma_p 过大 → Kp快速振荡
 *   - gamma_p 过小 → 参数几乎不变
 *   - 推荐从 0.0001 开始，逐步增大到 0.001
 *
 * Step 4: 调整Ki自适应
 *   - gamma_i 通常比 gamma_p 小 5~10 倍
 *   - 稳态误差大时 Ki 应该增大
 *
 * Step 5: 调整Kd自适应（最谨慎）
 *   - gamma_d 通常最小
 *   - 系统振荡时 Kd 应该增大
 *
 * 【MRAC调参步骤】
 *
 * Step 1: 确定参考模型
 *   - tau_m 定义了期望响应速度
 *   - tau_m 太小 → 期望响应过快，实际跟不上 → 大误差
 *   - tau_m 太大 → 期望响应过慢，性能浪费
 *   - 推荐：tau_m ≈ 实际系统时间常数的 1/2 ~ 1/3
 *   - 电机速度环典型值：tau_m = 0.01~0.03s
 *
 * Step 2: 调整自适应增益
 *   - gamma1/gamma2 过大 → 参数振荡
 *   - gamma1/gamma2 过小 → 收敛太慢
 *   - 推荐从 0.0005 开始
 *
 * Step 3: 启用σ修正（可选）
 *   - 如果参数在无负载时缓慢漂移 → 增大 sigma
 *   - sigma 过大会降低跟踪精度
 *
 * 【增益调度PID调参步骤】
 *
 * Step 1: 确定调度变量
 *   - 底盘：当前速度、功率缓冲百分比
 *   - 机械臂：关节角度（反映转动惯量）
 *   - 发射机构：射速、热量
 *
 * Step 2: 确定模式数量和阈值
 *   - 通常 3~5 个模式足够
 *   - 阈值应覆盖实际工作范围
 *
 * Step 3: 为每个模式单独整定PID
 *   - 可以用原有方法（试凑/ZN法）
 *   - 相邻模式的参数应平滑变化
 *
 * Step 4: 调整过渡速率
 *   - transition_rate 过大 → 模式切换时输出跳变
 *   - transition_rate 过小 → 模式切换响应慢
 *   - 推荐：0.1~0.3（每周期10%~30%变化）
 */
