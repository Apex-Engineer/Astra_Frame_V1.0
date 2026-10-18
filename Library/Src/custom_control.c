#include "custom_control.h"

/* ======================== 外部变量 ======================== */
extern data_motor_t data_motor;
extern data_referee_t data_referee;
extern VT13_RC_ctrl_t VT13_rc_ctrl;

/*
 * pos[0~4]: J6/J5/J4/J3/J1 的外部目标位置（由串口解析填入）
 * pos[5]:   Pitch1 的外部目标位置（由串口解析填入）
 *
 * 正常模式下 Pitch1 直接由 pos[5] 驱动（像其他关节一样）
 * 预设模式下 Pitch1 由预设目标值驱动，Z/X 可微调
 */
extern float pos[6];
extern float vel[6];
extern float tor[6];

/* ======================== 模块内部变量 ======================== */
ArmContext_t arm_ctx = {0}; /* 定义在 custom_control.c 中，供其他模块访问 */
static uint8_t arm_tx_buf_pool[ARM_TX_BUF_COUNT][40];
static uint8_t arm_tx_buf_index = 0;
/* ======================== 工具函数 ======================== */

/**
 * @brief  通过 UART6 上报 6 个机械臂电机扭矩（CmdID 0x0309）
 *         帧格式与 CustomController_StructSend 一致，仅 CmdID 不同
 *         总长度 = 5(帧头) + 2(CmdID) + 30(数据) + 2(CRC16) = 39 字节
 */
void ArmTorque_ReportSend(ArmTorqueReport_t *data)
{
    uint8_t current_buf = arm_tx_buf_index;
    uint8_t *tx_buf = arm_tx_buf_pool[current_buf];
    uint16_t idx = 0;

    /* 步骤1：帧头 */
    tx_buf[idx++] = 0xA5;
    tx_buf[idx++] = 30 & 0xFF;        // 0x1E
    tx_buf[idx++] = (30 >> 8) & 0xFF; // 0x00
    tx_buf[idx++] = 0x00;             // seq

    /* 步骤2：CRC8 */
    tx_buf[idx] = Get_CRC8_Check_Sum(tx_buf, 4, 0xff);
    idx++;

    /* 步骤3：CmdID = 0x0309 小端序 */
    tx_buf[idx++] = 0x09;
    tx_buf[idx++] = 0x03;

    /* 步骤4：30字节数据段 */
    memcpy(&tx_buf[idx], data, sizeof(ArmTorqueReport_t));
    idx += 30;

    /* 步骤5：CRC16 */
    uint16_t crc16 = Get_CRC16_Check_Sum(tx_buf, 37, 0xffff);
    tx_buf[idx++] = crc16 & 0xFF;
    tx_buf[idx++] = (crc16 >> 8) & 0xFF;

    /* 步骤6：DMA 发送，总长 39 字节 */
    HAL_UART_Transmit_DMA(&huart6, tx_buf, 39);

    arm_tx_buf_index = (arm_tx_buf_index + 1) % ARM_TX_BUF_COUNT;
}

/**
 * @brief  浮点数限幅
 */
float clampf(float val, float min_val, float max_val)
{
    if (val > max_val)
        return max_val;
    if (val < min_val)
        return min_val;
    return val;
}

/* ======================== 阻抗控制 ======================== */

/**
 * @brief  阻抗滤波器 — 根据扭矩反馈柔顺化目标位置
 *
 *   原理:
 *     1) 读取当前关节扭矩反馈 tau
 *     2) 如果 |tau| > torque_limit → 碰撞! 目标位置沿力方向让步
 *        compliance_offset += sign(tau) * gain * (|tau| - limit) * dt
 *     3) 如果 |tau| <= torque_limit → 无碰撞, 让步偏移逐渐恢复
 *        compliance_offset *= (1 - restore_rate * dt)
 *     4) 输出 = 原始目标 - 让步偏移
 *
 *   效果:
 *     碰到障碍 → 扭矩超限 → 目标"顺从"偏移 → 电机不再硬推
 *     障碍移除 → 扭矩回落 → 偏移慢慢恢复 → 臂回到原始目标
 *
 * @param  idx         关节索引
 * @param  target_pos  原始目标位置 (deg)
 * @retval             柔顺化后的目标位置 (deg)
 */
float arm_impedance_filter(ArmMotorIndex_e idx, float target_pos)
{
    if (idx >= ARM_MOTOR_COUNT || !arm_ctx.impedance_enabled)
        return target_pos;

    const ImpedanceParams_t *params = &ARM_IMPEDANCE_PARAMS[idx];
    ImpedanceState_t *state = &arm_ctx.impedance_state[idx];

    /* 读取当前扭矩反馈 */
    float tau = data_motor.arm_motor_measure[idx].tor;
    float dt = 0.01f; /* 控制周期 10ms */

    if (fabsf(tau) > params->torque_limit)
    {
        /* ---- 碰撞: 让步 ---- */
        state->collision = 1;
        float excess = fabsf(tau) - params->torque_limit;
        float direction = (tau > 0.0f) ? 1.0f : -1.0f;
        /* 让步方向与力矩同向: 力往正方向推 → 目标往正方向让 */
        state->compliance_offset += direction * params->compliance_gain * excess * dt;
        state->compliance_offset = clampf(state->compliance_offset, -params->max_offset, params->max_offset);
    }
    else
    {
        /* ---- 无碰撞: 逐渐恢复 ---- */
        state->collision = 0;
        state->compliance_offset *= (1.0f - params->restore_rate * dt);
        /* 死区: 防止浮点漂移 */
        if (fabsf(state->compliance_offset) < 0.1f)
            state->compliance_offset = 0.0f;
    }

    /* 输出 = 原始目标 - 让步偏移
     * (让步偏移与力同向, 减去后目标朝力方向移动 → 电机不再硬推) */
    return target_pos - state->compliance_offset;
}

/**
 * @brief  复位所有关节的阻抗状态
 */
void arm_impedance_reset(ArmContext_t *ctx)
{
    for (int i = 0; i < ARM_MOTOR_COUNT; i++)
    {
        ctx->impedance_state[i].compliance_offset = 0.0f;
        ctx->impedance_state[i].collision = 0;
    }
}

/**
 * @brief  通过查表发送 MIT 控制指令到指定关节
 * @param  idx  关节索引 (ArmMotorIndex_e)
 * @param  pos  目标位置
 * @param  vel  目标速度
 * @param  tau  前馈力矩
 */
void arm_send_mit(ArmMotorIndex_e idx, float pos, float vel, float tau)
{
    if (idx >= ARM_MOTOR_COUNT)
        return;
    if (idx == ARM_MOTOR_J6 || idx == ARM_MOTOR_J5 || idx == ARM_MOTOR_J4)
    {
        DM_custom_motor_MIT_ctrl(&hcan2, ARM_MIT_CAN_ID[idx], pos, 0.0f, ARM_MIT_PARAMS[idx].kp, ARM_MIT_PARAMS[idx].kd,
                                 tau);
    }
    else
    {
        DM_motor_MIT_ctrl(&hcan2, ARM_MIT_CAN_ID[idx], pos, 0.0f, ARM_MIT_PARAMS[idx].kp, ARM_MIT_PARAMS[idx].kd, tau);
    }
}

/**
 * @brief  通过查表发送 p_max为1250的MIT 控制指令到指定关节
 * @param  idx  关节索引 (ArmMotorIndex_e)
 * @param  pos  目标位置
 * @param  vel  目标速度
 * @param  tau  前馈力矩
 */
void arm_custom_send_mit(ArmMotorIndex_e idx, float pos, float vel, float tau)
{
    if (idx >= ARM_MOTOR_COUNT)
        return;
    DM_custom_motor_MIT_ctrl(&hcan2, ARM_MIT_CAN_ID[idx], pos, vel, ARM_MIT_PARAMS[idx].kp, ARM_MIT_PARAMS[idx].kd,
                             tau);
}

/**
 * @brief  锁定模式下的 MIT 控制（使用锁定参数，支持前馈力矩）
 * @param  idx  关节索引
 * @param  pos  锁定目标位置
 * @param  tau  前馈力矩（用于重力补偿）
 */
void arm_send_mit_locked(ArmMotorIndex_e idx, float pos, float tau)
{
    if (idx >= ARM_MOTOR_COUNT)
        return;
    if (idx == ARM_MOTOR_J6 || idx == ARM_MOTOR_J5 || idx == ARM_MOTOR_J4)
    {
        DM_custom_motor_MIT_ctrl(&hcan2, ARM_MIT_CAN_ID[idx], pos, 0.0f, ARM_MIT_PARAMS[idx].kp_lock,
                                 ARM_MIT_PARAMS[idx].kd_lock, tau);
    }
    else
    {
        DM_motor_MIT_ctrl(&hcan2, ARM_MIT_CAN_ID[idx], pos, 0.0f, ARM_MIT_PARAMS[idx].kp_lock,
                          ARM_MIT_PARAMS[idx].kd_lock, tau);
    }
}

void ZeroCrossing(ArmMotorIndex_e idx, float pos, float p_max, float p_min)
{
    if (pos > p_max)
    {
        DM_motor_zeroset(&hcan2, ARM_ZEROSET_CAN_ID[idx][0]);
        osDelay(1);
    }
    else if (pos < p_min)
    {
        DM_motor_zeroset(&hcan2, ARM_ZEROSET_CAN_ID[idx][0]);
        osDelay(1);
    }
}
/* ======================== 初始化 ======================== */

/**
 * @brief  机械臂电机使能
 */
static void arm_enable_motors(void)
{
    for (int i = 0; i < ARM_MOTOR_COUNT; i++)
    {
        DM_motor_enable(&hcan2, ARM_MIT_CAN_ID[i]);
        osDelay(ARM_INIT_DELAY_MS);
    }
    /* 夹爪单独使能 */
    DM_motor_enable(&hcan2, ARM_CAN_ID_GRIPPER);
    osDelay(ARM_INIT_DELAY_MS);
}

/**
 * @brief  机械臂电机零点设置（每个电机发两帧）
 */
static void arm_set_zeros(void)
{
    for (int i = 0; i < ARM_MOTOR_COUNT; i++)
    {
        DM_motor_zeroset(&hcan2, ARM_ZEROSET_CAN_ID[i][0]);
        osDelay(ARM_INIT_DELAY_MS);
        DM_motor_zeroset(&hcan2, ARM_ZEROSET_CAN_ID[i][1]);
        osDelay(ARM_INIT_DELAY_MS);
    }
    /* 夹爪 */
    DM_motor_zeroset(&hcan2, ARM_CAN_ID_GRIPPER);
    osDelay(ARM_INIT_DELAY_MS);
    DM_motor_zeroset(&hcan2, ARM_CAN_RX_ID_GRIPPER);
    osDelay(ARM_INIT_DELAY_MS);
}

/**
 * @brief  机械臂完整初始化
 */
int8_t arm_init(void)
{
    if (arm_ctx.initialized)
    {
        return 0;
    }

    arm_enable_motors();
    arm_set_zeros();

    /* 初始化上下文 */
    arm_ctx.mode = ARM_MODE_NORMAL;
    arm_ctx.arm_locked = 0;
    arm_ctx.preset_id = PRESET_NONE;
    arm_ctx.preset_step = 0;
    arm_ctx.gripper_pos = data_motor.arm_motor_measure[ARM_GRIPPER_IDX].pos;
    arm_ctx.pitch1_pos = data_motor.arm_motor_measure[ARM_MOTOR_PITCH1].pos;
    arm_ctx.j3_manual_offset = 0.0f;
    arm_ctx.pitch1_manual_offset = 0.0f;
    arm_ctx.j4_manual_offset = 0.0f;
    arm_ctx.j1_manual_offset = 0.0f;
    arm_ctx.j6j5_pitch_manual_offset = 0.0f;
    arm_ctx.j6j5_yaw_manual_offset = 0.0f;

    arm_ctx.initialized = 1;
    arm_ctx.impedance_enabled = 0; /* 默认开启阻抗控制 */

    return 0;
}

/**
 * @brief  重新初始化（电机掉线后调用）
 */
void arm_reinit(void)
{
    arm_ctx.initialized = 0;
    arm_init();
}

/* ======================== 电机在线检测 ======================== */

/**
 * @brief  检测所有机械臂电机是否在线
 * @retval 1: 全部在线, 0: 有电机掉线
 */
void arm_check_motor_online(ArmContext_t *ctx)
{
    for (int i = 0; i < ARM_MOTOR_COUNT; i++)
    {
        if (data_motor.arm_motor_measure[i].id == 0)
        {
            arm_reinit();
            ctx->system_ready = 0;
            return;
        }
    }
}

/* ======================== Pitch1-J1 联动保护 ======================== */

/**
 * @brief  J1-Pitch1 联动保护限幅（改进版）
 *
 *  保护逻辑：
 *    当 Pitch1 抬起（> PITCH1_J1_PROTECT_THRESHOLD）时，
 *    J1 旋转可能与 Pitch1 臂产生机械干涉。
 *
 *    但干涉只在 J1 处于"中间区域"时发生。
 *    当 J1 已经转到"大角度区域"（超过 J1_PROTECT_CANCEL_ANGLE），
 *    机械上不再干涉，此时应取消保护让 J1 自由运动。
 *
 *  判断规则：
 *    ┌──────────────────────┬──────────────────────────┐
 *    │ Pitch1 状态           │ J1 限幅                  │
 *    ├──────────────────────┼──────────────────────────┤
 *    │ Pitch1 > -25° (抬起) │                          │
 *    │   且 |J1| < 30°      │  J1 钳位到 ±10°          │
 *    │   且 |J1| ≥ 30°      │  不限幅（J1已在大角度）   │
 *    ├──────────────────────┼──────────────────────────┤
 *    │ Pitch1 ≤ -25° (放下) │  不限幅                  │
 *    └──────────────────────┴──────────────────────────┘
 *
 *  注意：使用 J1 的**实际反馈角度**判断，而非目标角度，
 *        避免目标角度突变导致保护状态闪烁。
 */
float arm_j1_protection_clamp(float j1_target)
{
    float pitch1_actual = data_motor.arm_motor_measure[ARM_MOTOR_PITCH1].pos;
    float j1_actual = data_motor.arm_motor_measure[ARM_MOTOR_J1].pos;
    if (pitch1_actual > PITCH1_J1_PROTECT_THRESHOLD)
    {
        /* Pitch1 高于阈值 → J1 限幅到 ±J1_PROTECT_LIMIT */
        if (fabsf(j1_actual) < J1_PROTECT_CANCEL_ANGLE)
        {
            return clampf(j1_target, -J1_PROTECT_LIMIT, J1_PROTECT_LIMIT);
        }
    }

    /* Pitch1 正常 → J1 不限幅，原值返回 */
    return j1_target;
}

/* ======================== 夹爪控制 ======================== */

/**
 * @brief  夹爪控制（Q/E + 滚轮）
 * @note   Q/滚轮上 = 收缩（角度增加），E/滚轮下 = 张开（角度减少）
 *         正常模式和预设模式下均可使用
 */
void gripper_control(ArmContext_t *ctx)
{
    float step = GRIPPER_STEP;

    if (IF_KEY_PRESSED_Q_VT13 &&
        !IF_KEY_PRESSED_SHIFT_VT13) //&& !IF_KEY_PRESSED_CTRL_VT13 && !IF_KEY_PRESSED_SHIFT_VT13)
    {
        ctx->gripper_pos += step;
    }
    else if (IF_KEY_PRESSED_E_VT13 && !IF_KEY_PRESSED_SHIFT_VT13)
    {
        ctx->gripper_pos -= step;
    }
    else if (VT13_rc_ctrl.rc.wheel > 0)
    {
        ctx->gripper_pos += step;
    }
    else if (VT13_rc_ctrl.rc.wheel < 0)
    {
        ctx->gripper_pos -= step;
    }
    else
    {
        /* 无输入：保持当前位置，零刚度 */
        DM_custom_motor_MIT_ctrl(&hcan2, ARM_CAN_ID_GRIPPER, ctx->gripper_pos, 0.0f, GRIPPER_KP_HOLD, GRIPPER_KD_HOLD,
                                 0.0f);
        osDelay(1);
        return;
    }

    /* 过圈处理 */
    if (ctx->gripper_pos >= GRIPPER_POS_LIMIT || ctx->gripper_pos <= -GRIPPER_POS_LIMIT)
    {
        DM_motor_zeroset(&hcan2, ARM_CAN_ID_GRIPPER);
        osDelay(1);
    }

    DM_custom_motor_MIT_ctrl(&hcan2, ARM_CAN_ID_GRIPPER, ctx->gripper_pos, 0.0f, GRIPPER_KP, GRIPPER_KD, 0.0f);
}

/* ======================== J3 手动控制（预设模式专用） ======================== */

/**
 * @brief  J3 手动控制（C/V 键，仅在预设模式下使用）
 * @note   C = 增大 J3 偏移，V = 减小 J3 偏移
 *         偏移量叠加在预设目标角度上
 *         进入预设时偏移量归零，退出预设时偏移量丢弃
 */
void j3_manual_control(ArmContext_t *ctx)
{
    float step = J3_STEP_VT13;

    /* 必须排除 Ctrl/Shift 同时按下的情况，避免与预设切换冲突 */
    if (IF_KEY_PRESSED_C_VT13 && !IF_KEY_PRESSED_SHIFT_VT13 && !IF_KEY_PRESSED_CTRL_VT13)
    {
        ctx->j3_manual_offset += step;
    }
    else if (IF_KEY_PRESSED_V_VT13 && !IF_KEY_PRESSED_SHIFT_VT13 && !IF_KEY_PRESSED_CTRL_VT13)
    {
        ctx->j3_manual_offset -= step;
    }

    ctx->j3_manual_offset = clampf(ctx->j3_manual_offset, -ARM_POS_LIMIT, ARM_POS_LIMIT);
}

/**
 * @brief  J4 手动控制（R/G 键，仅在预设模式下使用）
 * @note   R = 增大 J4 偏移，G = 减小 J4 偏移
 *         偏移量叠加在预设目标角度上
 *         进入预设时偏移量归零，退出预设时偏移量丢弃
 */
void j4_manual_control(ArmContext_t *ctx)
{
    float step = J4_STEP_VT13;

    /* 必须排除 Ctrl/Shift 同时按下的情况，避免与预设切换冲突 */
    if (IF_KEY_PRESSED_R_VT13 && !IF_KEY_PRESSED_CTRL_VT13 && !IF_KEY_PRESSED_SHIFT_VT13)
    {
        ctx->j4_manual_offset += step;
    }
    else if (IF_KEY_PRESSED_G_VT13 && !IF_KEY_PRESSED_CTRL_VT13 && !IF_KEY_PRESSED_SHIFT_VT13)
    {
        ctx->j4_manual_offset -= step;
    }

    ctx->j4_manual_offset = clampf(ctx->j4_manual_offset, -69999.0f, 69999.0f);
}

/**
 * @brief  J1 手动控制（shift+Q/shift+E 键，仅在预设模式下使用）
 * @note   shift+Q = 增大 J1 偏移，shift+E = 减小 J1 偏移
 *         偏移量叠加在预设目标角度上
 *         进入预设时偏移量归零，退出预设时偏移量丢弃
 */
void j1_manual_control(ArmContext_t *ctx)
{
    float step = J1_STEP_VT13;

    /* 必须排除 Ctrl/Shift 同时按下的情况，避免与预设切换冲突 */
    if (IF_KEY_PRESSED_Q_VT13 && IF_KEY_PRESSED_SHIFT_VT13 && !IF_KEY_PRESSED_CTRL_VT13)
    {
        ctx->j1_manual_offset += step;
    }
    else if (IF_KEY_PRESSED_E_VT13 && IF_KEY_PRESSED_SHIFT_VT13 && !IF_KEY_PRESSED_CTRL_VT13)
    {
        ctx->j1_manual_offset -= step;
    }

    ctx->j1_manual_offset = clampf(ctx->j1_manual_offset, -ARM_POS_LIMIT, ARM_POS_LIMIT);
}

void j6j5_manual_control(ArmContext_t *ctx)
{
    /* ---- J6/J5 pitch 微调（Shift+V 增加 / Ctrl+V 减少） ---- */
    if (IF_KEY_PRESSED_V_VT13 && IF_KEY_PRESSED_SHIFT_VT13)
    {
        ctx->j6j5_pitch_manual_offset += J6J5_PITCH_STEP_VT13;
    }
    else if (IF_KEY_PRESSED_V_VT13 && IF_KEY_PRESSED_CTRL_VT13)
    {
        ctx->j6j5_pitch_manual_offset -= J6J5_PITCH_STEP_VT13;
    }
    ctx->j6j5_pitch_manual_offset = clampf(ctx->j6j5_pitch_manual_offset, -ARM_POS_LIMIT, ARM_POS_LIMIT);

    /* ---- J6/J5 yaw 微调（Shift+R 增加 / Shift+G 减少） ---- */
    if (IF_KEY_PRESSED_R_VT13 && IF_KEY_PRESSED_SHIFT_VT13)
    {
        ctx->j6j5_yaw_manual_offset += J6J5_YAW_STEP_VT13;
    }
    else if (IF_KEY_PRESSED_G_VT13 && IF_KEY_PRESSED_SHIFT_VT13)
    {
        ctx->j6j5_yaw_manual_offset -= J6J5_YAW_STEP_VT13;
    }
    ctx->j6j5_yaw_manual_offset = clampf(ctx->j6j5_yaw_manual_offset, -ARM_POS_LIMIT, ARM_POS_LIMIT);
}

/* ======================== B键边沿检测 ======================== */

/**
 * @brief  检测 B 键上升沿，切换锁定模式
 */
static void arm_update_lock_state(ArmContext_t *ctx)
{
    uint8_t current_b = (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_B) ? 1 : 0;

    if (current_b && !ctx->last_b_key)
    {
        if (!ctx->arm_locked)
        {
            /* 进入锁定：记录当前角度 */
            ctx->arm_locked = 1;
            for (int i = 0; i < ARM_MOTOR_COUNT; i++)
            {
                ctx->locked_pos[i] = data_motor.arm_motor_measure[i].pos;
            }
        }
        else
        {
            /* 退出锁定 */
            ctx->arm_locked = 0;
        }
    }
    ctx->last_b_key = current_b;
}

/**
 * @brief  【新增】F键边沿检测 —— 预设中途退出回零 / 回零后再按回正常
 *
 *   使用前提：system_ready == 1 时才响应（避免与初始化使能冲突）
 */
static void arm_update_exit_key(ArmContext_t *ctx)
{
    if (!ctx->system_ready)
        return;

    uint8_t cur_f = IF_KEY_PRESSED_F_VT13 ? 1 : 0;

    if (cur_f && !ctx->last_f_key)
    {
        if (ctx->preset_id != PRESET_NONE)
        {
            /* === 在预设模式中：退出到回零模式 === */

            /* 清理预设状态（复制你原有 PRESET_EXIT 宏的逻辑） */
            ctx->preset_id = PRESET_NONE;
            ctx->preset_step = 0;
            ctx->j3_manual_offset = 0.0f;
            ctx->pitch1_manual_offset = 0.0f;
            ctx->j1_manual_offset = 0.0f;
            ctx->j4_manual_offset = 0.0f;
            ctx->j6j5_pitch_manual_offset = 0.0f;
            ctx->j6j5_yaw_manual_offset = 0.0f;
            ctx->pitch1_pos = data_motor.arm_motor_measure[ARM_MOTOR_PITCH1].pos;
            ctx->gripper_pos = data_motor.arm_motor_measure[ARM_GRIPPER_IDX].pos;

            /* 进入回零模式，插值起点设为当前位置防止跳变 */
            ctx->mode = ARM_MODE_HOMING;
            ctx->homing_phase = 0;
            for (int i = 0; i < ARM_MOTOR_COUNT; i++)
            {
                ctx->interp_pos[i] = data_motor.arm_motor_measure[i].pos;
            }
        }
        else if (ctx->mode == ARM_MODE_HOMING)
        {
            /* === 在回零模式中：再按一下回到正常模式 === */
            ctx->mode = ARM_MODE_NORMAL;

            /* 同步外部 pos[] 防止切回 normal 时跳变（可选） */
            pos[0] = data_motor.arm_motor_measure[ARM_MOTOR_J6].pos;
            pos[1] = data_motor.arm_motor_measure[ARM_MOTOR_J5].pos;
            pos[2] = data_motor.arm_motor_measure[ARM_MOTOR_J4].pos;
            pos[3] = data_motor.arm_motor_measure[ARM_MOTOR_J3].pos;
            pos[4] = data_motor.arm_motor_measure[ARM_MOTOR_J1].pos;
            pos[5] = data_motor.arm_motor_measure[ARM_MOTOR_PITCH1].pos;
        }
    }

    ctx->last_f_key = cur_f;
}

/* ======================== 预设位姿 ======================== */

/**
 * @brief  预设位姿目标角度查找表
 *         每个 Ctrl 组合键有 MAX_PRESET_STEPS(3) 个预设位置
 *         每个 Shift 组合键有 1 个预设位置
 *         列顺序 = ArmMotorIndex_e: [J6, J5, J4, J3, J1, Pitch1]
 *         TODO: 根据实际需求调整各预设的目标角度
 */

/* Ctrl+Z: 步骤1→步骤2→步骤3→退出 */ // 取左上
static const float CTRL_Z_ANGLES[MAX_PRESET_STEPS][ARM_MOTOR_COUNT] = {
    /*  J6    J5    J4    J3    J1    Pitch1  */
    //    {-67.021f, 25.27f, 45.042f, -100.65f, -10.27f, -51.164f}, /* 步骤0 */
    //     {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -20.0f},                 /* 步骤1 */
    //     {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -25.0f},                 /* 步骤2 */
    //     {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},                 /* 步骤3（新增） */

    {-29.664f, -20.875f, -71.409f, -89.748f, -10.404f, -51.911f}, /* 步骤1 */
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -35.0f},                       /* 步骤2 */
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -40.0f},                       /* 步骤3 */
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, /* 占位 */
};

/* Ctrl+X: 步骤1→步骤2→步骤3→退出 */ // 存车左
static const float CTRL_X_ANGLES[MAX_PRESET_STEPS][ARM_MOTOR_COUNT] = {
    /*  J6    J5    J4    J3    J1    Pitch1  */
    {0.0f, 0.0f, 0.0f, -50.0f, 80.0f, -55.0f},              /* 步骤1 */
    {64.81f, -69.21f, -1.096f, -20.93f, 130.794f, -25.27f}, /* 步骤2 */
    {0.0f, 0.0f, 0.0f, -50.0f, 0.0f, -55.0f},               /* 步骤3 */
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},                   /* 步骤3（新增） */
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},                   /* 占位 */
};

/* Ctrl+C: 步骤1→步骤2→步骤3→退出 */ // 取车左
static const float CTRL_C_ANGLES[5][ARM_MOTOR_COUNT] = {
    /*  J6    J5    J4    J3    J1    Pitch1  */
    {0.0f, 0.0f, 0.0f, -50.0f, 80.0f, -55.0f},                 /* 步骤1 */
    {60.426f, -71.409f, -1.096f, -2.4719f, 131.299f, -4.735f}, /* 步骤2 */
    {80.198f, -53.232f, -1.096f, 0.252f, 131.299f, -47.516f},  /* 步骤3 */
    {0.0f, 0.0f, 0.0f, -50.0f, 0.0f, -55.0f},
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
};

/* Shift+Z: 取右上 */
static const float SHIFT_Z_ANGLES[MAX_PRESET_STEPS][ARM_MOTOR_COUNT] = {
    /*  J6    J5    J4    J3    J1    Pitch1  */
    // {-29.664f, -20.875f, -71.409f, -89.748f, -10.404f, -51.911f}, /* 步骤1 */
    // {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -35.0f}, /* 步骤2 */
    // {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -40.0f}, /* 步骤3 */
    // {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},

    {-67.021f, 25.27f, 45.042f, -100.65f, -10.27f, -51.164f}, /* 步骤0 */
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -20.0f},                   /* 步骤1 */
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -25.0f},                   /* 步骤2 */
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},                     /* 步骤3（新增） */
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},                     /* 占位 */
};

/* Shift+X: 存车右（与存车左对称，J1 取反） */
static const float SHIFT_X_ANGLES[MAX_PRESET_STEPS][ARM_MOTOR_COUNT] = {
    /*  J6    J5    J4    J3    J1    Pitch1  */
    {0.0f, 0.0f, 0.0f, -50.0f, -80.0f, -55.0f},
    {64.81f, -69.21f, -1.096f, -20.93f, -133.794f, -25.27f}, /* 步骤1（新增） */
    {0.0f, 0.0f, 0.0f, -50.0f, 0.0f, -55.0f},                /* 步骤2（新增） */
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},                    /* 步骤3（新增） */
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},                    /* 占位 */
};

/* Shift+C: 取车右（与取车左对称，J1 取反） */
static const float SHIFT_C_ANGLES[MAX_PRESET_STEPS][ARM_MOTOR_COUNT] = {
    /*  J6    J5    J4    J3    J1    Pitch1  */
    {0.0f, 0.0f, 0.0f, -50.0f, -80.0f, -55.0f},
    {60.426f, -71.409f, -1.096f, -2.4719f, -131.299f, -4.735f}, /* 步骤2 */
    {80.198f, -53.232f, -1.096f, 0.252f, -131.299f, -47.516f},  /* 步骤3 */
    {0.0f, 0.0f, 0.0f, -50.0f, 0.0f, -55.0f},
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
};

/**
 * @brief  更新预设目标角度（根据 preset_id + preset_step 查表，含 J3 手动偏移）
 */
void arm_preset_update_target(ArmContext_t *ctx)
{
    if (ctx->preset_id == PRESET_NONE || ctx->preset_id >= PRESET_COUNT)
        return;

    uint8_t step = ctx->preset_step;
    const float(*src)[ARM_MOTOR_COUNT] = NULL;

    switch (ctx->preset_id)
    {
    case PRESET_CTRL_Z:
        src = CTRL_Z_ANGLES;
        break;
    case PRESET_CTRL_X:
        src = CTRL_X_ANGLES;
        break;
    case PRESET_CTRL_C:
        src = CTRL_C_ANGLES;
        break;
    case PRESET_SHIFT_Z:
        src = SHIFT_Z_ANGLES;
        break;
    case PRESET_SHIFT_X:
        src = SHIFT_X_ANGLES;
        break;
    case PRESET_SHIFT_C:
        src = SHIFT_C_ANGLES;
        break;
    default:
        return;
    }

    /* 步骤越界保护 */

    uint8_t max_steps = PRESET_MAX_STEPS[ctx->preset_id];
    if (step >= max_steps)
        step = max_steps - 1;
    for (int i = 0; i < ARM_MOTOR_COUNT; i++)
    {
        ctx->preset_target[i] = src[step][i];
    }

    /* 叠加 J3 手动偏移量 */
    ctx->preset_target[ARM_MOTOR_J3] += ctx->j3_manual_offset;
    ctx->preset_target[ARM_MOTOR_J3] = clampf(ctx->preset_target[ARM_MOTOR_J3], -ARM_POS_LIMIT, ARM_POS_LIMIT);

    ctx->preset_target[ARM_MOTOR_PITCH1] += ctx->pitch1_manual_offset;
    ctx->preset_target[ARM_MOTOR_PITCH1] = clampf(ctx->preset_target[ARM_MOTOR_PITCH1], -ARM_POS_LIMIT, ARM_POS_LIMIT);

    ctx->preset_target[ARM_MOTOR_J4] += ctx->j4_manual_offset;
    ctx->preset_target[ARM_MOTOR_J4] = clampf(ctx->preset_target[ARM_MOTOR_J4], -70000.0f, 70000.0f);

    ctx->preset_target[ARM_MOTOR_J1] += ctx->j1_manual_offset;
    ctx->preset_target[ARM_MOTOR_J1] = clampf(ctx->preset_target[ARM_MOTOR_J1], -ARM_POS_LIMIT, ARM_POS_LIMIT);

    ctx->preset_target[ARM_MOTOR_J6] += ctx->j6j5_pitch_manual_offset;
    ctx->preset_target[ARM_MOTOR_J5] -= ctx->j6j5_pitch_manual_offset;

    /* 叠加 J6/J5 yaw 偏移量（两电机同向：J6+, J5+ → 小臂偏航） */
    ctx->preset_target[ARM_MOTOR_J6] += ctx->j6j5_yaw_manual_offset;
    ctx->preset_target[ARM_MOTOR_J5] += ctx->j6j5_yaw_manual_offset;

    /* J6/J5 限幅 */
    ctx->preset_target[ARM_MOTOR_J6] = clampf(ctx->preset_target[ARM_MOTOR_J6], -69999.0f, 69999.0f);
    ctx->preset_target[ARM_MOTOR_J5] = clampf(ctx->preset_target[ARM_MOTOR_J5], -69999.0f, 69999.0f);
}

/**
 * @brief  检测预设位姿是否到达
 * @retval 1: 到达, 0: 未到达
 */
uint8_t arm_preset_check_arrived(ArmContext_t *ctx)
{
    for (int i = 0; i < ARM_MOTOR_COUNT; i++)
    {
        ctx->pos_error[i] = ctx->preset_target[i] - data_motor.arm_motor_measure[i].pos;
        if (fabsf(ctx->pos_error[i]) > POS_ARRIVE_THRESH)
        {
            return 0;
        }
    }
    return 1;
}

/**
 * @brief  六种组合键边沿检测，切换预设编号和步骤
 *
 *   Ctrl+Z/X/C/V: 4次循环 (按1→步骤0, 按2→步骤1, 按3→步骤2, 按4→退出)
 *   Shift+Z/X:    2次切换 (按1→预设, 按2→退出)
 *   按不同组合键 → 切换到新预设（步骤从0开始）
 *
 *   进入预设时：归零步骤和J3偏移
 *   退出预设时：归零步骤和J3偏移，从电机反馈刷新位置
 *   步骤前进时：归零J3偏移
 */
void arm_update_preset_keys(ArmContext_t *ctx)
{
    /* ---- 检测各组合键的当前状态 ---- */
    uint8_t cur_ctrl_z = (IF_KEY_PRESSED_CTRL_VT13 && IF_KEY_PRESSED_Z_VT13) ? 1 : 0;
    uint8_t cur_ctrl_x = (IF_KEY_PRESSED_CTRL_VT13 && IF_KEY_PRESSED_X_VT13) ? 1 : 0;
    uint8_t cur_ctrl_c = (IF_KEY_PRESSED_CTRL_VT13 && IF_KEY_PRESSED_C_VT13) ? 1 : 0;
    uint8_t cur_shift_z = (IF_KEY_PRESSED_SHIFT_VT13 && IF_KEY_PRESSED_Z_VT13) ? 1 : 0;
    uint8_t cur_shift_x = (IF_KEY_PRESSED_SHIFT_VT13 && IF_KEY_PRESSED_X_VT13) ? 1 : 0;
    uint8_t cur_shift_c = (IF_KEY_PRESSED_SHIFT_VT13 && IF_KEY_PRESSED_C_VT13) ? 1 : 0;

/* ---- 辅助宏 ---- */
#define PRESET_ENTER(ctx, preset_id_val)                                                                               \
    do                                                                                                                 \
    {                                                                                                                  \
        (ctx)->preset_id = (preset_id_val);                                                                            \
        (ctx)->preset_step = 0;                                                                                        \
        (ctx)->j3_manual_offset = 0.0f;                                                                                \
        (ctx)->pitch1_manual_offset = 0.0f;                                                                            \
        (ctx)->j1_manual_offset = 0.0f;                                                                                \
        (ctx)->j4_manual_offset = 0.0f;                                                                                \
        (ctx)->j6j5_pitch_manual_offset = 0.0f;                                                                        \
        (ctx)->j6j5_yaw_manual_offset = 0.0f;                                                                          \
        arm_impedance_reset(ctx);                                                                                      \
        for (int _i = 0; _i < ARM_MOTOR_COUNT; _i++)                                                                   \
            (ctx)->interp_pos[_i] = data_motor.arm_motor_measure[_i].pos;                                              \
    } while (0)

#define PRESET_EXIT(ctx)                                                                                               \
    do                                                                                                                 \
    {                                                                                                                  \
        (ctx)->preset_id = PRESET_NONE;                                                                                \
        (ctx)->preset_step = 0;                                                                                        \
        (ctx)->j3_manual_offset = 0.0f;                                                                                \
        (ctx)->pitch1_manual_offset = 0.0f;                                                                            \
        (ctx)->j1_manual_offset = 0.0f;                                                                                \
        (ctx)->j4_manual_offset = 0.0f;                                                                                \
        (ctx)->j6j5_pitch_manual_offset = 0.0f;                                                                        \
        (ctx)->j6j5_yaw_manual_offset = 0.0f;                                                                          \
        (ctx)->pitch1_pos = data_motor.arm_motor_measure[ARM_MOTOR_PITCH1].pos;                                        \
        (ctx)->gripper_pos = data_motor.arm_motor_measure[ARM_GRIPPER_IDX].pos;                                        \
    } while (0)

    /* ---- Ctrl+Z: 取左上 ---- */
    if (cur_ctrl_z && !ctx->last_ctrl_z_key)
    {
        if (ctx->preset_id == PRESET_CTRL_Z)
        {
            ctx->preset_step++;
            if (ctx->preset_step >= PRESET_MAX_STEPS[PRESET_CTRL_Z])
                PRESET_EXIT(ctx);
            else
            {
                ctx->j3_manual_offset = 0.0f;
                ctx->pitch1_manual_offset = 0.0f;
            }
        }
        else if (ctx->preset_id == PRESET_NONE)
        {
            PRESET_ENTER(ctx, PRESET_CTRL_Z);
        }
    }

    /* ---- Ctrl+X: 存车左 ---- */
    if (cur_ctrl_x && !ctx->last_ctrl_x_key)
    {
        if (ctx->preset_id == PRESET_CTRL_X)
        {
            ctx->preset_step++;
            if (ctx->preset_step >= PRESET_MAX_STEPS[PRESET_CTRL_X])
                PRESET_EXIT(ctx);
            else
            {
                ctx->j3_manual_offset = 0.0f;
                ctx->pitch1_manual_offset = 0.0f;
            }
        }
        else if (ctx->preset_id == PRESET_NONE)
        {
            PRESET_ENTER(ctx, PRESET_CTRL_X);
        }
    }

    /* ---- Ctrl+C: 取车左 ---- */
    if (cur_ctrl_c && !ctx->last_ctrl_c_key)
    {
        if (ctx->preset_id == PRESET_CTRL_C)
        {
            ctx->preset_step++;
            if (ctx->preset_step >= PRESET_MAX_STEPS[PRESET_CTRL_C])
                PRESET_EXIT(ctx);
            else
            {
                ctx->j3_manual_offset = 0.0f;
                ctx->pitch1_manual_offset = 0.0f;
            }
        }
        else if (ctx->preset_id == PRESET_NONE)
        {
            PRESET_ENTER(ctx, PRESET_CTRL_C);
        }
    }

    /* ---- Shift+Z: 取右上 ---- */
    if (cur_shift_z && !ctx->last_shift_z_key)
    {
        if (ctx->preset_id == PRESET_SHIFT_Z)
        {
            ctx->preset_step++;
            if (ctx->preset_step >= PRESET_MAX_STEPS[PRESET_SHIFT_Z])
                PRESET_EXIT(ctx);
            else
            {
                ctx->j3_manual_offset = 0.0f;
                ctx->pitch1_manual_offset = 0.0f;
            }
        }
        else if (ctx->preset_id == PRESET_NONE)
        {
            PRESET_ENTER(ctx, PRESET_SHIFT_Z);
        }
    }

    /* ---- Shift+X: 存车右 ---- */
    if (cur_shift_x && !ctx->last_shift_x_key)
    {
        if (ctx->preset_id == PRESET_SHIFT_X)
        {
            ctx->preset_step++;
            if (ctx->preset_step >= PRESET_MAX_STEPS[PRESET_SHIFT_X])
                PRESET_EXIT(ctx);
            else
            {
                ctx->j3_manual_offset = 0.0f;
                ctx->pitch1_manual_offset = 0.0f;
            }
        }
        else if (ctx->preset_id == PRESET_NONE)
        {
            PRESET_ENTER(ctx, PRESET_SHIFT_X);
        }
    }

    /* ---- Shift+C: 取车右 ---- */
    if (cur_shift_c && !ctx->last_shift_c_key)
    {
        if (ctx->preset_id == PRESET_SHIFT_C)
        {
            ctx->preset_step++;
            if (ctx->preset_step >= PRESET_MAX_STEPS[PRESET_SHIFT_C])
                PRESET_EXIT(ctx);
            else
            {
                ctx->j3_manual_offset = 0.0f;
                ctx->pitch1_manual_offset = 0.0f;
            }
        }
        else if (ctx->preset_id == PRESET_NONE)
        {
            PRESET_ENTER(ctx, PRESET_SHIFT_C);
        }
    }

    /* ---- 保存本次状态供下次边沿检测 ---- */
    ctx->last_ctrl_z_key = cur_ctrl_z;
    ctx->last_ctrl_x_key = cur_ctrl_x;
    ctx->last_ctrl_c_key = cur_ctrl_c;
    ctx->last_shift_z_key = cur_shift_z;
    ctx->last_shift_x_key = cur_shift_x;
    ctx->last_shift_c_key = cur_shift_c;

#undef PRESET_ENTER
#undef PRESET_EXIT
}

/* ======================== 模式处理函数 ======================== */

/**
 * @brief  锁定模式：保持当前角度，不叠加重力补偿。
 */
void arm_mode_locked_handler(ArmContext_t *ctx)
{
    for (int i = 0; i < ARM_MOTOR_COUNT; i++)
    {
        float tau = 0.0f; // 锁定模式下暂不叠加重力补偿，保持位置锁定即可
        arm_send_mit_locked((ArmMotorIndex_e)i, ctx->locked_pos[i], tau);
    }
}

/**
 * @brief  预设位姿模式：梯形减速运动到目标
 *         支持手动微调：Z/X → Pitch1, C/V → J3, Q/E → 夹爪
 */
void arm_mode_preset_handler(ArmContext_t *ctx)
{
    /* ---- J3 手动微调（C/V） ---- */
    j3_manual_control(ctx);

    /* ---- J4 手动微调（R/G） ---- */
    j4_manual_control(ctx);

    /* ---- J1 手动微调（shift+Q/CTRL+E） ---- */
    j1_manual_control(ctx);

    /* ---- J6/J5 手动微调（shift+V/CTRL+v + shift+r/shift+g） ---- */
    j6j5_manual_control(ctx);

    /* ---- Pitch1 手动微调（Z/X，排除 Ctrl/Shift） ---- */
    if (IF_KEY_PRESSED_Z_VT13 && !IF_KEY_PRESSED_CTRL_VT13 && !IF_KEY_PRESSED_SHIFT_VT13)
    {
        ctx->pitch1_manual_offset += PITCH1_STEP_VT13;
    }
    else if (IF_KEY_PRESSED_X_VT13 && !IF_KEY_PRESSED_CTRL_VT13 && !IF_KEY_PRESSED_SHIFT_VT13)
    {
        ctx->pitch1_manual_offset -= PITCH1_STEP_VT13;
    }
    ctx->pitch1_manual_offset = clampf(ctx->pitch1_manual_offset, -ARM_POS_LIMIT, ARM_POS_LIMIT);

    /* ---- 更新预设目标角度（含 J3/Pitch1 偏移） ---- */
    arm_preset_update_target(ctx);

    /* ---- J1 联动保护：Pitch1 下压时限制 J1 旋转范围 ---- */
    ctx->preset_target[ARM_MOTOR_J1] = arm_j1_protection_clamp(ctx->preset_target[ARM_MOTOR_J1]);

    /* ---- 计算重力补偿力矩 ---- */
    ctx->gravity_torque = -GRAVITY_COMP_COEFF * GRAVITY_COMP_ARM *
                          cosf(GRAVITY_COMP_ANGLE - data_motor.arm_motor_measure[ARM_MOTOR_PITCH1].pos);
    /* ---- 计算位置误差 ---- */
    for (int i = 0; i < ARM_MOTOR_COUNT; i++)
    {
        ctx->pos_error[i] = ctx->preset_target[i] - data_motor.arm_motor_measure[i].pos;
    }

    ctx->preset_arrived = arm_preset_check_arrived(ctx);

    // 阻抗控制版
    if (!ctx->preset_arrived)
    {
        /* 每拍将 interp_pos 向目标步进一小步，实现恒速运动 */
        for (int i = 0; i < ARM_MOTOR_COUNT; i++)
        {
            float step_err = ctx->preset_target[i] - ctx->interp_pos[i];

            if (fabsf(step_err) <= PRESET_STEP_SIZE)
            {
                ctx->interp_pos[i] = ctx->preset_target[i];
            }
            else
            {
                ctx->interp_pos[i] += (step_err > 0.0f) ? PRESET_STEP_SIZE : -PRESET_STEP_SIZE;
            }
        }

        /* 发送插值位置（经阻抗滤波柔顺化） */
        for (int i = 0; i < ARM_MOTOR_COUNT; i++)
        {
            float compliant_pos = arm_impedance_filter((ArmMotorIndex_e)i, ctx->interp_pos[i]);
            float tau = 0.0f;
            arm_send_mit((ArmMotorIndex_e)i, compliant_pos, 0.0f, tau);
            osDelay(1);
        }
    }
    else
    {
        /* 已到达：锁定 + 阻抗柔顺（防碰撞） */
        for (int i = 0; i < ARM_MOTOR_COUNT; i++)
        {
            float compliant_pos = arm_impedance_filter((ArmMotorIndex_e)i, ctx->preset_target[i]);
            float tau = 0.0f;
            arm_send_mit_locked((ArmMotorIndex_e)i, compliant_pos, tau);
            osDelay(1);
        }
    }

    /* ---- 夹爪控制（Q/E + 滚轮） ---- */
    gripper_control(ctx);

    /* ---- 持续发送 Pitch1 指令（覆盖上面的 MIT 控制，确保最新微调生效 + 重力补偿） ---- */
}

/**
 * @brief  【新增】回零模式处理函数
 *         所有关节平滑插值到零位，到位后保持
 */
void arm_mode_homing_handler(ArmContext_t *ctx)
{
    /* 设置回零目标 */
    for (int i = 0; i < ARM_MOTOR_COUNT; i++)
    {
        ctx->homing_target[i] = 0.0f;
    }

    if (ctx->homing_phase == 0)
    {
        /* ---- 阶段0：只动 J1，其他轴保持当前位置 ---- */
        float j1_step_err = ctx->homing_target[ARM_MOTOR_J1] - ctx->interp_pos[ARM_MOTOR_J1];
        if (fabsf(j1_step_err) <= PRESET_STEP_SIZE)
        {
            ctx->interp_pos[ARM_MOTOR_J1] = ctx->homing_target[ARM_MOTOR_J1];
        }
        else
        {
            ctx->interp_pos[ARM_MOTOR_J1] += (j1_step_err > 0.0f) ? PRESET_STEP_SIZE : -PRESET_STEP_SIZE;
        }

        /* 其他轴跟踪当前实际位置，保持不动（防止外力导致跳变） */
        for (int i = 0; i < ARM_MOTOR_COUNT; i++)
        {
            if (i != ARM_MOTOR_J1)
            {
                ctx->interp_pos[i] = data_motor.arm_motor_measure[i].pos;
            }
        }

        /* 检测 J1 是否到位，到位后进入阶段1 */
        float j1_error = fabsf(ctx->homing_target[ARM_MOTOR_J1] - data_motor.arm_motor_measure[ARM_MOTOR_J1].pos);
        if (j1_error <= POS_ARRIVE_THRESH)
        {
            ctx->homing_phase = 1;
        }
    }
    else
    {
        /* ---- 阶段1：J1 到位后，所有关节一起回零 ---- */
        for (int i = 0; i < ARM_MOTOR_COUNT; i++)
        {
            float step_err = ctx->homing_target[i] - ctx->interp_pos[i];
            if (fabsf(step_err) <= PRESET_STEP_SIZE)
            {
                ctx->interp_pos[i] = ctx->homing_target[i];
            }
            else
            {
                ctx->interp_pos[i] += (step_err > 0.0f) ? PRESET_STEP_SIZE : -PRESET_STEP_SIZE;
            }
        }
    }

    /* 发送MIT指令（带阻抗滤波） */
    for (int i = 0; i < ARM_MOTOR_COUNT; i++)
    {
        float compliant_pos = arm_impedance_filter((ArmMotorIndex_e)i, ctx->interp_pos[i]);
        arm_send_mit((ArmMotorIndex_e)i, compliant_pos, 0.0f, 0.0f);
        osDelay(1);
    }

    /* 夹爪控制 */
    gripper_control(ctx);
}

/**
 * @brief  正常模式：遥控器直接控制
 *         所有关节（含 Pitch1）均由外部 pos[] 输入驱动
 *         pos[0~4] → J6/J5/J4/J3/J1
 *         pos[5]   → Pitch1（0x13）
 */
void arm_mode_normal_handler(ArmContext_t *ctx)
{
    /* ---- 限位处理 ---- */
    float limited_pos[5];
    limited_pos[4] = clampf(pos[4], -ARM_POS_LIMIT, ARM_POS_LIMIT);
    limited_pos[3] = clampf(pos[3], -ARM_POS_LIMIT, ARM_POS_LIMIT);
    limited_pos[2] = pos[2]; /* J4 暂无限位 */

    /* ---- J1 联动保护：Pitch1 下压时限制 J1 旋转范围 ---- */

    /* ---- J5/J6 合成角度（Roll + Pitch 解算） ---- */
    float roll = (pos[0] + pos[1]) / 2.0f;
    float pitch = (pos[0] - pos[1]) / 2.0f;

    limited_pos[0] = roll + pitch;
    limited_pos[1] = roll - pitch;

    /* ---- Pitch1 限位 ---- */
    float pitch1_target = -clampf(pos[5], -ARM_POS_LIMIT, ARM_POS_LIMIT);

    // 阻抗控制版
    /* ---- 发送各关节 MIT 控制（经阻抗滤波） ---- */
    ZeroCrossing(ARM_MOTOR_J6, limited_pos[0], 69999.0f, -69999.0f);
    osDelay(1);
    ZeroCrossing(ARM_MOTOR_J5, limited_pos[1], 69999.0f, -69999.0f);
    osDelay(1);
    ZeroCrossing(ARM_MOTOR_J4, limited_pos[2], 69999.0f, -69999.0f);
    osDelay(1);

    float cp0 = arm_impedance_filter(ARM_MOTOR_J6, limited_pos[0]);
    float cp1 = arm_impedance_filter(ARM_MOTOR_J5, limited_pos[1]);
    float cp2 = arm_impedance_filter(ARM_MOTOR_J4, limited_pos[2]);
    float cp3 = arm_impedance_filter(ARM_MOTOR_J3, limited_pos[3]);
    float cp4 = arm_impedance_filter(ARM_MOTOR_J1, limited_pos[4]);

    arm_custom_send_mit(ARM_MOTOR_J6, cp0, 0.0f, 0.0f);
    osDelay(1);
    arm_custom_send_mit(ARM_MOTOR_J5, cp1, 0.0f, 0.0f);
    osDelay(1);
    arm_custom_send_mit(ARM_MOTOR_J4, cp2, 0.0f, 0.0f);
    osDelay(1);

    arm_send_mit(ARM_MOTOR_J3, cp3, 0.0f, 0.0f);
    osDelay(1);
    arm_send_mit(ARM_MOTOR_J1, cp4, 0.0f, 0.0f);
    osDelay(1);

    /* ---- Pitch1 ---- */
    float cp5 = arm_impedance_filter(ARM_MOTOR_PITCH1, pitch1_target);
    ctx->pitch1_pos = cp5;
    DM_motor_MIT_ctrl(&hcan2, ARM_CAN_ID_PITCH1, cp5, 0.0f, ARM_MIT_PARAMS[ARM_MOTOR_PITCH1].kp,
                      ARM_MIT_PARAMS[ARM_MOTOR_PITCH1].kd, 0.0f);
    osDelay(1);

    /* ---- 夹爪控制 ---- */
    gripper_control(ctx);
}

/* ======================== 主任务入口 ======================== */

/**
 * @brief  机械臂控制主任务
 */
void arm_control_task(void)
{
    /* 初始化 */
    arm_init();
    osDelay(ARM_INIT_DELAY_MS);

    while (1)
    {
        /* ---- 裁判系统供电检测 ---- */
        if (data_referee.GameRobotState.power_management_gimbal_output == 1 && !arm_ctx.system_ready)
        {
            if (!arm_ctx.system_ready)
            {
                osDelay(2000);
                arm_reinit();
                arm_ctx.system_ready = 1;
            }
        }

        /* ---- 手动使能 (stop键) ---- */
        if (VT13_rc_ctrl.rc.stop == 1)
        {
            if (!arm_ctx.system_ready)
            {
                osDelay(1000);
                arm_reinit();
                arm_ctx.system_ready = 1;
            }
        }

        if (data_referee.GameRobotState.power_management_gimbal_output == 0)
        {
            arm_ctx.system_ready = 0;
        }
        /* ---- 电机在线检测 ---- */

        /* ---- B 键锁定切换 ---- */
        arm_update_lock_state(&arm_ctx);

        /* ---- 根据模式分发处理 ---- */
        if (arm_ctx.arm_locked)
        {
            arm_mode_locked_handler(&arm_ctx);
        }
        else
        {
            /* 预设组合键检测 */
            arm_update_preset_keys(&arm_ctx);

            /* F键退出检测 */
            arm_update_exit_key(&arm_ctx);

            if (arm_ctx.preset_id != PRESET_NONE)
            {
                arm_mode_preset_handler(&arm_ctx);
            }
            else if (arm_ctx.mode == ARM_MODE_HOMING)
            {
                arm_mode_homing_handler(&arm_ctx);
            }
            else
            {
                arm_mode_normal_handler(&arm_ctx);
            }
        }
        /* ---- 30Hz 扭矩上报 ---- */

        osDelay(10);
    }
}

/* ======================== 兼容旧接口 ======================== */
void custom_control(void)
{
    arm_control_task();
}

void custom_control_init(void)
{
    arm_init();
}
