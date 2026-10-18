#include "Data.h"

extern data_rc_t data_rc;
extern RC_ctrl_t rc_ctrl;
extern data_motor_t data_motor;
extern data_referee_t data_referee;
extern VT13_RC_ctrl_t VT13_rc_ctrl;
extern VT13_data_rc_t VT13_data_rc;
extern ArmContext_t arm_ctx;

pid_type_def chassis_pid_vel[4];
pid_type_def hex_chassis_pid_vel[HEX_MOTOR_NUM];
int16_t chassis_speed[4] = {0};
int16_t hex_speed_set[HEX_MOTOR_NUM] = {0};

// interaction_figure_t G1, G2, G3, G4, G5, G6, G7, G8, G9, G10, G11;
// String_Data CH_MODE;
// String_Data CH_STEP;
// String_Data OFFEST;
// #define MODE_STR_LEN  8          /* 最长 "Shift+Z" = 7，固定8避免Change长度变化 */
// #define STEP_STR_LEN  3          /* 最长 "S0*" = 3，固定3 */
//#define OFFEST_ANGLE  4          /* j1电机与车正方向角度偏差 */
//char mode_arr[MODE_STR_LEN + 1] = "NORMAL  "; /* 固定长度，不足补空格 */
//char step_arr[STEP_STR_LEN + 1] = "S0 ";      /* 固定长度，不足补空格 */
//char offest_angle[OFFEST_ANGLE + 1] ="+0.0";
// String_Data CH_SHOOT;
// String_Data CH_FLRB;
// char shoot_arr[5] = "shoot";
// char flrb_arr[4] = "FRBL";

// // 进度条配置参数
// #define PROGRESS_BAR_X 50                                          // 进度条左侧X坐标
// #define PROGRESS_BAR_Y 500                                         // 进度条Y坐标（底部）
// #define PROGRESS_BAR_HEIGHT 30                                     // 进度条高度
// #define PROGRESS_BAR_WIDTH 300                                     // 进度条最大宽度
// #define PROGRESS_TEXT_X (PROGRESS_BAR_X + PROGRESS_BAR_WIDTH + 20) // 文字位置
// #define PROGRESS_TEXT_Y (PROGRESS_BAR_Y + 5)                       // 文字Y坐标

//     // 进度条图形对象（需要全局或静态保存，用于后续修改）
//     static interaction_figure_t progress_bar;  // 进度条矩形
//     static interaction_figure_t progress_text; // 进度百分比文字
//     static u8 progress_initialized = 0;        // 初始化标志
//     static float current_progress = 0.0f;      // 当前进度值(0.0~1.0)

// 底盘控制参数配置
#define CHASSIS_SPEED_MAX 15000       // 最大速度
#define CHASSIS_SPEED_MAX_BOOST 15000 // 加速模式下最大速度
#define CHASSIS_ACCEL_STEP 2500       // 加速度步进//2500
#define CHASSIS_DECEL_STEP 4000       // 减速度步进
#define CHASSIS_MOUSE_YAW_SCALE 15    // 鼠标转向缩放系数
#define CHASSIS_MOUSE_YAW_MAX 3000    // 鼠标转向最大速度
#define ENABLE_S_CURVE_ACCEL 0        // 1=启用S曲线加减速, 0=禁用
#define ENABLE_HEX_S_CURVE_ACCEL 0    // 1=启用六足S曲线加减速, 0=禁用

#define M3508_GEAR_RATIO (15.76f)
#define M2006_GEAR_RATIO (36.0f)
#define WHEEL_DIAMETER_3508 (0.154f) // 3508轮径 100mm
#define WHEEL_DIAMETER_2006 (0.060f) // 2006轮径 50mm

// 缩放系数 = (2006减速比 / 3508减速比) * (3508轮径 / 2006轮径)
#define RPM_SCALE_2006_TO_3508 ((M2006_GEAR_RATIO / M3508_GEAR_RATIO) * \
                                (WHEEL_DIAMETER_3508 / WHEEL_DIAMETER_2006))

/** 底盘 PID 初始化，由 main 在 osKernelStart 前调用，避免 Chassis_Task 使用未初始化 PID */
void Chassis_PID_Init(void)
{
    PID_init(&chassis_pid_vel[0], 10.5f, 0.15f, 0.35f, 4000.0f, 16000.0f);
    PID_init(&chassis_pid_vel[1], 10.5f, 0.15f, 0.35f, 4000.0f, 16000.0f);
    PID_init(&chassis_pid_vel[2], 10.5f, 0.15f, 0.35f, 4000.0f, 16000.0f);
    PID_init(&chassis_pid_vel[3], 10.5f, 0.15f, 0.35f, 4000.0f, 16000.0f);

    PID_init(&hex_chassis_pid_vel[0], 5.5f, 0.05f, 0.25f, 4000.0f, 16000.0f);
    PID_init(&hex_chassis_pid_vel[1], 5.5f, 0.05f, 0.25f, 4000.0f, 16000.0f);
    PID_init(&hex_chassis_pid_vel[2], 5.5f, 0.05f, 0.25f, 4000.0f, 16000.0f);
    PID_init(&hex_chassis_pid_vel[3], 5.5f, 0.05f, 0.25f, 4000.0f, 16000.0f);
    // PID_init(&hex_chassis_pid_vel[4], 5.0f, 0.05f, 0.00f, 3000.0f, 1００００.０f);
    // PID_init(&hex_chassis_pid_vel[5], 5.０f, ０.０５f, ０.００f, 3０００.０f, 1００００.０f);
    PID_init(&hex_chassis_pid_vel[4], 7.0f, 0.2f, 0.002f, 4000.0f, 10000.0f);
    PID_init(&hex_chassis_pid_vel[5], 7.0f, 0.2f, 0.002f, 4000.0f, 10000.0f);
}

/**
 * @brief  根据机械臂当前状态返回模式名称字符串
 *         优先级: 锁定 > 预设 > 正常
 */
static char *arm_mode_get_name(void)
{
    /* 锁定模式优先 */
    if (arm_ctx.arm_locked)
        return "LOCKED";

    /* 预设模式 */
    switch (arm_ctx.preset_id)
    {
    case PRESET_CTRL_Z:
        return "Ctrl+Z";
    case PRESET_CTRL_X:
        return "Ctrl+X";
    case PRESET_CTRL_C:
        return "Ctrl+C";
    case PRESET_SHIFT_Z:
        return "Shift+Z";
    case PRESET_SHIFT_X:
        return "Shift+X";
    case PRESET_SHIFT_C:
        return "Shift+C";
    case PRESET_NONE:
    default:
        return "NORMAL";
    }
}
//char mode[5] = "shoot";
void Data_Task(void const *argument)
{
    // memset(&CH_MODE, 0, sizeof(CH_MODE)); // 模式文字
    // memset(&CH_STEP, 0, sizeof(CH_STEP)); // 步骤文字
    // // Line_Draw(&G1, "001", UI_Graph_ADD, 9, UI_Color_Main, 3, 0, 0, 1920, 1080);
    // // Rectangle_Draw(&G2, "002", UI_Graph_ADD, 9, UI_Color_Main, 3, 100, 100, 1720, 880);
    // // Circle_Draw(&G3, "003", UI_Graph_ADD, 9, UI_Color_Main, 3, 960, 540, 200);
    // // Arc_Draw(&G4, "004", UI_Graph_ADD, 9, UI_Color_Main, 0, 90, 3, 960, 540, 150, 150);
    // // Float_Draw(&G5, "005", UI_Graph_ADD, 9, UI_Color_Main, 30, 3, 100, 500, VT13_data_rc.angle_pitch);
    
    //                                       /* 画模式名称（层8） */
    // Char_Draw(&CH_MODE, "081", UI_Graph_ADD, 8, UI_Color_Yellow, 24, MODE_STR_LEN, 4, 80, 700, &mode_arr[0]);
    // Char_ReFresh(CH_MODE);
    // osDelay(30); /* 避免连续发送导致第二个包被丢弃 */

    // /* 画预设步骤（层8） */
    // Char_Draw(&CH_STEP, "082", UI_Graph_ADD, 8, UI_Color_Yellow, 24, STEP_STR_LEN, 4, 80, 650, &step_arr[0]);
    // Char_ReFresh(CH_STEP);
    // osDelay(30);

    // Char_Draw(&OFFEST , "083", UI_Graph_ADD, 8 ,UI_Color_Yellow , 24 ,OFFEST_ANGLE,4 ,80 ,600 ,&offest_angle[0]);
    // Char_ReFresh(OFFEST);
    // osDelay(30);
    // /* 临时调试：显示按键值和 preset_id */
    // Int_Draw(&G6, "006", UI_Graph_ADD, 9, UI_Color_Green, 20, 3, 500, 500, VT13_rc_ctrl.key.v);
    // Int_Draw(&G7, "007", UI_Graph_ADD, 9, UI_Color_Cyan, 20, 3, 500, 550, arm_ctx.preset_id);
    // UI_ReFresh(2, G6, G7);
    // Char_Draw(&decision, "006", UI_Graph_ADD, 9, UI_Color_Green, 30, 5, 3, 100, 100, &arm_ctx.preset_id);
    // UI_Delete(UI_Data_Del_Layer, 9);
    // osDelay(1);
    {
        // char mode[7]="shoot";
        //  char *mode_str = arm_mode_get_name();
        //  uint32_t mode_len = strlen(mode_str);
        //  Char_Draw(&decision, "006", UI_Graph_ADD, 9, UI_Color_Green, 20, mode_len, 2, 500, 500, mode_str);
        //  Char_ReFresh(decision);
    }
    while (1)
    {
#if USE_VT13_REMOTE_CONTROL
        // VT13图传遥控器数据读取和速度解算
        VT13_data_rc.VT13_ctrl = get_VT13_remote_control_point();
        VT13_read_data(&VT13_data_rc);
        speed_calculation(&VT13_data_rc, VT13_data_rc.status_chassis, VT13_data_rc.status_gimbal, VT13_data_rc.status_ammo);
        osDelay(8);
#else
        data_rc.RC_ctrl = get_remote_control_point();
        rc_read_data(&data_rc);
        rc_speed_calculation(&data_rc, data_rc.status_chassis, data_rc.status_gimbal, data_rc.status_ammo);
        osDelay(10);
#endif
        // Float_Draw(&G5, "005", UI_Graph_Change, 9, UI_Color_Main, 30, 3, 100, 500, VT13_data_rc.angle_pitch);
        // UI_ReFresh(5, G1, G2, G3, G4, G5);
        // osDelay(1);
        // Char_Draw(&decision, "006", UI_Graph_Change, 9, UI_Color_Purplish_red, 30, 5, 3, 100, 100, &arm_ctx.preset_id); // 需要判断状态来刷新显示内容，暂时先固定显示"决策"，后续根据实际需求修改为不同的字符串
        // Char_ReFresh(decision);
        /* ---- 更新模式名称 ---- */
        // {
        //     char *name = arm_mode_get_name();
        //     uint32_t raw_len = strlen(name);
        //     memset(mode_arr, ' ', MODE_STR_LEN);
        //     memcpy(mode_arr, name, raw_len < MODE_STR_LEN ? raw_len : MODE_STR_LEN);
        //     mode_arr[MODE_STR_LEN] = '\0';
        //     /* 先删后加：裁判系统 Change 对字符串内容不生效 */
        //     Char_Draw(&CH_MODE, "081", UI_Graph_Change, 8, UI_Color_Yellow,
        //               24, MODE_STR_LEN, 4, 80, 700, &mode_arr[0]);
        //     Char_ReFresh(CH_MODE);
        //     osDelay(10);
        //     // Char_Draw(&CH_MODE, "081", UI_Graph_ADD, 8,
        //     //           arm_ctx.arm_locked ? UI_Color_Pink : UI_Color_Yellow,
        //     //           24, MODE_STR_LEN, 4, 80, 700, &mode_arr[0]);
        //     // Char_ReFresh(CH_MODE);
        //     // osDelay(10);
        // }

        // /* ---- 更新预设步骤 ---- */
        // {
        //     memset(step_arr, ' ', STEP_STR_LEN);
        //     if (arm_ctx.preset_id != PRESET_NONE)
        //     {
        //         step_arr[0] = 'S';
        //         step_arr[1] = '0' + arm_ctx.preset_step;
        //         step_arr[2] = arm_ctx.preset_arrived ? '*' : ' ';
        //     }
        //     else
        //     {
        //         step_arr[0] = '-';
        //         step_arr[1] = '-';
        //     }
        //     step_arr[STEP_STR_LEN] = '\0';
        //     /* 先删后加 */
        //     Char_Draw(&CH_STEP, "082", UI_Graph_Del, 8, UI_Color_Yellow,
        //               24, STEP_STR_LEN, 4, 90, 650, &step_arr[0]);
        //     Char_ReFresh(CH_STEP);
        //     osDelay(10);
        //     Char_Draw(&CH_STEP, "082", UI_Graph_ADD, 8,
        //               (arm_ctx.preset_id != PRESET_NONE)
        //                   ? (arm_ctx.preset_arrived ? UI_Color_Green : UI_Color_Orange)
        //                   : UI_Color_Yellow,
        //               24, STEP_STR_LEN, 4, 90, 650, &step_arr[0]);
        //     Char_ReFresh(CH_STEP);
        // }

        /* 临时调试：更新按键值和 preset_id 显示 */
        // Int_Draw(&G6, "006", UI_Graph_Change, 9, UI_Color_Green, 20, 3, 500, 500, VT13_rc_ctrl.key.v);
        // Int_Draw(&G7, "007", UI_Graph_Change, 9, UI_Color_Cyan, 20, 3, 500, 550, arm_ctx.preset_id);
        // UI_ReFresh(2, G6, G7);

        /* 频率控制 10Hz */
        osDelay(2);
        // {
        //     char *angle=(char)data_motor.arm_motor_measure[5].pos;
        //     memset(offest_angle, ' ', OFFEST_ANGLE);
        //     memcpy(offest_angle, angle, 3);
        //     offest_angle[OFFEST_ANGLE] = '\0';
        //     /* 先删后加：裁判系统 Change 对字符串内容不生效 */
        //     Char_Draw(&OFFEST, "083", UI_Graph_Del, 8, UI_Color_Yellow,
        //               24, OFFEST_ANGLE, 4, 80, 600, &offest_angle[0]);
        //     Char_ReFresh(OFFEST);
        //     osDelay(10);
        //     Char_Draw(&OFFEST, "083", UI_Graph_ADD, 8 , UI_Color_Yellow,
        //                24, OFFEST_ANGLE, 4, 80, 600, &offest_angle[0]);
        //     Char_ReFresh(OFFEST);
        //     osDelay(10);

        // }
        // static float hp_percent = 1.0f; // 血量百分比
        // // 计算当前血量百分比（示例）
        // hp_percent = (float)data_referee.GameRobotHP.blue_2_robot_HP / 250.0f; // 假设最大HP为250，根据实际情况调整
        // // 初始化（只需调用一次）
        // if (!progress_initialized)
        // {
        //     ProgressBar_Init(hp_percent);
        // }
        // else
        // {
        //     // 更新进度条
        //     ProgressBar_Update(hp_percent);
        // }
    }
}
/**
 * @brief VT13速度解算函数 - 支持双键控制和Shift加速
 * @param rc_data 遥控器数据
 * @param chassis_status 底盘状态
 * @param gimbal_status 云台状态
 * @param ammo_status 发射状态
 */
void speed_calculation(VT13_data_rc_t *rc_data_VT13, uint8_t chassis_status, uint8_t gimbal_status, uint8_t ammo_status)
{
    // 底盘速度解算
    switch (chassis_status)
    {
    case CHASSIS_STATUS_STOP: // 静止状态
        rc_data_VT13->speed_x = 0;
        rc_data_VT13->speed_y = 0;
        rc_data_VT13->speed_yaw = 0;
        break;

    case CHASSIS_STATUS_FOLLOW: // 跟随状态
    {
        // 根据是否按住Shift决定最大速度
        int16_t speed_max = IF_KEY_PRESSED_SHIFT_VT13 ? CHASSIS_SPEED_MAX_BOOST : CHASSIS_SPEED_MAX;

        // ========== X轴目标速度计算（左右移动：A/D键） ==========
        int16_t target_vx = 0;
        if (IF_KEY_PRESSED_A_VT13)
        {
            target_vx = -speed_max;
        }
        else if (IF_KEY_PRESSED_D_VT13)
        {
            target_vx = speed_max;
        }

        // ========== Y轴目标速度计算（前后移动：W/S键） ==========
        int16_t target_vy = 0;
        if (IF_KEY_PRESSED_W_VT13)
        {
            target_vy = speed_max;
        }
        else if (IF_KEY_PRESSED_S_VT13)
        {
            target_vy = -speed_max;
        }

#if ENABLE_S_CURVE_ACCEL
        // 使用S曲线加减速平滑过渡
        rc_data_VT13->speed_x = speed_ramp_s_curve(rc_data_VT13->speed_x, target_vx, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
        rc_data_VT13->speed_y = speed_ramp_s_curve(rc_data_VT13->speed_y, target_vy, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
#else
        // 线性加减速（原始方式）
        if (target_vx > rc_data_VT13->speed_x)
        {
            rc_data_VT13->speed_x += CHASSIS_ACCEL_STEP;
            if (rc_data_VT13->speed_x > target_vx)
                rc_data_VT13->speed_x = target_vx;
        }
        else if (target_vx < rc_data_VT13->speed_x)
        {
            rc_data_VT13->speed_x -= CHASSIS_ACCEL_STEP;
            if (rc_data_VT13->speed_x < target_vx)
                rc_data_VT13->speed_x = target_vx;
        }

        if (target_vy > rc_data_VT13->speed_y)
        {
            rc_data_VT13->speed_y += CHASSIS_ACCEL_STEP;
            if (rc_data_VT13->speed_y > target_vy)
                rc_data_VT13->speed_y = target_vy;
        }
        else if (target_vy < rc_data_VT13->speed_y)
        {
            rc_data_VT13->speed_y -= CHASSIS_ACCEL_STEP;
            if (rc_data_VT13->speed_y < target_vy)
                rc_data_VT13->speed_y = target_vy;
        }
#endif

        // 额外限幅保护
        if (rc_data_VT13->speed_x > speed_max)
            rc_data_VT13->speed_x = speed_max;
        if (rc_data_VT13->speed_x < -speed_max)
            rc_data_VT13->speed_x = -speed_max;
        if (rc_data_VT13->speed_y > speed_max)
            rc_data_VT13->speed_y = speed_max;
        if (rc_data_VT13->speed_y < -speed_max)
            rc_data_VT13->speed_y = -speed_max;

        // ========== 转向控制（鼠标） ==========
        static int16_t last_mouse_yaw = 0;
        int16_t yaw_input = 0;

        // 优先使用鼠标控制，如果鼠标无输入则使用摇杆CH3
        if (abs(VT13_rc_ctrl.mouse.x) > 10) // 鼠标有输入
        {
            yaw_input = VT13_rc_ctrl.mouse.x * CHASSIS_MOUSE_YAW_SCALE;
        }
        else // 使用摇杆CH3
        {
            yaw_input = (int16_t)VT13_rc_ctrl.rc.ch[3] * 15; // 根据实际需要调整缩放系数
        }

        // 限幅
        if (yaw_input > CHASSIS_MOUSE_YAW_MAX)
            yaw_input = CHASSIS_MOUSE_YAW_MAX;
        if (yaw_input < -CHASSIS_MOUSE_YAW_MAX)
            yaw_input = -CHASSIS_MOUSE_YAW_MAX;

        // 平滑处理
        //rc_data_VT13->speed_yaw = (int16_t)(last_mouse_yaw * 0.7f + yaw_input * 0.3f);
        rc_data_VT13->speed_yaw = yaw_input;
        last_mouse_yaw = rc_data_VT13->speed_yaw;

        // 注意：根据实际电机方向可能需要调整符号
        int16_t vx = rc_data_VT13->speed_x;   // 左右速度
        int16_t vy = rc_data_VT13->speed_y;   // 前后速度
        int16_t vw = rc_data_VT13->speed_yaw; // 旋转速度

        chassis_speed[0] = -vx - vy - vw;
        chassis_speed[1] = -vx + vy - vw;
        chassis_speed[2] = +vx - vy - vw;
        chassis_speed[3] = +vx + vy - vw;
        break;
    }

    case CHASSIS_STATUS_SPIN: // 六足模式
                              //     {
                              //         // 根据是否按住Shift决定最大速度
                              //         int16_t speed_max = IF_KEY_PRESSED_SHIFT_VT13 ? CHASSIS_SPEED_MAX_BOOST : CHASSIS_SPEED_MAX;

        //         // ========== X轴目标速度计算（左右移动：A/D键） ==========
        //         int16_t target_vx = 0;
        //         if (IF_KEY_PRESSED_A_VT13)
        //         {
        //             target_vx = -speed_max;
        //         }
        //         else if (IF_KEY_PRESSED_D_VT13)
        //         {
        //             target_vx = speed_max;
        //         }

        //         // ========== Y轴目标速度计算（前后移动：W/S键） ==========
        //         int16_t target_vy = 0;
        //         if (IF_KEY_PRESSED_W_VT13)
        //         {
        //             target_vy = speed_max;
        //         }
        //         else if (IF_KEY_PRESSED_S_VT13)
        //         {
        //             target_vy = -speed_max;
        //         }

        // #if ENABLE_HEX_S_CURVE_ACCEL
        //         // 使用S曲线加减速平滑过渡
        //         rc_data_VT13->speed_x = hex_speed_ramp_s_curve(rc_data_VT13->speed_x, target_vx, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
        //         rc_data_VT13->speed_y = hex_speed_ramp_s_curve(rc_data_VT13->speed_y, target_vy, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
        // #else
        //         // 线性加减速（原始方式）
        //         if (target_vx > rc_data_VT13->speed_x)
        //         {
        //             rc_data_VT13->speed_x += CHASSIS_ACCEL_STEP;
        //             if (rc_data_VT13->speed_x > target_vx)
        //                 rc_data_VT13->speed_x = target_vx;
        //         }
        //         else if (target_vx < rc_data_VT13->speed_x)
        //         {
        //             rc_data_VT13->speed_x -= CHASSIS_DECEL_STEP;
        //             if (rc_data_VT13->speed_x < target_vx)
        //                 rc_data_VT13->speed_x = target_vx;
        //         }

        //         if (target_vy > rc_data_VT13->speed_y)
        //         {
        //             rc_data_VT13->speed_y += CHASSIS_ACCEL_STEP;
        //             if (rc_data_VT13->speed_y > target_vy)
        //                 rc_data_VT13->speed_y = target_vy;
        //         }
        //         else if (target_vy < rc_data_VT13->speed_y)
        //         {
        //             rc_data_VT13->speed_y -= CHASSIS_DECEL_STEP;
        //             if (rc_data_VT13->speed_y < target_vy)
        //                 rc_data_VT13->speed_y = target_vy;
        //         }
        // #endif

        //         // 限幅保护
        //         if (rc_data_VT13->speed_x > speed_max)
        //             rc_data_VT13->speed_x = speed_max;
        //         if (rc_data_VT13->speed_x < -speed_max)
        //             rc_data_VT13->speed_x = -speed_max;
        //         if (rc_data_VT13->speed_y > speed_max)
        //             rc_data_VT13->speed_y = speed_max;
        //         if (rc_data_VT13->speed_y < -speed_max)
        //             rc_data_VT13->speed_y = -speed_max;

        //         // ========== 转向控制（鼠标） ==========
        //         static int16_t last_mouse_yaw = 0;
        //         int16_t yaw_input = 0;

        //         // 优先使用鼠标控制，如果鼠标无输入则使用摇杆CH3
        //         if (abs(VT13_rc_ctrl.mouse.x) > 10) // 鼠标有输入
        //         {
        //             yaw_input = VT13_rc_ctrl.mouse.x * CHASSIS_MOUSE_YAW_SCALE;
        //         }
        //         else // 使用摇杆CH3
        //         {
        //             yaw_input = (int16_t)VT13_rc_ctrl.rc.ch[3] * 15; // 根据实际需要调整缩放系数
        //         }

        //         // 限幅
        //         if (yaw_input > CHASSIS_MOUSE_YAW_MAX)
        //             yaw_input = CHASSIS_MOUSE_YAW_MAX;
        //         if (yaw_input < -CHASSIS_MOUSE_YAW_MAX)
        //             yaw_input = -CHASSIS_MOUSE_YAW_MAX;

        //         // 平滑处理
        //         // rc_data_VT13->speed_yaw = (int16_t)(last_mouse_yaw * 0.7f + yaw_input * 0.3f);
        //         rc_data_VT13->speed_yaw = yaw_input;
        //         last_mouse_yaw = rc_data_VT13->speed_yaw;

        //         // 注意：根据实际电机方向可能需要调整符号
        //         int16_t vx = rc_data_VT13->speed_x;   // 左右速度
        //         int16_t vy = rc_data_VT13->speed_y;   // 前后速度
        //         int16_t vw = rc_data_VT13->speed_yaw; // 旋转速度
        //         // 六轮
        //         hex_speed_set[0] = -vx - vy - vw; // 左前
        //         hex_speed_set[1] = -vx + vy - vw; // 右前
        //         hex_speed_set[2] = vx - vy - vw;  // 左后
        //         hex_speed_set[3] = vx + vy - vw;  // 右后
        //         // hex_speed_set[4] = (+ vy + vw)*RPM_SCALE_2006_TO_3508; // 左中
        //         // hex_speed_set[5] = (- vy + vw)*RPM_SCALE_2006_TO_3508; // 右中
        //         hex_speed_set[4] = (+vy + vw) * RPM_SCALE_2006_TO_3508; //*2.284; //1.875
        //         hex_speed_set[5] = (-vy + vw) * RPM_SCALE_2006_TO_3508; //*2.284; //1.875
        //         if(hex_speed_set[4] > speed_max)
        //             hex_speed_set[4] = speed_max;
        //         if(hex_speed_set[4] < -speed_max)
        //             hex_speed_set[4] = -speed_max;
        //         if(hex_speed_set[5] > speed_max)
        //             hex_speed_set[5] = speed_max;
        //         if(hex_speed_set[5] < -speed_max)
        //             hex_speed_set[5] = -speed_max;
        //         break;
        //     }

        {
            // 根据是否按住Shift决定最大速度
            int16_t speed_max = IF_KEY_PRESSED_SHIFT_VT13 ? CHASSIS_SPEED_MAX_BOOST : CHASSIS_SPEED_MAX;

            // ========== X轴目标速度计算（左右移动：A/D键） ==========
            int16_t target_vx = 0;
            if (IF_KEY_PRESSED_A_VT13)
            {
                target_vx = -speed_max;
            }
            else if (IF_KEY_PRESSED_D_VT13)
            {
                target_vx = speed_max;
            }

            // ========== Y轴目标速度计算（前后移动：W/S键） ==========
            int16_t target_vy = 0;
            if (IF_KEY_PRESSED_W_VT13)
            {
                target_vy = speed_max;
            }
            else if (IF_KEY_PRESSED_S_VT13)
            {
                target_vy = -speed_max;
            }

#if ENABLE_S_CURVE_ACCEL
            // 使用S曲线加减速平滑过渡
            rc_data_VT13->speed_x = speed_ramp_s_curve(rc_data_VT13->speed_x, target_vx, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
            rc_data_VT13->speed_y = speed_ramp_s_curve(rc_data_VT13->speed_y, target_vy, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
#else
            // 线性加减速（原始方式）
            if (target_vx > rc_data_VT13->speed_x)
            {
                rc_data_VT13->speed_x += CHASSIS_ACCEL_STEP;
                if (rc_data_VT13->speed_x > target_vx)
                    rc_data_VT13->speed_x = target_vx;
            }
            else if (target_vx < rc_data_VT13->speed_x)
            {
                rc_data_VT13->speed_x -= CHASSIS_ACCEL_STEP;
                if (rc_data_VT13->speed_x < target_vx)
                    rc_data_VT13->speed_x = target_vx;
            }

            if (target_vy > rc_data_VT13->speed_y)
            {
                rc_data_VT13->speed_y += CHASSIS_ACCEL_STEP;
                if (rc_data_VT13->speed_y > target_vy)
                    rc_data_VT13->speed_y = target_vy;
            }
            else if (target_vy < rc_data_VT13->speed_y)
            {
                rc_data_VT13->speed_y -= CHASSIS_ACCEL_STEP;
                if (rc_data_VT13->speed_y < target_vy)
                    rc_data_VT13->speed_y = target_vy;
            }
#endif

            // 额外限幅保护
            if (rc_data_VT13->speed_x > speed_max)
                rc_data_VT13->speed_x = speed_max;
            if (rc_data_VT13->speed_x < -speed_max)
                rc_data_VT13->speed_x = -speed_max;
            if (rc_data_VT13->speed_y > speed_max)
                rc_data_VT13->speed_y = speed_max;
            if (rc_data_VT13->speed_y < -speed_max)
                rc_data_VT13->speed_y = -speed_max;

            // ========== 转向控制（鼠标） ==========
            static int16_t last_mouse_yaw = 0;
            int16_t yaw_input = 0;

            // 优先使用鼠标控制，如果鼠标无输入则使用摇杆CH3
            if (abs(VT13_rc_ctrl.mouse.x) > 10) // 鼠标有输入
            {
                yaw_input = VT13_rc_ctrl.mouse.x * CHASSIS_MOUSE_YAW_SCALE;
            }
            else // 使用摇杆CH3
            {
                yaw_input = (int16_t)VT13_rc_ctrl.rc.ch[3] * 15; // 根据实际需要调整缩放系数
            }

            // 限幅
            if (yaw_input > CHASSIS_MOUSE_YAW_MAX)
                yaw_input = CHASSIS_MOUSE_YAW_MAX;
            if (yaw_input < -CHASSIS_MOUSE_YAW_MAX)
                yaw_input = -CHASSIS_MOUSE_YAW_MAX;

            // 平滑处理
            // rc_data_VT13->speed_yaw = (int16_t)(last_mouse_yaw * 0.7f + yaw_input * 0.3f);
            rc_data_VT13->speed_yaw = yaw_input;
            last_mouse_yaw = rc_data_VT13->speed_yaw;

            // 注意：根据实际电机方向可能需要调整符号
            int16_t vx = rc_data_VT13->speed_x;   // 左右速度
            int16_t vy = rc_data_VT13->speed_y;   // 前后速度
            int16_t vw = rc_data_VT13->speed_yaw; // 旋转速度

            chassis_speed[0] = -vx - vy - vw;
            chassis_speed[1] = -vx + vy - vw;
            chassis_speed[2] = +vx - vy - vw;
            chassis_speed[3] = +vx + vy - vw;
            break;
        }

    default:
        break;
    }
}

#undef CHASSIS_ACCEL_STEP
#define CHASSIS_ACCEL_STEP 100
#undef CHASSIS_DECEL_STEP
#define CHASSIS_DECEL_STEP 200

static int16_t rc_axis_target(int16_t joystick, uint16_t negative_key,
                              uint16_t positive_key, int16_t speed_max)
{
    int32_t target = (int32_t)joystick * CHASSIS_OPEN_RC_SCALE;

    if (rc_ctrl.key.v & negative_key)
        target = -speed_max;
    else if (rc_ctrl.key.v & positive_key)
        target = speed_max;

    if (target > speed_max)
        target = speed_max;
    else if (target < -speed_max)
        target = -speed_max;

    return (int16_t)target;
}

void rc_speed_calculation(data_rc_t *rc_data, uint8_t chassis_status, uint8_t gimbal_status, uint8_t ammo_status)
{
    // 底盘速度解算
    switch (chassis_status)
    {
    case 0: // 静止状态
        rc_data->speed_x = 0;
        rc_data->speed_y = 0;
        rc_data->speed_yaw = 0;
        break;

    case 1: // 跟随状态
    {
        // 根据是否按住Shift决定最大速度
        int16_t speed_max = (rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) ? CHASSIS_SPEED_MAX_BOOST : CHASSIS_SPEED_MAX;

        // ========== X轴目标速度计算（左右移动：A/D键） ==========
        int16_t target_vx = -rc_axis_target(rc_ctrl.rc.ch[0],
                                           KEY_PRESSED_OFFSET_A,
                                           KEY_PRESSED_OFFSET_D, speed_max);

        // ========== Y轴目标速度计算（前后移动：W/S键） ==========
        int16_t target_vy = rc_axis_target(rc_ctrl.rc.ch[1],
                                           KEY_PRESSED_OFFSET_S,
                                           KEY_PRESSED_OFFSET_W, speed_max);

#if ENABLE_S_CURVE_ACCEL
        // 使用S曲线加减速平滑过渡
        rc_data->speed_x = speed_ramp_s_curve(rc_data->speed_x, target_vx, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
        rc_data->speed_y = speed_ramp_s_curve(rc_data->speed_y, target_vy, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
#else
        // 线性加减速（原始方式）
        if (target_vx > rc_data->speed_x)
        {
            rc_data->speed_x += CHASSIS_ACCEL_STEP;
            if (rc_data->speed_x > target_vx)
                rc_data->speed_x = target_vx;
        }
        else if (target_vx < rc_data->speed_x)
        {
            rc_data->speed_x -= CHASSIS_ACCEL_STEP;
            if (rc_data->speed_x < target_vx)
                rc_data->speed_x = target_vx;
        }

        if (target_vy > rc_data->speed_y)
        {
            rc_data->speed_y += CHASSIS_ACCEL_STEP;
            if (rc_data->speed_y > target_vy)
                rc_data->speed_y = target_vy;
        }
        else if (target_vy < rc_data->speed_y)
        {
            rc_data->speed_y -= CHASSIS_ACCEL_STEP;
            if (rc_data->speed_y < target_vy)
                rc_data->speed_y = target_vy;
        }
#endif

        // 额外限幅保护
        if (rc_data->speed_x > speed_max)
            rc_data->speed_x = speed_max;
        if (rc_data->speed_x < -speed_max)
            rc_data->speed_x = -speed_max;
        if (rc_data->speed_y > speed_max)
            rc_data->speed_y = speed_max;
        if (rc_data->speed_y < -speed_max)
            rc_data->speed_y = -speed_max;

        // ========== 转向控制（鼠标） ==========
        static int16_t last_mouse_yaw = 0;
        int16_t yaw_input = 0;

        // 优先使用鼠标控制，如果鼠标无输入则使用摇杆CH3
        if (abs(rc_ctrl.mouse.x) > 10) // 鼠标有输入
        {
            yaw_input = rc_ctrl.mouse.x * CHASSIS_MOUSE_YAW_SCALE;
        }
        else // 使用摇杆CH3
        {
            yaw_input = (int16_t)rc_ctrl.rc.ch[2] * 15; // 根据实际需要调整缩放系数
        }

        // 限幅
        if (yaw_input > CHASSIS_MOUSE_YAW_MAX)
            yaw_input = CHASSIS_MOUSE_YAW_MAX;
        if (yaw_input < -CHASSIS_MOUSE_YAW_MAX)
            yaw_input = -CHASSIS_MOUSE_YAW_MAX;

        // 平滑处理
        // rc_data_VT13->speed_yaw = (int16_t)(last_mouse_yaw * 0.7f + yaw_input * 0.3f);
        rc_data->speed_yaw = yaw_input;
        last_mouse_yaw = rc_data->speed_yaw;
        // 注意：根据实际电机方向可能需要调整符号
        int16_t vx = rc_data->speed_x;   // 左右速度
        int16_t vy = rc_data->speed_y;   // 前后速度
        int16_t vw = rc_data->speed_yaw; // 旋转速度

        chassis_speed[0] = -vx - vy - vw;
        chassis_speed[1] = -vx + vy - vw;
        chassis_speed[2] = vx - vy - vw;
        chassis_speed[3] = vx + vy - vw;
        break;
    }

    case 2: // 六足模式
    {
        // 根据是否按住Shift决定最大速度
        int16_t speed_max = (rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) ? CHASSIS_SPEED_MAX_BOOST : CHASSIS_SPEED_MAX;

        // ========== X轴目标速度计算（左右移动：A/D键） ==========
        int16_t target_vx = rc_axis_target(rc_ctrl.rc.ch[0],
                                           KEY_PRESSED_OFFSET_A,
                                           KEY_PRESSED_OFFSET_D, speed_max);

        // ========== Y轴目标速度计算（前后移动：W/S键） ==========
        int16_t target_vy = rc_axis_target(rc_ctrl.rc.ch[1],
                                           KEY_PRESSED_OFFSET_S,
                                           KEY_PRESSED_OFFSET_W, speed_max);

#if ENABLE_HEX_S_CURVE_ACCEL
        // 使用S曲线加减速平滑过渡
        rc_data->speed_x = hex_speed_ramp_s_curve(rc_data->speed_x, target_vx, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
        rc_data->speed_y = hex_speed_ramp_s_curve(rc_data->speed_y, target_vy, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
#else
        // 线性加减速（原始方式）
        if (target_vx > rc_data->speed_x)
        {
            rc_data->speed_x += CHASSIS_ACCEL_STEP;
            if (rc_data->speed_x > target_vx)
                rc_data->speed_x = target_vx;
        }
        else if (target_vx < rc_data->speed_x)
        {
            rc_data->speed_x -= CHASSIS_ACCEL_STEP;
            if (rc_data->speed_x < target_vx)
                rc_data->speed_x = target_vx;
        }

        if (target_vy > rc_data->speed_y)
        {
            rc_data->speed_y += CHASSIS_ACCEL_STEP;
            if (rc_data->speed_y > target_vy)
                rc_data->speed_y = target_vy;
        }
        else if (target_vy < rc_data->speed_y)
        {
            rc_data->speed_y -= CHASSIS_ACCEL_STEP;
            if (rc_data->speed_y < target_vy)
                rc_data->speed_y = target_vy;
        }
#endif

        // 额外限幅保护
        if (rc_data->speed_x > speed_max)
            rc_data->speed_x = speed_max;
        if (rc_data->speed_x < -speed_max)
            rc_data->speed_x = -speed_max;
        if (rc_data->speed_y > speed_max)
            rc_data->speed_y = speed_max;
        if (rc_data->speed_y < -speed_max)
            rc_data->speed_y = -speed_max;

        // ========== 转向控制（鼠标） ==========
        static int16_t last_mouse_yaw = 0;
        int16_t yaw_input = 0;

        // 优先使用鼠标控制，如果鼠标无输入则使用摇杆CH2
        if (abs(rc_ctrl.mouse.x) > 10) // 鼠标有输入
        {
            yaw_input = rc_ctrl.mouse.x * CHASSIS_MOUSE_YAW_SCALE;
        }
        else // 使用摇杆CH2
        {
            yaw_input = (int16_t)rc_ctrl.rc.ch[2] * 15; // 根据实际需要调整缩放系数
        }

        // 限幅
        if (yaw_input > CHASSIS_MOUSE_YAW_MAX)
            yaw_input = CHASSIS_MOUSE_YAW_MAX;
        if (yaw_input < -CHASSIS_MOUSE_YAW_MAX)
            yaw_input = -CHASSIS_MOUSE_YAW_MAX;

        // 平滑处理
        // rc_data_VT13->speed_yaw = (int16_t)(last_mouse_yaw * 0.7f + yaw_input * 0.3f);
        rc_data->speed_yaw = yaw_input;
        last_mouse_yaw = rc_data->speed_yaw;
        // ========== 麦克纳姆轮速度解算（修正公式） ==========
        // 标准麦克纳姆轮解算：
        // speed[0]: 左前  =  vx - vy + vw
        // speed[1]: 右前  =  vx + vy + vw
        // speed[2]: 左后  =  -vx + vy - vw
        // speed[3]: 右后  =  -vx - vy - vw
        // 注意：根据实际电机方向可能需要调整符号
        int16_t vx = rc_data->speed_x;   // 左右速度
        int16_t vy = rc_data->speed_y;   // 前后速度
        int16_t vw = rc_data->speed_yaw; // 旋转速度
        // 六轮
        hex_speed_set[0] = -vx - vy - vw;
        hex_speed_set[1] = -vx + vy - vw;
        hex_speed_set[2] = vx - vy - vw;
        hex_speed_set[3] = vx + vy - vw;
        // hex_speed_set[4] = (+ vy + vw)*RPM_SCALE_2006_TO_3508;//*2.284; //1.875
        // hex_speed_set[5] = (- vy + vw)*RPM_SCALE_2006_TO_3508;//*2.284; //1.875
        hex_speed_set[4] = +vy * RPM_SCALE_2006_TO_3508; //*2.284; //1.875
        hex_speed_set[5] = -vy * RPM_SCALE_2006_TO_3508; //*2.284; //1.875
        // if(hex_speed_set[4]>500)
        //     hex_speed_set[4] = 500;
        // else if(hex_speed_set[4]<-500)
        //     hex_speed_set[4] = -500;
        // if(hex_speed_set[5]>500)
        //     hex_speed_set[5] = 500;
        // else if(hex_speed_set[5]<-500)
        //     hex_speed_set[5] = -500;

        break;
    }

    default:
        break;
    }
}
// /**
//  * @brief 初始化进度条（首次绘制）
//  * @param initial_progress 初始进度值(0.0~1.0)
//  */
// void ProgressBar_Init(float initial_progress)
// {
//     u32 bar_width = (u32)(PROGRESS_BAR_WIDTH * initial_progress);

//     // 1. 绘制进度条矩形（绿色）
//     Rectangle_Draw(&progress_bar, "PB", UI_Graph_ADD, 0, UI_Color_Green, 3,
//                    PROGRESS_BAR_X, PROGRESS_BAR_Y,
//                    PROGRESS_BAR_X + bar_width, PROGRESS_BAR_Y + PROGRESS_BAR_HEIGHT);

//     // 2. 绘制进度百分比（浮点数，保留1位小数）
//     // 显示值 = progress * 100，例如 0.75 显示为 75.0
//     Float_Draw(&progress_text, "PT", UI_Graph_ADD, 1, UI_Color_White,
//                20, 2, PROGRESS_TEXT_X, PROGRESS_TEXT_Y,
//                initial_progress * 100.0f);

//     // 3. 一次性发送两个图形
//     UI_ReFresh(2, progress_bar, progress_text);

//     current_progress = initial_progress;
//     progress_initialized = 1;
// }

// /**
//  * @brief 更新进度条（动态修改）
//  * @param new_progress 新进度值(0.0~1.0)
//  */
// void ProgressBar_Update(float new_progress)
// {
//     // 限制进度范围
//     if (new_progress >=1.0f)
//         new_progress = 1.0f;
//     if (new_progress <= 0.0f)
//         new_progress = 0.0f;

//     // 如果进度没有变化，不需要更新
//     if (new_progress == current_progress)
//         return;

//     u32 bar_width = (u32)(PROGRESS_BAR_WIDTH * new_progress);

//     if (!progress_initialized)
//     {
//         ProgressBar_Init(new_progress);
//         return;
//     }

//     // 方案1：使用 MODIFY 操作（推荐，更高效）
//     // 修改矩形宽度
//     Rectangle_Draw(&progress_bar, "PB", UI_Graph_Change, 0, UI_Color_Green, 3,
//                    PROGRESS_BAR_X, PROGRESS_BAR_Y,
//                    PROGRESS_BAR_X + bar_width, PROGRESS_BAR_Y + PROGRESS_BAR_HEIGHT);

//     // 修改浮点数值
//     Float_Draw(&progress_text, "PT", UI_Graph_Change, 1, UI_Color_White,
//                20, 2, PROGRESS_TEXT_X, PROGRESS_TEXT_Y,
//                new_progress * 100.0f);

//     // 发送修改后的图形
//     UI_ReFresh(2, progress_bar, progress_text);

//     current_progress = new_progress;
// }

// /**
//  * @brief 删除进度条
//  */
// void ProgressBar_Delete(void)
// {
//     if (!progress_initialized)
//         return;

//     // 使用 MODIFY 操作，但将 operate_tpye 设为删除
//     Rectangle_Draw(&progress_bar, "PB", UI_Graph_Del, 0, UI_Color_Green, 3,
//                    0, 0, 0, 0);

//     Float_Draw(&progress_text, "PT", UI_Graph_Del, 1, UI_Color_White,
//                20, 2, 0, 0, 0.0f);

//     UI_ReFresh(2, progress_bar, progress_text);
//     progress_initialized = 0;
// }

// /**
//  * @brief 带颜色变化的进度条（根据进度改变颜色）
//  * @param new_progress 新进度值(0.0~1.0)
//  */
// void ProgressBar_Update_WithColor(float new_progress)
// {
//     if (new_progress >=1.0f)
//         new_progress = 1.0f;
//     if (new_progress <= 0.0f)
//         new_progress = 0.0f;

//     u32 bar_width = (u32)(PROGRESS_BAR_WIDTH * new_progress);
//     u32 color;

//     // 根据进度选择颜色
//     if (new_progress >= 0.7f)
//     {
//         color = UI_Color_Green; // ≥70% 绿色
//     }
//     else if (new_progress >= 0.3f)
//     {
//         color = UI_Color_Yellow; // 30%~70% 黄色
//     }
//     else
//     {
//         color = UI_Color_Purplish_red; //  30% 红色
//     }

//     Rectangle_Draw(&progress_bar, "PB", UI_Graph_Change, 0, color, 3,
//                    PROGRESS_BAR_X, PROGRESS_BAR_Y,
//                    PROGRESS_BAR_X + bar_width, PROGRESS_BAR_Y + PROGRESS_BAR_HEIGHT);

//     Float_Draw(&progress_text, "PT", UI_Graph_Change, 1, UI_Color_White,
//                20, 2, PROGRESS_TEXT_X, PROGRESS_TEXT_Y,
//                new_progress * 100.0f);

//     UI_ReFresh(2, progress_bar, progress_text);
//     current_progress = new_progress;
//     progress_initialized = 1;
// }

static void usart_printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);
    len = vsprintf((char *)tx_buf, fmt, ap);
    va_end(ap);
    HAL_UART_Transmit_DMA(huart, tx_buf, len);
}
