#include "Data.h"
#include "Chassis.h"
#include "PID.h"
#include "hex_chassis_power_control.h"
#include "cmsis_os.h"
#include <stdlib.h>

extern data_rc_t data_rc;
extern RC_ctrl_t rc_ctrl;
extern VT13_RC_ctrl_t VT13_rc_ctrl;
extern VT13_data_rc_t VT13_data_rc;

pid_type_def chassis_pid_vel[4];
pid_type_def hex_chassis_pid_vel[HEX_MOTOR_NUM];
int16_t chassis_speed[4] = {0};
int16_t hex_speed_set[HEX_MOTOR_NUM] = {0};

#define CHASSIS_SPEED_MAX 15000       // 最大速度
#define CHASSIS_SPEED_MAX_BOOST 15000 // 加速模式下最大速度
#define CHASSIS_ACCEL_STEP 2500       // 每次更新的速度增量
#define CHASSIS_DECEL_STEP 4000       // 减速度步进
#define CHASSIS_MOUSE_YAW_SCALE 15    // 鼠标转向缩放系数
#define CHASSIS_MOUSE_YAW_MAX 3000    // 鼠标转向最大速度
#define ENABLE_S_CURVE_ACCEL 0        // 1=启用S曲线加减速, 0=禁用
#define ENABLE_HEX_S_CURVE_ACCEL 0    // 1=启用六足S曲线加减速, 0=禁用

#define M3508_GEAR_RATIO (15.76f)
#define M2006_GEAR_RATIO (36.0f)
#define WHEEL_DIAMETER_3508 (0.154f) // 轮径，单位 m（154 mm）
#define WHEEL_DIAMETER_2006 (0.060f) // 轮径，单位 m（60 mm）

#define RPM_SCALE_2006_TO_3508 ((M2006_GEAR_RATIO / M3508_GEAR_RATIO) * (WHEEL_DIAMETER_3508 / WHEEL_DIAMETER_2006))

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
    PID_init(&hex_chassis_pid_vel[4], 7.0f, 0.2f, 0.002f, 4000.0f, 10000.0f);
    PID_init(&hex_chassis_pid_vel[5], 7.0f, 0.2f, 0.002f, 4000.0f, 10000.0f);
}

void Data_Task(void const *argument)
{
    (void)argument;
    while (1)
    {
#if USE_VT13_REMOTE_CONTROL
        VT13_data_rc.VT13_ctrl = get_VT13_remote_control_point();
        VT13_read_data(&VT13_data_rc);
        speed_calculation(&VT13_data_rc, VT13_data_rc.status_chassis, VT13_data_rc.status_gimbal,
                          VT13_data_rc.status_ammo);
        osDelay(8);
#else
        data_rc.RC_ctrl = get_remote_control_point();
        rc_read_data(&data_rc);
        rc_speed_calculation(&data_rc, data_rc.status_chassis, data_rc.status_gimbal, data_rc.status_ammo);
        osDelay(10);
#endif
        osDelay(2);
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
    switch (chassis_status)
    {
    case CHASSIS_STATUS_STOP: // 静止状态
        rc_data_VT13->speed_x = 0;
        rc_data_VT13->speed_y = 0;
        rc_data_VT13->speed_yaw = 0;
        break;

    case CHASSIS_STATUS_FOLLOW: // 跟随状态
    {
        int16_t speed_max = IF_KEY_PRESSED_SHIFT_VT13 ? CHASSIS_SPEED_MAX_BOOST : CHASSIS_SPEED_MAX;

        int16_t target_vx = 0;
        if (IF_KEY_PRESSED_A_VT13)
        {
            target_vx = -speed_max;
        }
        else if (IF_KEY_PRESSED_D_VT13)
        {
            target_vx = speed_max;
        }

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
        rc_data_VT13->speed_x =
            speed_ramp_s_curve(rc_data_VT13->speed_x, target_vx, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
        rc_data_VT13->speed_y =
            speed_ramp_s_curve(rc_data_VT13->speed_y, target_vy, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
#else
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

        if (rc_data_VT13->speed_x > speed_max)
            rc_data_VT13->speed_x = speed_max;
        if (rc_data_VT13->speed_x < -speed_max)
            rc_data_VT13->speed_x = -speed_max;
        if (rc_data_VT13->speed_y > speed_max)
            rc_data_VT13->speed_y = speed_max;
        if (rc_data_VT13->speed_y < -speed_max)
            rc_data_VT13->speed_y = -speed_max;

        int16_t yaw_input = 0;

        if (abs(VT13_rc_ctrl.mouse.x) > 10) // 鼠标有输入
        {
            yaw_input = VT13_rc_ctrl.mouse.x * CHASSIS_MOUSE_YAW_SCALE;
        }
        else // 使用摇杆CH3
        {
            yaw_input = (int16_t)VT13_rc_ctrl.rc.ch[3] * 15; // 根据实际需要调整缩放系数
        }

        if (yaw_input > CHASSIS_MOUSE_YAW_MAX)
            yaw_input = CHASSIS_MOUSE_YAW_MAX;
        if (yaw_input < -CHASSIS_MOUSE_YAW_MAX)
            yaw_input = -CHASSIS_MOUSE_YAW_MAX;

        rc_data_VT13->speed_yaw = yaw_input;

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

    {
        int16_t speed_max = IF_KEY_PRESSED_SHIFT_VT13 ? CHASSIS_SPEED_MAX_BOOST : CHASSIS_SPEED_MAX;

        int16_t target_vx = 0;
        if (IF_KEY_PRESSED_A_VT13)
        {
            target_vx = -speed_max;
        }
        else if (IF_KEY_PRESSED_D_VT13)
        {
            target_vx = speed_max;
        }

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
        rc_data_VT13->speed_x =
            speed_ramp_s_curve(rc_data_VT13->speed_x, target_vx, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
        rc_data_VT13->speed_y =
            speed_ramp_s_curve(rc_data_VT13->speed_y, target_vy, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
#else
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

        if (rc_data_VT13->speed_x > speed_max)
            rc_data_VT13->speed_x = speed_max;
        if (rc_data_VT13->speed_x < -speed_max)
            rc_data_VT13->speed_x = -speed_max;
        if (rc_data_VT13->speed_y > speed_max)
            rc_data_VT13->speed_y = speed_max;
        if (rc_data_VT13->speed_y < -speed_max)
            rc_data_VT13->speed_y = -speed_max;

        int16_t yaw_input = 0;

        if (abs(VT13_rc_ctrl.mouse.x) > 10) // 鼠标有输入
        {
            yaw_input = VT13_rc_ctrl.mouse.x * CHASSIS_MOUSE_YAW_SCALE;
        }
        else // 使用摇杆CH3
        {
            yaw_input = (int16_t)VT13_rc_ctrl.rc.ch[3] * 15; // 根据实际需要调整缩放系数
        }

        if (yaw_input > CHASSIS_MOUSE_YAW_MAX)
            yaw_input = CHASSIS_MOUSE_YAW_MAX;
        if (yaw_input < -CHASSIS_MOUSE_YAW_MAX)
            yaw_input = -CHASSIS_MOUSE_YAW_MAX;

        rc_data_VT13->speed_yaw = yaw_input;

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

static int16_t rc_axis_target(int16_t joystick, uint16_t negative_key, uint16_t positive_key, int16_t speed_max)
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
    switch (chassis_status)
    {
    case 0: // 静止状态
        rc_data->speed_x = 0;
        rc_data->speed_y = 0;
        rc_data->speed_yaw = 0;
        break;

    case 1: // 跟随状态
    {
        int16_t speed_max = (rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) ? CHASSIS_SPEED_MAX_BOOST : CHASSIS_SPEED_MAX;

        int16_t target_vx = -rc_axis_target(rc_ctrl.rc.ch[0], KEY_PRESSED_OFFSET_A, KEY_PRESSED_OFFSET_D, speed_max);

        int16_t target_vy = rc_axis_target(rc_ctrl.rc.ch[1], KEY_PRESSED_OFFSET_S, KEY_PRESSED_OFFSET_W, speed_max);

#if ENABLE_S_CURVE_ACCEL
        rc_data->speed_x =
            speed_ramp_s_curve(rc_data->speed_x, target_vx, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
        rc_data->speed_y =
            speed_ramp_s_curve(rc_data->speed_y, target_vy, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
#else
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

        if (rc_data->speed_x > speed_max)
            rc_data->speed_x = speed_max;
        if (rc_data->speed_x < -speed_max)
            rc_data->speed_x = -speed_max;
        if (rc_data->speed_y > speed_max)
            rc_data->speed_y = speed_max;
        if (rc_data->speed_y < -speed_max)
            rc_data->speed_y = -speed_max;

        int16_t yaw_input = 0;

        if (abs(rc_ctrl.mouse.x) > 10) // 鼠标有输入
        {
            yaw_input = rc_ctrl.mouse.x * CHASSIS_MOUSE_YAW_SCALE;
        }
        else // 使用摇杆CH3
        {
            yaw_input = (int16_t)rc_ctrl.rc.ch[2] * 15; // 根据实际需要调整缩放系数
        }

        if (yaw_input > CHASSIS_MOUSE_YAW_MAX)
            yaw_input = CHASSIS_MOUSE_YAW_MAX;
        if (yaw_input < -CHASSIS_MOUSE_YAW_MAX)
            yaw_input = -CHASSIS_MOUSE_YAW_MAX;

        rc_data->speed_yaw = yaw_input;
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
        int16_t speed_max = (rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) ? CHASSIS_SPEED_MAX_BOOST : CHASSIS_SPEED_MAX;

        int16_t target_vx = rc_axis_target(rc_ctrl.rc.ch[0], KEY_PRESSED_OFFSET_A, KEY_PRESSED_OFFSET_D, speed_max);

        int16_t target_vy = rc_axis_target(rc_ctrl.rc.ch[1], KEY_PRESSED_OFFSET_S, KEY_PRESSED_OFFSET_W, speed_max);

#if ENABLE_HEX_S_CURVE_ACCEL
        rc_data->speed_x =
            hex_speed_ramp_s_curve(rc_data->speed_x, target_vx, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
        rc_data->speed_y =
            hex_speed_ramp_s_curve(rc_data->speed_y, target_vy, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP, speed_max);
#else
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

        if (rc_data->speed_x > speed_max)
            rc_data->speed_x = speed_max;
        if (rc_data->speed_x < -speed_max)
            rc_data->speed_x = -speed_max;
        if (rc_data->speed_y > speed_max)
            rc_data->speed_y = speed_max;
        if (rc_data->speed_y < -speed_max)
            rc_data->speed_y = -speed_max;

        int16_t yaw_input = 0;

        if (abs(rc_ctrl.mouse.x) > 10) // 鼠标有输入
        {
            yaw_input = rc_ctrl.mouse.x * CHASSIS_MOUSE_YAW_SCALE;
        }
        else // 使用摇杆CH2
        {
            yaw_input = (int16_t)rc_ctrl.rc.ch[2] * 15; // 根据实际需要调整缩放系数
        }

        if (yaw_input > CHASSIS_MOUSE_YAW_MAX)
            yaw_input = CHASSIS_MOUSE_YAW_MAX;
        if (yaw_input < -CHASSIS_MOUSE_YAW_MAX)
            yaw_input = -CHASSIS_MOUSE_YAW_MAX;

        rc_data->speed_yaw = yaw_input;
        int16_t vx = rc_data->speed_x;   // 左右速度
        int16_t vy = rc_data->speed_y;   // 前后速度
        int16_t vw = rc_data->speed_yaw; // 旋转速度
        hex_speed_set[0] = -vx - vy - vw;
        hex_speed_set[1] = -vx + vy - vw;
        hex_speed_set[2] = vx - vy - vw;
        hex_speed_set[3] = vx + vy - vw;
        hex_speed_set[4] = +vy * RPM_SCALE_2006_TO_3508; //*2.284; //1.875
        hex_speed_set[5] = -vy * RPM_SCALE_2006_TO_3508; //*2.284; //1.875

        break;
    }

    default:
        break;
    }
}
