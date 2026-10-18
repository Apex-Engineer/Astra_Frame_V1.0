#include "Chassis.h"
#include "Data.h"
#include "chassis_power_control.h"
#include "hex_chassis_power_control.h"
#include "can.h"

#define USE_CHASSIS_POWER_CONTROL 0

extern data_rc_t data_rc;
extern VT13_RC_ctrl_t VT13_rc_ctrl;
extern VT13_data_rc_t VT13_data_rc;
extern data_motor_t data_motor;
extern data_referee_t data_referee;

extern pid_type_def chassis_pid_vel[4];
extern pid_type_def hex_chassis_pid_vel[HEX_MOTOR_NUM];
extern int16_t chassis_speed[4];
extern int16_t hex_speed_set[HEX_MOTOR_NUM];
pid_type_def lift_attach_pid_pos[4];
pid_type_def lift_attach_pid_vel[4];

float angle2 = 0.0f; // PITCH1

void Chassis_Task(void const *argument)
{
#if USE_CHASSIS_POWER_CONTROL
    chassis_power_control_init();     // 四轮功率初始化
    //hex_chassis_power_control_init(); // 六轮功率初始化
#endif
    while (1)
    {
#if USE_VT13_REMOTE_CONTROL
        chassis_control(VT13_data_rc.status_chassis);
#else
        chassis_control(data_rc.status_chassis);
#endif
        osDelay(10);
        // lift_attach();
        // osDelay(5);
    }
}

/**
 * @brief 底盘控制函数（带功率控制）
 * @param chassis_status 底盘状态
 */
void chassis_control(uint8_t chassis_status)
{
    uint8_t i;

    switch (chassis_status)
    {
#if USE_VT13_REMOTE_CONTROL
    case CHASSIS_STATUS_STOP:
        DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID, 0, 0, 0, 0);
        DJ_124_ctrl(&hcan1, AMMO_ALL_ID, 0, 0, 0, 0);
        for (i = 0; i < 4; i++)
        {
            chassis_pid_vel[i].iout = 0;
        }
        break;

    case CHASSIS_STATUS_FOLLOW:
#if USE_CHASSIS_POWER_CONTROL
        /* 功率控制内部会完成PID计算: 先用原始目标算一次获取线性化参数,
         * 再用衰减后的目标重算PID作为最终输出 */
        chassis_power_control_process(chassis_pid_vel, data_motor.Chassis_motor_measure, chassis_speed);
        DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID,
                    (int16_t)chassis_pid_vel[0].out,
                    (int16_t)chassis_pid_vel[1].out,
                    (int16_t)chassis_pid_vel[2].out,
                    (int16_t)chassis_pid_vel[3].out);
        break;
#else
        DJ_124_ctrl_vel(&hcan1, CHASSIS_ALL_ID, chassis_pid_vel, data_motor.Chassis_motor_measure,
                        chassis_speed[0],
                        chassis_speed[1],
                        chassis_speed[2],
                        chassis_speed[3]);
        break;
#endif
    case CHASSIS_STATUS_SPIN:
// #if USE_CHASSIS_POWER_CONTROL
//         // 六轮上台阶，起伏路段控制
//         for (i = 0; i < 6; i++)
//         {
//             hex_chassis_pid_vel[i].out = PID_Calc(&hex_chassis_pid_vel[i],
//                                                   data_motor.Hex_motor_measure[i].speed_rpm,
//                                                   hex_speed_set[i], 0);
//         }
//         for (i = 0; i < 6; i++)
//         {
//             // 速度接近0时限制最大电流（防止起步冲击）
//             if (abs(data_motor.Hex_motor_measure[i].speed_rpm) < 500)
//             {
//                 if (hex_chassis_pid_vel[i].out > 3000)
//                     hex_chassis_pid_vel[i].out = 3000;
//                 if (hex_chassis_pid_vel[i].out < -3000)
//                     hex_chassis_pid_vel[i].out = -3000;
//             }
//         }
//         hex_chassis_power_control_process(hex_chassis_pid_vel, data_motor.Hex_motor_measure, hex_speed_set);
//         DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID,
//                     (int16_t)hex_chassis_pid_vel[0].out,
//                     (int16_t)hex_chassis_pid_vel[1].out,
//                     (int16_t)hex_chassis_pid_vel[2].out,
//                     (int16_t)hex_chassis_pid_vel[3].out);
//         DJ_124_ctrl(&hcan1, AMMO_ALL_ID,
//                     (int16_t)hex_chassis_pid_vel[4].out,
//                     (int16_t)hex_chassis_pid_vel[5].out,
//                     0, 0);
//         break;
// #else
//         for (i = 0; i < 6; i++)
//         {
//             hex_chassis_pid_vel[i].out = PID_Calc(&hex_chassis_pid_vel[i],
//                                                   data_motor.Hex_motor_measure[i].speed_rpm,
//                                                   hex_speed_set[i], 0);
//         }
//         DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID,
//                     (int16_t)hex_chassis_pid_vel[0].out,
//                     (int16_t)hex_chassis_pid_vel[1].out,
//                     (int16_t)hex_chassis_pid_vel[2].out,
//                     (int16_t)hex_chassis_pid_vel[3].out);
//         DJ_124_ctrl(&hcan1, AMMO_ALL_ID,
//                     (int16_t)hex_chassis_pid_vel[4].out,
//                     (int16_t)hex_chassis_pid_vel[5].out,
//                     0, 0);
//         break;
// #endif
#if USE_CHASSIS_POWER_CONTROL
        chassis_power_control_process(chassis_pid_vel, data_motor.Chassis_motor_measure, chassis_speed);
        DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID,
                    (int16_t)chassis_pid_vel[0].out,
                    (int16_t)chassis_pid_vel[1].out,
                    (int16_t)chassis_pid_vel[2].out,
                    (int16_t)chassis_pid_vel[3].out);
        break;
#else
        DJ_124_ctrl_vel(&hcan1, CHASSIS_ALL_ID, chassis_pid_vel, data_motor.Chassis_motor_measure,
                        chassis_speed[0],
                        chassis_speed[1],
                        chassis_speed[2],
                        chassis_speed[3]);
        break;
#endif
    default:
        DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID, 0, 0, 0, 0);
        DJ_124_ctrl(&hcan1, AMMO_ALL_ID, 0, 0, 0, 0);
        break;

#else
    case 0:
        DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID, 0, 0, 0, 0);
        DJ_124_ctrl(&hcan1, AMMO_ALL_ID, 0, 0, 0, 0);
        for (i = 0; i < 4; i++)
        {
            chassis_pid_vel[i].iout = 0;
        }
        break;

    case 1:
#if USE_CHASSIS_POWER_CONTROL
        chassis_power_control_process(chassis_pid_vel, data_motor.Chassis_motor_measure, chassis_speed);
        DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID,
                    (int16_t)chassis_pid_vel[0].out,
                    (int16_t)chassis_pid_vel[1].out,
                    (int16_t)chassis_pid_vel[2].out,
                    (int16_t)chassis_pid_vel[3].out);
        DJ_124_ctrl(&hcan1, AMMO_ALL_ID, 0, 0, 0, 0);
        break;
#else
        DJ_124_ctrl_vel(&hcan1, CHASSIS_ALL_ID, chassis_pid_vel, data_motor.Chassis_motor_measure,
                        chassis_speed[0],
                        chassis_speed[1],
                        chassis_speed[2],
                        chassis_speed[3]);
        break;
#endif
    case 2:
#if USE_CHASSIS_POWER_CONTROL
        // 六轮上台阶，起伏路段控制
        for (i = 0; i < 6; i++)
        {
            hex_chassis_pid_vel[i].out = PID_Calc(&hex_chassis_pid_vel[i],
                                                  data_motor.Hex_motor_measure[i].speed_rpm,
                                                  hex_speed_set[i], 0);
        }
        for (i = 0; i < 6; i++)
        {
            // 速度接近0时限制最大电流（防止起步冲击）
            if (abs(data_motor.Hex_motor_measure[i].speed_rpm) < 500)
            {
                if (hex_chassis_pid_vel[i].out > 3000)
                    hex_chassis_pid_vel[i].out = 3000;
                if (hex_chassis_pid_vel[i].out < -3000)
                    hex_chassis_pid_vel[i].out = -3000;
            }
        }
        hex_chassis_power_control_process(hex_chassis_pid_vel, data_motor.Hex_motor_measure, hex_speed_set);
        DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID,
                    (int16_t)hex_chassis_pid_vel[0].out,
                    (int16_t)hex_chassis_pid_vel[1].out,
                    (int16_t)hex_chassis_pid_vel[2].out,
                    (int16_t)hex_chassis_pid_vel[3].out);
        DJ_124_ctrl(&hcan1, AMMO_ALL_ID,
                    (int16_t)hex_chassis_pid_vel[4].out,
                    (int16_t)hex_chassis_pid_vel[5].out,
                    0, 0);
        break;
#else
        // DJ_124_ctrl_vel(&hcan1, CHASSIS_ALL_ID, hex_chassis_pid_vel, data_motor.Hex_motor_measure,
        //                 hex_speed_set[0],
        //                 hex_speed_set[1],
        //                 hex_speed_set[2],
        //                 hex_speed_set[3]);
        // DJ_124_ctrl_vel(&hcan1, AMMO_ALL_ID, &hex_chassis_pid_vel[4], &data_motor.Hex_motor_measure[4],
        //                 hex_speed_set[4],
        //                 hex_speed_set[5], 0, 0);
        for (i = 0; i < 6; i++)
        {
            hex_chassis_pid_vel[i].out = PID_Calc(&hex_chassis_pid_vel[i],
                                                  data_motor.Hex_motor_measure[i].speed_rpm,
                                                  hex_speed_set[i], 0);
        }
        DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID,
                    (int16_t)hex_chassis_pid_vel[0].out,
                    (int16_t)hex_chassis_pid_vel[1].out,
                    (int16_t)hex_chassis_pid_vel[2].out,
                    (int16_t)hex_chassis_pid_vel[3].out);
        DJ_124_ctrl(&hcan1, AMMO_ALL_ID,
                    (int16_t)hex_chassis_pid_vel[4].out,
                    (int16_t)hex_chassis_pid_vel[5].out,
                    0, 0);
        break;
#endif
    default:
        DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID, 0, 0, 0, 0);
        DJ_124_ctrl(&hcan1, AMMO_ALL_ID, 0, 0, 0, 0);
        break;

#endif
    }
}
/**
 * @brief PITCH1
 */
// void lift_attach()
// {
//     // float angle2 =0.0f;
//     // angle2 = data_motor.arm_motor_measure[6].pos;
//     // if (VT13_data_rc.status_chassis == CHASSIS_STATUS_FOLLOW)
//     // {
//     //     angle2 = data_motor.arm_motor_measure[6].pos;
//     //     // 抬升
//     //     if (IF_KEY_PRESSED_Z_VT13)
//     //     {
//     //         angle2 += 1;
//     //         DM_motor_MIT_ctrl(&hcan2, 0x13, angle2, 0.0f, 125.0f, 10.0f, 0.0f);
//     //     }
//     //     else if (IF_KEY_PRESSED_X_VT13)
//     //     {
//     //         angle2 -= 1;
//     //         DM_motor_MIT_ctrl(&hcan2, 0x13, angle2, 0.0f, 125.0f, 10.0f, 0.0f);
//     //     }
//     //     else if (VT13_rc_ctrl.rc.wheel > 0)
//     //     {
//     //         angle2 += 5;
//     //         DM_motor_MIT_ctrl(&hcan2, 0x13, angle2, 0.0f, 125.0f, 10.0f, 0.0f);
//     //         osDelay(1);
//     //         //DM_motor_MIT_ctrl(&hcan2, 0x13, 0.0f, -1.0f, 0.0f, 0.0f, 1.0f);
//     //     }
//     //     else if (VT13_rc_ctrl.rc.wheel < 0)//
//     //     {
//     //         angle2 -= 5;
//     //         DM_motor_MIT_ctrl(&hcan2, 0x13, angle2, 0.0f, 125.0f, 10.0f, 0.0f);
//     //         osDelay(1);
//     //         //DM_motor_MIT_ctrl(&hcan2, 0x13, 0.0f, 1.0f, 0.0f, 0.0f, -1.0f);
//     //     }
//     //     DM_motor_MIT_ctrl(&hcan2, 0x13, angle2, 0.0f, 125.0f, 10.0f, 0.0f);
//     //     osDelay(1);
//     // }
//     // float angle3 = data_motor.Ammo_motor_measure[1].ecd;
//     // PID_init(&lift_attach_pid_pos[0], 100.5f, 0.00f, 0.50f, 4000.0f, 16384.0f);
//     // PID_init(&lift_attach_pid_vel[0], 25.5f, 0.20f, 0.60f, 4000.0f, 16384.0f);

//     // PID_init(&lift_attach_pid_pos[1], 40.5f, 0.00f, 0.50f, 2000.0f, 16384.0f);
//     // PID_init(&lift_attach_pid_vel[1], 15.5f, 0.10f, 0.25f, 2000.0f, 16384.0f);
//     // if (VT13_data_rc.status_chassis == CHASSIS_STATUS_FOLLOW)
//     // {
//     //     // 抬升
//     //     if (IF_KEY_PRESSED_Z_VT13)
//     //     {
//     //         angle2 += 10;
//     //         DJ_124_ctrl_pos(&hcan1, 0x1FF, lift_attach_pid_pos, lift_attach_pid_vel, data_motor.Ammo_motor_measure, angle2, angle3, 0, 0); // 抬升
//     //         // lock_angle=angle2;
//     //     }
//     //     else if (IF_KEY_PRESSED_X_VT13)
//     //     {
//     //         angle2 -= 10;
//     //         DJ_124_ctrl_pos(&hcan1, 0x1FF, lift_attach_pid_pos, lift_attach_pid_vel, data_motor.Ammo_motor_measure, angle2, angle3, 0, 0); // 下降
//     //         // lock_angle=angle2;
//     //     }
//     //     else if (VT13_rc_ctrl.rc.wheel > 0)
//     //     {
//     //         angle2 += 10;
//     //         DJ_124_ctrl_pos(&hcan1, 0x1FF, lift_attach_pid_pos, lift_attach_pid_vel, data_motor.Ammo_motor_measure, angle2, angle3, 0, 0); // 抬升
//     //     }
//     //     else if (VT13_rc_ctrl.rc.wheel < 0)
//     //     {
//     //         angle2 -= 10;
//     //         DJ_124_ctrl_pos(&hcan1, 0x1FF, lift_attach_pid_pos, lift_attach_pid_vel, data_motor.Ammo_motor_measure, angle2, angle3, 0, 0); // 下降
//     //     }
//     //     DJ_124_ctrl_pos(&hcan1, 0x1FF, lift_attach_pid_pos, lift_attach_pid_vel, data_motor.Ammo_motor_measure, angle2, angle3, 0, 0);
//     //     // if(IF_KEY_PRESSED_X_VT13){
//     //     //     DJ_124_ctrl_pos(&hcan1,0x1FF,lift_attach_pid_pos,lift_attach_pid_vel,data_motor.Ammo_motor_measure,lock_angle,0,0,0);
//     //     // }
//     //     // else{
//     //     //     DJ_124_ctrl_pos(&hcan1,0x1FF,lift_attach_pid_pos,lift_attach_pid_vel,data_motor.Ammo_motor_measure,lock_angle,0,0,0);
//     //     // }
//     //     // 前伸
//     //     if (IF_KEY_PRESSED_C_VT13)
//     //     {
//     //         angle3 += 30;
//     //         DJ_124_ctrl_pos(&hcan2, 0x1FF, lift_attach_pid_pos, lift_attach_pid_vel, data_motor.Ammo_motor_measure, angle2, angle3, 0, 0);
//     //     }
//     //     else if (IF_KEY_PRESSED_V_VT13)
//     //     {
//     //         angle3 -= 30;
//     //         DJ_124_ctrl_pos(&hcan2, 0x1FF, lift_attach_pid_pos, lift_attach_pid_vel, data_motor.Ammo_motor_measure, angle2, angle3, 0, 0);
//     //     }
//     //     DJ_124_ctrl_pos(&hcan2, 0x1FF, lift_attach_pid_pos, lift_attach_pid_vel, data_motor.Ammo_motor_measure, angle2, angle3, 0, 0);
//     // }
// }
