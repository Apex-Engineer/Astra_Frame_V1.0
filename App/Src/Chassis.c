#include "Chassis.h"
#include "Data.h"
#include "chassis_power_control.h"
#include "hex_chassis_power_control.h"
#include "can.h"
#include <stdlib.h>

#define USE_CHASSIS_POWER_CONTROL 0

extern data_rc_t data_rc;
extern VT13_data_rc_t VT13_data_rc;
extern data_motor_t data_motor;
extern pid_type_def chassis_pid_vel[4];
extern pid_type_def hex_chassis_pid_vel[HEX_MOTOR_NUM];
extern int16_t chassis_speed[4];
extern int16_t hex_speed_set[HEX_MOTOR_NUM];

void Chassis_Task(void const *argument)
{
    (void)argument;
#if USE_CHASSIS_POWER_CONTROL
    chassis_power_control_init(); // 四轮功率初始化
#endif
    while (1)
    {
#if USE_VT13_REMOTE_CONTROL
        chassis_control(VT13_data_rc.status_chassis);
#else
        chassis_control(data_rc.status_chassis);
#endif
        osDelay(10);
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
        DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID, (int16_t)chassis_pid_vel[0].out, (int16_t)chassis_pid_vel[1].out,
                    (int16_t)chassis_pid_vel[2].out, (int16_t)chassis_pid_vel[3].out);
        break;
#else
        DJ_124_ctrl_vel(&hcan1, CHASSIS_ALL_ID, chassis_pid_vel, data_motor.Chassis_motor_measure, chassis_speed[0],
                        chassis_speed[1], chassis_speed[2], chassis_speed[3]);
        break;
#endif
    case CHASSIS_STATUS_SPIN:
#if USE_CHASSIS_POWER_CONTROL
        chassis_power_control_process(chassis_pid_vel, data_motor.Chassis_motor_measure, chassis_speed);
        DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID, (int16_t)chassis_pid_vel[0].out, (int16_t)chassis_pid_vel[1].out,
                    (int16_t)chassis_pid_vel[2].out, (int16_t)chassis_pid_vel[3].out);
        break;
#else
        DJ_124_ctrl_vel(&hcan1, CHASSIS_ALL_ID, chassis_pid_vel, data_motor.Chassis_motor_measure, chassis_speed[0],
                        chassis_speed[1], chassis_speed[2], chassis_speed[3]);
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
        DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID, (int16_t)chassis_pid_vel[0].out, (int16_t)chassis_pid_vel[1].out,
                    (int16_t)chassis_pid_vel[2].out, (int16_t)chassis_pid_vel[3].out);
        DJ_124_ctrl(&hcan1, AMMO_ALL_ID, 0, 0, 0, 0);
        break;
#else
        DJ_124_ctrl_vel(&hcan1, CHASSIS_ALL_ID, chassis_pid_vel, data_motor.Chassis_motor_measure, chassis_speed[0],
                        chassis_speed[1], chassis_speed[2], chassis_speed[3]);
        break;
#endif
    case 2:
#if USE_CHASSIS_POWER_CONTROL
        for (i = 0; i < 6; i++)
        {
            hex_chassis_pid_vel[i].out =
                PID_Calc(&hex_chassis_pid_vel[i], data_motor.Hex_motor_measure[i].speed_rpm, hex_speed_set[i], 0);
        }
        for (i = 0; i < 6; i++)
        {
            if (abs(data_motor.Hex_motor_measure[i].speed_rpm) < 500)
            {
                if (hex_chassis_pid_vel[i].out > 3000)
                    hex_chassis_pid_vel[i].out = 3000;
                if (hex_chassis_pid_vel[i].out < -3000)
                    hex_chassis_pid_vel[i].out = -3000;
            }
        }
        hex_chassis_power_control_process(hex_chassis_pid_vel, data_motor.Hex_motor_measure, hex_speed_set);
        DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID, (int16_t)hex_chassis_pid_vel[0].out, (int16_t)hex_chassis_pid_vel[1].out,
                    (int16_t)hex_chassis_pid_vel[2].out, (int16_t)hex_chassis_pid_vel[3].out);
        DJ_124_ctrl(&hcan1, AMMO_ALL_ID, (int16_t)hex_chassis_pid_vel[4].out, (int16_t)hex_chassis_pid_vel[5].out, 0,
                    0);
        break;
#else
        for (i = 0; i < 6; i++)
        {
            hex_chassis_pid_vel[i].out =
                PID_Calc(&hex_chassis_pid_vel[i], data_motor.Hex_motor_measure[i].speed_rpm, hex_speed_set[i], 0);
        }
        DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID, (int16_t)hex_chassis_pid_vel[0].out, (int16_t)hex_chassis_pid_vel[1].out,
                    (int16_t)hex_chassis_pid_vel[2].out, (int16_t)hex_chassis_pid_vel[3].out);
        DJ_124_ctrl(&hcan1, AMMO_ALL_ID, (int16_t)hex_chassis_pid_vel[4].out, (int16_t)hex_chassis_pid_vel[5].out, 0,
                    0);
        break;
#endif
    default:
        DJ_124_ctrl(&hcan1, CHASSIS_ALL_ID, 0, 0, 0, 0);
        DJ_124_ctrl(&hcan1, AMMO_ALL_ID, 0, 0, 0, 0);
        break;

#endif
    }
}
