#ifndef APP_DATA_H
#define APP_DATA_H

#include "VT13_rc_ctrl.h"
#include "remote_control.h"

#define USE_VT13_REMOTE_CONTROL 0

void speed_calculation(VT13_data_rc_t *rc_data, uint8_t chassis_status, uint8_t gimbal_status, uint8_t ammo_status);
void rc_speed_calculation(data_rc_t *rc_data, uint8_t chassis_status, uint8_t gimbal_status, uint8_t ammo_status);

/* Initialize chassis PID controllers before starting the scheduler. */
void Chassis_PID_Init(void);
void Data_Task(void const *argument);

#endif
