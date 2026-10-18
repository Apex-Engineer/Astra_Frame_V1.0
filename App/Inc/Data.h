#ifndef __DATA_H__
#define __DATA_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "custom_control.h"

#include "motor.h"
#include "referee.h"
#include "PID.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "VT13_rc_ctrl.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "stdarg.h"
#include "chassis_power_control.h"
#include "RM_Cilent_UI.h"
#include "hex_chassis_power_control.h"
#include "Chassis.h"

#define USE_VT13_REMOTE_CONTROL 0

typedef struct
{
    interaction_figure_t line_center; // 中心线
    interaction_figure_t rect_hp;     // 血量框
    interaction_figure_t circle_aim;  // 瞄准圈
    interaction_figure_t arc_charge;  // 充能指示器
    Float_Data float_pitch;           // 俯仰角显示
    String_Data text_status;          // 状态文本
    uint8_t init_done;                // 初始化标志
} UI_Engineer_t;

extern UI_Engineer_t ui_engineer;

void speed_calculation(VT13_data_rc_t *rc_data_VT13, uint8_t chassis_status, uint8_t gimbal_status, uint8_t ammo_status);
void rc_speed_calculation(data_rc_t *rc_data, uint8_t chassis_status, uint8_t gimbal_status, uint8_t ammo_status);
/** 底盘 PID 初始化，需在 main 中、启动 FreeRTOS 前调用 */
void Chassis_PID_Init(void);
void Data_Task(void const *argument);

#endif /* __DATA_H__ */
