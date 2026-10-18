#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/** 底盘状态枚举，与 VT13 遥控器约定一致 */
typedef enum
{
    //中间为静止，右边为四轮，左边为六轮
    // CHASSIS_STATUS_STOP = 0,   /** 静止 */
    // CHASSIS_STATUS_FOLLOW = 1, /** 跟随 */
    // CHASSIS_STATUS_SPIN = 2    /** 小陀螺 */
    //左边为静止，中间为四轮，右边为四轮
    CHASSIS_STATUS_FOLLOW = 0,   /** 静止 *///最左边
    CHASSIS_STATUS_SPIN = 1, /** 跟随 *///中间
    CHASSIS_STATUS_STOP = 2    /** 小陀螺 *///最右边
} chassis_status_e;

void chassis_control(uint8_t chassis_status);
void lift_attach(void);
void Chassis_Task(void const *argument);

#endif /* __CHASSIS_H__ */
