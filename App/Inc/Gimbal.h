#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "custom_control.h"

#define PI 3.14159265358979323846

typedef enum
{
    // GIMBLE_STATUS_STOP = 0,   /** 静止 */
    // GIMBLE_STATUS_FOLLOW = 1, /** 跟随 */
    // GIMBLE_STATUS_SPIN = 2    /** 小陀螺 */
    GIMBLE_STATUS_FOLLOW = 0,   /** 四轮 */
    GIMBLE_STATUS_SPIN = 1, /** 六轮 */
    GIMBLE_STATUS_STOP = 2    /** 静止 */
} gimble_status_e;


#endif /* __GIMBAL_H__ */