#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include <stdint.h>

typedef enum
{
    CHASSIS_STATUS_FOLLOW = 0,
    CHASSIS_STATUS_SPIN = 1,
    CHASSIS_STATUS_STOP = 2
} chassis_status_e;

void chassis_control(uint8_t chassis_status);
void Chassis_Task(void const *argument);

#endif
