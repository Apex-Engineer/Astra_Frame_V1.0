#include "Gimbal.h"
#include "custom_control.h"

void Gimbal_Task(void const *argument)
{
    (void)argument;
    arm_control_task();
}
