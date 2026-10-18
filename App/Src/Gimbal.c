#include "Gimbal.h"
#include "Data.h"

extern data_motor_t data_motor;
extern VT13_RC_ctrl_t VT13_rc_ctrl;
extern VT13_data_rc_t VT13_data_rc;

int c = 0;

void Gimbal_Task(void const *argument)
{
    (void)argument;
    arm_control_task();
}
