#ifndef VT13_RC_CTRL_H
#define VT13_RC_CTRL_H

#include "struct_typedef.h"
#include "remote_control.h"

#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define VT13_HEADER_SIZE sizeof(VT13_RC_ctrl_t)

#define PIT_MOTOR_MAX 2.0f
#define PIT_MOTOR_MIN 0.3f

/* 检测键盘按键状态 
   若对应按键被按下，则逻辑表达式的值为1，否则为0 */
#define IF_KEY_PRESSED_VT13 (VT13_rc_ctrl.key.v)
#define IF_KEY_PRESSED_W_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_W) != 0)
#define IF_KEY_PRESSED_S_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_S) != 0)
#define IF_KEY_PRESSED_A_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_A) != 0)
#define IF_KEY_PRESSED_D_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_D) != 0)
#define IF_KEY_PRESSED_Q_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q) != 0)
#define IF_KEY_PRESSED_E_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_E) != 0)
#define IF_KEY_PRESSED_G_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_G) != 0)
#define IF_KEY_PRESSED_X_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_X) != 0)
#define IF_KEY_PRESSED_Z_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z) != 0)
#define IF_KEY_PRESSED_C_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_C) != 0)
#define IF_KEY_PRESSED_B_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_B) != 0)
#define IF_KEY_PRESSED_V_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_V) != 0)
#define IF_KEY_PRESSED_F_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_F) != 0)
#define IF_KEY_PRESSED_R_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_R) != 0)
#define IF_KEY_PRESSED_CTRL_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL) != 0)
#define IF_KEY_PRESSED_SHIFT_VT13 ((VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0)

/* 检测鼠标按键状态 
   按下为1，没按下为0*/
#define IF_MOUSE_PRESSED_LEFT_VT13 (VT13_rc_ctrl.mouse.press_l == 1)
#define IF_MOUSE_PRESSED_RIGH_VT13 (VT13_rc_ctrl.mouse.press_r == 1)
#define IF_MOUSE_PRESSED_MID_VT13 (VT13_rc_ctrl.mouse.middle == 1)

/* 获取鼠标三轴的移动速度 */
#define MOUSE_X_MOVE_SPEED_VT13 (VT13_rc_ctrl.mouse.x)
#define MOUSE_Y_MOVE_SPEED_VT13 (VT13_rc_ctrl.mouse.y)
#define MOUSE_Z_MOVE_SPEED_VT13 (VT13_rc_ctrl.mouse.z)

typedef struct __attribute__((packed))
{
    struct __attribute__((packed))
    {
        int16_t ch[4];
        uint8_t mode_sw;
        uint8_t stop;
        uint8_t left_button; //fn;
        uint8_t right_button;
        int16_t wheel;
        uint8_t shutter;
    } rc;
    struct __attribute__((packed))
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
        uint8_t middle;
    } mouse;
    struct __attribute__((packed))
    {
        uint16_t v;
    } key;
    uint16_t crc16;

} VT13_RC_ctrl_t;
/**
  * @brief                  遥控器获取的数据信息
  * @param  VT13_ctrl       VT13遥控器原始数据
  * @param  speed_x         解算后左右运动速度
  * @param  speed_y         解算后前后运动速度
  * @param  speed_yaw       解算后yaw运动速度
  * @param  angle_pitch     解算后的pitch角度
  * @param  angle_yaw       解算后的yaw角度
  * @param  status_chassis  底盘状态 0为静止 1为跟随 2为小陀螺
  * @param  status_gimbal   云台状态 0为静止 1为常规
  * @param  status_shoot    发射状态 0为静止 1为开摩擦轮 2为开拨盘       
  * @param  reset           达妙手动使能           
  */
typedef struct
{
    const VT13_RC_ctrl_t *VT13_ctrl;
    int16_t speed_x;
    int16_t speed_y;
    int16_t speed_yaw;
    float angle_pitch;
    float angle_yaw;
    uint8_t status_chassis;
    uint8_t status_gimbal;
    uint8_t status_ammo;
    uint8_t reset;
} VT13_data_rc_t;

void VT13_read_data(VT13_data_rc_t *VT13_data_rc);

extern VT13_RC_ctrl_t VT13_rc_ctrl;
extern const VT13_RC_ctrl_t *get_VT13_remote_control_point(void);

void VT13_to_rc(uint8_t *VT13_buf, VT13_RC_ctrl_t *rc_ctrl);

#endif
