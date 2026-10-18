#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "main.h"
#include "usart.h"

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
#define CHASSIS_OPEN_RC_SCALE 10 // 遥控器摇杆平移速度缩放系数
#define RC_SW_UP ((uint16_t)2)
#define RC_SW_MID ((uint16_t)1)
#define RC_SW_DOWN ((uint16_t)0)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP) //遥控器判断
/*键盘映射*/
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)
/*键盘映射*/

typedef struct __attribute__((packed))
{
    struct __attribute__((packed))
    {
        int16_t ch[5];
        char s[2];
    } rc;
    struct __attribute__((packed))
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    struct __attribute__((packed))
    {
        uint16_t v;
    } key;

} RC_ctrl_t;

/**
  * @brief                  遥控器获取的数据信息
  * @param  RC_ctrl         遥控器原始数据
  * @param  speed_x         解算后左右运动速度
  * @param  speed_y         解算后前后运动速度
  * @param  angle_pitch     解算后的pitch角度
  * @param  angle_yaw       解算后的yaw角度
  * @param  status_chassis  底盘状态 0为静止 1为跟随 2为小陀螺
  * @param  status_gimbal   云台状态 0为静止 1为常规
  * @param  status_shoot    发射状态 0为静止 1为开摩擦轮 2为开拨盘       
  * @param  reset           达妙手动使能           
  */
typedef struct
{
    const RC_ctrl_t *RC_ctrl; //修改
    int16_t speed_x;
    int16_t speed_y;
    int16_t speed_yaw;
    float angle_pitch;
    float angle_yaw;
    float pitch_offset_set;
    float yaw_offset_set;
    uint8_t status_chassis;
    uint8_t status_gimbal;
    uint8_t status_ammo;
    uint8_t reset;
} data_rc_t;

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void remote_control_init(void);
void remote_control_uart1_idle_handler(void);
const RC_ctrl_t *get_remote_control_point(void);
void rc_read_data(data_rc_t *data_rc);

#endif
