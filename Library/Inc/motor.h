#ifndef MOTOR_H
#define MOTOR_H
#include "main.h"
#include "PID.h"
#include "bsp_can.h"

/*电机ID*/
#define CHASSIS_ALL_ID 0x200
#define CAN_chassis_M1_ID 0x201
#define CAN_chassis_M2_ID 0x202
#define CAN_chassis_M3_ID 0x203
#define CAN_chassis_M4_ID 0x204

#define CAN_YAW_MOTOR_T_ID 0x02
#define CAN_PIT_MOTOR_T_ID 0x01

#define CAN_YAW_MOTOR_ID 0x12
#define CAN_PIT_MOTOR_ID 0x11

#define AMMO_ALL_ID 0x1FF
#define CAN_TRIGGER_MOTOR_ID 0x205
#define CAN_AMMO1_MOTOR_ID 0x206
#define CAN_AMMO2_MOTOR_ID 0x207
/*电机ID*/

/*电机参数最值*/
#define DM_P_MIN -12.5f
#define DM_P_MAX 12.5f
#define DM_V_MIN -30.0f
#define DM_V_MAX 30.0f
#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f
#define DM_KD_MAX 5.0f
#define DM_T_MIN -10.0f
#define DM_T_MAX 10.0f

#define DM_P_MIN_3510 -1250.0f
#define DM_P_MAX_3510 1250.0f
#define DM_V_MIN_3510 -280.0f
#define DM_V_MAX_3510 280.0f
#define DM_T_MIN_3510 -1.0f
#define DM_T_MAX_3510 1.0f

/*电机参数最值*/

/**
  * @brief                  适用M3508、M2006、GM6020、DM3519
  * @param  ecd             转子编码器角度
  * @param  speed_rpm       当前转速
  * @param  given_current   实际电流
  * @param  temperate       转子温度
  * @param  last_ecd        上一帧编码器角度
  * @param  last_speed      上一帧转速
  */
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    int16_t last_speed;
} motor_measure_DJ_t;

/**
  * @brief          适用DM4310、DM4340、DM8009、DM3507
  * @param id       电机ID
  * @param err      电机报错类型
  * @param pos      电机角度
  * @param vel      电机速度
  * @param tor      电机扭矩
  * @param T_mos    电机驱动温度
  * @param T_Rotor  电机线圈温度
  */
typedef struct
{
    int id;
    int err;
    float pos;
    float vel;
    float tor;
    float T_mos;
    float T_Rotor;
    int p_int;
    int v_int;
    int t_int;
} motor_measure_DM_t;

/**
  * @brief                          电机报文结构体
  * @param Chassis_motor_measure    底盘电机报文
  * @param Pit_motor_measure        PIT电机报文
  * @param Yaw_motor_measure        Yaw电机报文
  * @param Ammo_motor_measure       发射电机报文
  * @param custom_motor_measure     自定义控制器报文
  * @param arm_motor_measure        机械臂电机报文
  */
typedef struct
{
    motor_measure_DJ_t Chassis_motor_measure[4];
    motor_measure_DJ_t Hex_motor_measure[6];
    motor_measure_DM_t Pit_motor_measure;
    motor_measure_DM_t Yaw_motor_measure;
    motor_measure_DM_t custom_motor_measure[5];
    motor_measure_DM_t arm_motor_measure[7];
    motor_measure_DJ_t Ammo_motor_measure[4];
} data_motor_t;

int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
void DJ_motor_fbdata(motor_measure_DJ_t *motor, uint8_t *rx_data);
void DJ_124_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t motor1, uint16_t motor2, uint16_t motor3,
                 uint16_t motor4);
void DJ_124_ctrl_vel(CAN_HandleTypeDef *hcan, uint16_t motor_id, pid_type_def pid_vel[4],
                     motor_measure_DJ_t motor_measure[4], int16_t vel1, int16_t vel2, int16_t vel3, int16_t vel4);
void DJ_124_ctrl_pos(CAN_HandleTypeDef *hcan, uint16_t motor_id, pid_type_def pid_pos[4], pid_type_def pid_vel[4],
                     motor_measure_DJ_t motor_measure[4], int16_t pos1, int16_t pos2, int16_t pos3, int16_t pos4);
void DM_motor_fbdata(motor_measure_DM_t *motor, uint8_t *rx_data);
void DM_custom_motor_fbdata(motor_measure_DM_t *motor, uint8_t *rx_data);
void DM_motor_enable(CAN_HandleTypeDef *hcan, uint16_t motor_id);
void DM_motor_disable(CAN_HandleTypeDef *hcan, uint16_t motor_id);
void DM_motor_zeroset(CAN_HandleTypeDef *hcan, uint16_t motor_id);
void DM_motor_MIT_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd,
                       float torq);
void DM_custom_motor_MIT_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd,
                              float torq);
void DM_motor_posvel_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel);

#endif
