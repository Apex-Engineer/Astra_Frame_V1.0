#include "motor.h"

data_motor_t data_motor;

/*float转换至uint*/
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
/*float转换至uint*/

/*uint转换至float*/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
/*uint转换至float*/

/**
 * @brief          can发数据
 * @param hcan     can位（&hcan1/&hcan2）
 * @param id       can id
 * @param data     发送数据
 * @param len      数据长度
 * @return         none
 */
uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.StdId = id;
    tx_header.ExtId = 0;
    tx_header.IDE = 0;
    tx_header.RTR = 0;
    tx_header.DLC = len;
    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief          DJ电机数据解包
 * @param motor    单位电机地址
 * @param rx_data  接收数据
 * @return         none
 */
void DJ_motor_fbdata(motor_measure_DJ_t *motor, uint8_t *rx_data)
{
    motor->last_ecd = motor->ecd;
    motor->last_speed = motor->speed_rpm;
    motor->ecd = (uint16_t)((rx_data)[0] << 8 | (rx_data)[1]);
    motor->speed_rpm = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]);
    motor->given_current = (uint16_t)((rx_data)[4] << 8 | (rx_data)[5]);
    motor->temperate = (rx_data)[6];
}

/**
 * @brief          3508/2006/6020电流开环
 * @param hcan     &hcan1 / &hcan2
 * @param motor_id 发送id
 * @param motor1   电机1电流
 * @param motor2   电机2电流
 * @param motor3   电机3电流
 * @param motor4   电机4电流
 * @return         none
 */
void DJ_124_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t motor1, uint16_t motor2, uint16_t motor3,
                 uint16_t motor4)
{
    static uint8_t tx_data[8];

    tx_data[0] = motor1 >> 8;
    tx_data[1] = motor1;
    tx_data[2] = motor2 >> 8;
    tx_data[3] = motor2;
    tx_data[4] = motor3 >> 8;
    tx_data[5] = motor3;
    tx_data[6] = motor4 >> 8;
    tx_data[7] = motor4;

    can_send_data(hcan, motor_id, tx_data, 0x08);
}

/**
 * @brief 3508/2006/6020速度环
 * @param hcan &hcan1 / &hcan2
 * @param motor_id 发送ID
 * @param pid_vel pid参数结构体
 * @param motor_measure 电机反馈数据
 * @param vel1 电机1转速
 * @param vel2 电机2转速
 * @param vel3 电机3转速
 * @param vel4 电机4转速
 * @return none
 */
void DJ_124_ctrl_vel(CAN_HandleTypeDef *hcan, uint16_t motor_id, pid_type_def pid_vel[4],
                     motor_measure_DJ_t motor_measure[4], int16_t vel1, int16_t vel2, int16_t vel3, int16_t vel4)
{
    int16_t motor_ctrl_vel[4] = {0};
    motor_ctrl_vel[0] = vel1;
    motor_ctrl_vel[1] = vel2;
    motor_ctrl_vel[2] = vel3;
    motor_ctrl_vel[3] = vel4;
    static int16_t motor_I[4] = {0};

    static uint8_t i = 0;
    for (i = 0; i < 4; i += 1)
    {
        motor_I[i] = PID_Calc(&pid_vel[i], motor_measure[i].speed_rpm, motor_ctrl_vel[i], 0);
    }

    DJ_124_ctrl(hcan, motor_id, motor_I[0], motor_I[1], motor_I[2], motor_I[3]);
}

/**
 * @brief 3508/2006/6020位置环
 * @param hcan &hcan1 / &hcan2
 * @param motor_id 发送ID
 * @param pid_vel pid参数结构体
 * @param motor_measure 电机反馈数据
 * @param pos1 电机1位置
 * @param pos2 电机2位置
 * @param pos3 电机3位置
 * @param pos4 电机4位置
 * @return none
 */
void DJ_124_ctrl_pos(CAN_HandleTypeDef *hcan, uint16_t motor_id, pid_type_def pid_pos[4], pid_type_def pid_vel[4],
                     motor_measure_DJ_t motor_measure[4], int16_t pos1, int16_t pos2, int16_t pos3, int16_t pos4)
{
    int16_t motor_ctrl_pos[4] = {0};
    motor_ctrl_pos[0] = pos1;
    motor_ctrl_pos[1] = pos2;
    motor_ctrl_pos[2] = pos3;
    motor_ctrl_pos[3] = pos4;
    static int16_t motor_I[4] = {0};

    static uint8_t i = 0;
    for (i = 0; i < 4; i += 1)
    {
        motor_I[i] = PID_Calc(&pid_pos[i], motor_measure[i].ecd, motor_ctrl_pos[i], 8192);
    }

    DJ_124_ctrl_vel(hcan, motor_id, pid_vel, motor_measure, motor_I[0], motor_I[1], motor_I[2], motor_I[3]);
}

/**
 * @brief          DM电机数据解包
 * @param motor    单位电机地址
 * @param rx_data  接收数据
 * @return         none
 */
void DM_motor_fbdata(motor_measure_DM_t *motor, uint8_t *rx_data)
{
    motor->id = (rx_data[0]) & 0x0F;                       // 电机控制id：data[0]高四位
    motor->err = (rx_data[0]) >> 4;                        // 错误状态data[0]的后四位
    motor->p_int = (rx_data[1] << 8) | rx_data[2];         // 位置值data[1][2]
    motor->v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);  // 速度值data[3]和data[4]高四位
    motor->t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5]; // 电机扭矩data[4]后四位和data[5]
    // 线性映射，将浮点型数据转换成有符号的定点数据
    motor->pos = uint_to_float(motor->p_int, DM_P_MIN, DM_P_MAX, 16) * 57.6; // (-12.5,12.5)->(-720,720)
    motor->vel = uint_to_float(motor->v_int, DM_V_MIN, DM_V_MAX, 12);        // (-45.0,45.0)
    motor->tor = uint_to_float(motor->t_int, DM_T_MIN, DM_T_MAX, 12);        // (-18.0,18.0)
    motor->T_mos = (float)(rx_data[6]);                                      // 驱动上mos的平均温度
    motor->T_Rotor = (float)(rx_data[7]);                                    // 电机线圈的平均温度
}

/**
 * @brief          DM3510电机数据解包
 * @param motor    单位电机地址
 * @param rx_data  接收数据
 * @return         none
 */
void DM_custom_motor_fbdata(motor_measure_DM_t *motor, uint8_t *rx_data)
{
    motor->id = (rx_data[0]) & 0x0F;
    motor->err = (rx_data[0]) >> 4;
    motor->p_int = (rx_data[1] << 8) | rx_data[2];
    motor->v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->pos = uint_to_float(motor->p_int, DM_P_MIN_3510, DM_P_MAX_3510, 16) * 57.6; // (-125,125)->(-7200,7200)
    motor->vel = uint_to_float(motor->v_int, DM_V_MIN, DM_V_MAX, 12);                  // (-45.0,45.0)
    motor->tor = uint_to_float(motor->t_int, DM_T_MIN, DM_T_MAX, 12);                  // (-18.0,18.0)
    motor->T_mos = (float)(rx_data[6]);
    motor->T_Rotor = (float)(rx_data[7]);
}

/**
 * @brief MIT模式控制
 * @param hcan 所在can接口
 * @param motor_id 电机id
 * @param pos 电机角度
 * @param vel 电机速度
 * @param kp 电机p
 * @param kd 电机d
 * @param torq 电机扭矩
 * @return none
 */
/**
 * kp=0,kd!=0,给定V_des实现匀速转动，kp，kd为0，给定torq实现定扭矩输出，对位置进行控制时kd！=0.
 */
void DM_motor_MIT_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    uint8_t data[8] = {0};
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id;
    pos = pos / 57.6; // 角度转弧度
    pos_tmp = float_to_uint(pos, DM_P_MIN, DM_P_MAX, 16);
    vel_tmp = float_to_uint(vel, DM_V_MIN, DM_V_MAX, 12);
    kp_tmp = float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    kd_tmp = float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    tor_tmp = float_to_uint(torq, DM_T_MIN, DM_T_MAX, 12);

    data[0] = (pos_tmp >> 8);                         // data[0][1]位置值
    data[1] = pos_tmp;                                // data[2][3]速度值和kp高4位
    data[2] = (vel_tmp >> 4);                         // data[4][5]kp低4位和kd高4位
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8); // data[6]kd低4位和力矩高4位
    data[4] = kp_tmp;                                 // data[7]力矩低4位
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;

    canx_send_data(hcan, id, data, 8);
}

/**
 * @brief 自定义参数MIT模式控制
 * @param hcan 所在can接口
 * @param motor_id 电机id
 * @param pos 电机角度
 * @param vel 电机速度
 * @param kp 电机p
 * @param kd 电机d
 * @param torq 电机扭矩
 * @return none
 */
/**
 * kp=0,kd!=0,给定V_des实现匀速转动，kp，kd为0，给定torq实现定扭矩输出，对位置进行控制时kd！=0.
 */
void DM_custom_motor_MIT_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd,
                              float torq)
{
    uint8_t data[8] = {0};
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id;
    pos = pos / 57.6; // 角度转弧度
    pos_tmp = float_to_uint(pos, DM_P_MIN_3510, DM_P_MAX_3510, 16);
    vel_tmp = float_to_uint(vel, DM_V_MIN, DM_V_MAX, 12);
    kp_tmp = float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    kd_tmp = float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    tor_tmp = float_to_uint(torq, DM_T_MIN, DM_T_MAX, 12);

    data[0] = (pos_tmp >> 8);                         // data[0][1]位置值
    data[1] = pos_tmp;                                // data[2][3]速度值和kp高4位
    data[2] = (vel_tmp >> 4);                         // data[4][5]kp低4位和kd高4位
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8); // data[6]kd低4位和力矩高4位
    data[4] = kp_tmp;                                 // data[7]力矩低4位
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;

    canx_send_data(hcan, id, data, 8);
}

/**
 * @brief 位置速度模式控制
 * @param hcan 所在can接口
 * @param motor_id 电机id
 * @param pos 电机角度
 * @param vel 电机最大速度
 * @return none
 */
void DM_motor_posvel_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel)
{
    uint16_t id;
    uint8_t *pbuf, *vbuf;
    uint8_t data[8];

    id = motor_id + 0x100;
    pbuf = (uint8_t *)&pos;
    vbuf = (uint8_t *)&vel;

    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);

    data[4] = *vbuf;
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);

    canx_send_data(hcan, id, data, 8);
}

void DM_motor_enable(CAN_HandleTypeDef *hcan, uint16_t motor_id)
{
    uint8_t data[8];
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;
    canx_send_data(hcan, motor_id, data, 8);
}

void DM_motor_disable(CAN_HandleTypeDef *hcan, uint16_t motor_id)
{
    uint8_t data[8];
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;
    canx_send_data(hcan, motor_id, data, 8);
}

void DM_motor_zeroset(CAN_HandleTypeDef *hcan, uint16_t motor_id)
{
    uint8_t data[8];
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFE;
    canx_send_data(hcan, motor_id, data, 8);
}
