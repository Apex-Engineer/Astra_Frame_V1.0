#include "CAN_receive.h"

extern data_motor_t data_motor;
// extern data_g2c_gimbal_t data_g2c_gimbal;
// extern data_g2c_chassis_t data_g2c_chassis;
// extern data_c2g_gimbal_t data_c2g_gimbal;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if (hcan == &hcan1)
    {
        switch (rx_header.StdId)
        {
        case 0x201:
            DJ_motor_fbdata(&data_motor.Chassis_motor_measure[0], rx_data);
            DJ_motor_fbdata(&data_motor.Hex_motor_measure[0], rx_data);
            break;
        case 0x202:
            DJ_motor_fbdata(&data_motor.Chassis_motor_measure[1], rx_data);
            DJ_motor_fbdata(&data_motor.Hex_motor_measure[1], rx_data);
            break;
        case 0x203:
            DJ_motor_fbdata(&data_motor.Chassis_motor_measure[2], rx_data);
            DJ_motor_fbdata(&data_motor.Hex_motor_measure[2], rx_data);
            break;
        case 0x204:
            DJ_motor_fbdata(&data_motor.Chassis_motor_measure[3], rx_data);
            DJ_motor_fbdata(&data_motor.Hex_motor_measure[3], rx_data);
            break;
        case 0x205:
            DJ_motor_fbdata(&data_motor.Hex_motor_measure[4], rx_data); // 抬升
            break;
        case 0x206:
            DJ_motor_fbdata(&data_motor.Hex_motor_measure[5], rx_data); // 前伸
            break;
        // case 0x202:DM_motor_fbdata(&data_motor.custom_motor_measure[0] , rx_data);break;
        // case 0x204:DM_motor_fbdata(&data_motor.custom_motor_measure[1] , rx_data);break;
        // case 0x206:DM_motor_fbdata(&data_motor.custom_motor_measure[2] , rx_data);break;
        // case 0x208:DM_motor_fbdata(&data_motor.custom_motor_measure[3] , rx_data);break;
        // case 0x20A:DM_motor_fbdata(&data_motor.custom_motor_measure[4] , rx_data);break;
        default:
            break;
        }
    }
    else if (hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {
        //12.5
        case 0x201:
            DM_custom_motor_fbdata(&data_motor.arm_motor_measure[0], rx_data);
            break;
        case 0x203:
            DM_custom_motor_fbdata(&data_motor.arm_motor_measure[1], rx_data);
            break;
        // case 0x205:
        //     DM_motor_fbdata(&data_motor.arm_motor_measure[2], rx_data);
        //     break;
        //1250
        // case 0x201:
        //     DM_custom_motor_fbdata(&data_motor.arm_motor_measure[0], rx_data);
        //     break;
        // case 0x203:
        //     DM_custom_motor_fbdata(&data_motor.arm_motor_measure[1], rx_data);
        //     break;
        case 0x205:
            DM_custom_motor_fbdata(&data_motor.arm_motor_measure[2], rx_data);
            break;

        case 0x207:
            DM_motor_fbdata(&data_motor.arm_motor_measure[3], rx_data);
            break;
        case 0x209:
            DM_motor_fbdata(&data_motor.arm_motor_measure[4], rx_data);
            break;
        case 0x211:
            DM_custom_motor_fbdata(&data_motor.arm_motor_measure[6], rx_data);
            break;
        case 0x213:
            // DJ_motor_fbdata(&data_motor.Ammo_motor_measure[1], rx_data);//前伸
            DM_motor_fbdata(&data_motor.arm_motor_measure[5], rx_data); // j2:键鼠控制
            break;
            // case 0x201:
            //     DM_motor_fbdata(&data_motor.arm_motor_measure[0], rx_data);
            //     break;
            // case 0x01:
            //     DM_motor_fbdata(&data_motor.arm_motor_measure[0], rx_data);
            // break;

        default:
            break;
        }
    }
}