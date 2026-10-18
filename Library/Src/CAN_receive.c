#include "CAN_receive.h"

extern data_motor_t data_motor;
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
            DM_motor_fbdata(&data_motor.arm_motor_measure[5], rx_data); // j2:键鼠控制
            break;

        default:
            break;
        }
    }
}