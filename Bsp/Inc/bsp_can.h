#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include "can.h"

void can_filter_init(void);
uint8_t can_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);

#endif
