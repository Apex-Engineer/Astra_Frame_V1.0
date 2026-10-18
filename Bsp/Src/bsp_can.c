#include "bsp_can.h"
#include <string.h>


/**
  * @brief          can发数据
  * @param hcan     can位（&hcan1/&hcan2）
  * @param id       can id
  * @param data     发送数据
  * @param len      数据长度
  * @return         none 
  */
uint8_t can_send_data(CAN_HandleTypeDef *hcan , uint16_t id , uint8_t *data , uint32_t len)
{	
	CAN_TxHeaderTypeDef	tx_header;
    uint32_t tx_mailbox;
	
	tx_header.StdId = id;
	tx_header.ExtId = 0;
	tx_header.IDE   = 0;
	tx_header.RTR   = 0;
	tx_header.DLC   = len;
	if(HAL_CAN_AddTxMessage(hcan , &tx_header , data , &tx_mailbox) != HAL_OK) 
    {
        return 1;
    }
    return 0;
}

/**
  * @brief can过滤器初始化
  * @return none 
  */
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    memset(&can_filter_st, 0, sizeof(can_filter_st));
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_st.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
