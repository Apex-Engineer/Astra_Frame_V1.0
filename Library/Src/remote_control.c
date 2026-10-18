#include "remote_control.h"

RC_ctrl_t rc_ctrl = {0};
data_rc_t data_rc = {0};
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // enable the DMA transfer for the receiver request
    // 使能DMA串口接收
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    // enalbe idle interrupt
    // 使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    // disable DMA
    // 失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR = (uint32_t)&(USART1->DR);
    // memory buffer 1
    // 内存缓冲区1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    // memory buffer 2
    // 内存缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    // data length
    // 数据长度
    hdma_usart1_rx.Instance->NDTR = dma_buf_num;
    // enable double memory buffer
    // 使能双缓冲区
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    // enable DMA
    // 使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}

void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &
                        0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                       //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 // NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

void rc_read_data(data_rc_t *data_rc)
{
    data_rc->speed_yaw = data_rc->RC_ctrl->rc.ch[2] * 5.0;//CHASSIS_OPEN_RC_SCALE;
    data_rc->angle_pitch += (data_rc->RC_ctrl->rc.ch[3] * 1.0 / 66000);
    if (data_rc->angle_pitch >= 2.0f) // 限位****
    {
        data_rc->angle_pitch = 2.0;
    }
    else if (data_rc->angle_pitch <= 0.3f) // 限位***
    {
        data_rc->angle_pitch = 0.3;
    }

    data_rc->angle_yaw += (data_rc->RC_ctrl->rc.ch[2] * 1.0 / 132000);
    if (data_rc->angle_yaw >= 180)
    {
        data_rc->angle_yaw -= 360;
    }
    else if (data_rc->angle_yaw <= -180)
    {
        data_rc->angle_yaw += 360;
    }

    switch (data_rc->RC_ctrl->rc.s[0])
    {
    case 2:
        data_rc->status_chassis = 0;
        data_rc->status_gimbal = 0;
        break;
    case 3:
        data_rc->status_chassis = 1;
        data_rc->status_gimbal = 1;
        break;
    case 1:
        data_rc->status_chassis = 2;
        data_rc->status_gimbal = 1;
        break;

    default:
        break;
    }

    if (data_rc->status_gimbal == 1)
    {
        switch (data_rc->RC_ctrl->rc.s[1])
        {
        case 2:
            data_rc->status_ammo = 0;
            break;
        case 3:
            data_rc->status_ammo = 1;
            break;
        case 1:
            data_rc->status_ammo = 2;
            break;

        default:
            break;
        }
    }
    else if (data_rc->status_gimbal != 1)
    {
        data_rc->status_ammo = 0;
    }

    if (data_rc->RC_ctrl->rc.ch[4] >= 550 || data_rc->RC_ctrl->rc.ch[4] <= -550)
    {
        data_rc->reset = 1;
    }
    else if (data_rc->RC_ctrl->rc.ch[4] < 550 && data_rc->RC_ctrl->rc.ch[4] > -550)
    {
        data_rc->reset = 0;
    }
}

const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

void remote_control_uart1_idle_handler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;
        __HAL_UART_CLEAR_PEFLAG(&huart1);
        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(&hdma_usart1_rx);
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
            // 重新设定数据长度
            __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, SBUS_RX_BUF_NUM);
            // 设定缓冲区1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart1_rx);
            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_usart1_rx);
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
            // 重新设定数据长度
            __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, SBUS_RX_BUF_NUM);
            // 设定缓冲区0
            hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart1_rx);
            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}
