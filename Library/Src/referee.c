#include "referee.h"
#include "remote_control.h"

extern RC_ctrl_t rc_ctrl;
data_referee_t data_referee;
uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
uint8_t uart8_buf[2][USART8_RX_BUF_LENGTH];
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
referee_unpack_data_t referee_unpack_obj;

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;

/** @brief 处理已接收的裁判系统数据；外设由 referee_system_init 初始化。 */
void Referee_Task(void const *argument)
{
    (void)argument;
    while (1)
    {
        referee_unpack_fifo_data();

        osDelay(10);
    }
}

void referee_system_init(void)
{
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    referee_Init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
    uart8_dma_init(uart8_buf[0], uart8_buf[1], USART8_RX_BUF_LENGTH);
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);
}

void referee_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    __HAL_DMA_DISABLE(&hdma_usart6_rx);

    while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    __HAL_DMA_ENABLE(&hdma_usart6_rx);

    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while (hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
}

void referee_unpack_fifo_data(void)
{
    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF;
    referee_unpack_data_t *p_obj = &referee_unpack_obj;

    while (fifo_s_used(&referee_fifo))
    {
        byte = fifo_s_get(&referee_fifo);
        switch (p_obj->unpack_step)
        {
        case STEP_HEADER_SOF:
        {
            if (byte == sof)
            {
                p_obj->unpack_step = STEP_LENGTH_LOW;
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            else
            {
                p_obj->index = 0;
            }
        }
        break;

        case STEP_LENGTH_LOW:
        {
            p_obj->data_len = byte;
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_LENGTH_HIGH;
        }
        break;

        case STEP_LENGTH_HIGH:
        {
            p_obj->data_len |= (byte << 8);
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
            {
                p_obj->unpack_step = STEP_FRAME_SEQ;
            }
            else
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
        }
        break;
        case STEP_FRAME_SEQ:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_HEADER_CRC8;
        }
        break;

        case STEP_HEADER_CRC8:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->index == 5)
            {
                if (Verify_CRC8_Check_Sum(p_obj->protocol_packet, 5))
                {
                    p_obj->unpack_step = STEP_DATA_CRC16;
                }
                else
                {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }
        }
        break;

        case STEP_DATA_CRC16:
        {
            uint16_t frame_len = REF_HEADER_CRC_CMDID_LEN + p_obj->data_len;

            /* 保存当前收到的字节 */
            if (p_obj->index < frame_len)
            {
                p_obj->protocol_packet[p_obj->index++] = byte;
            }

            /*
             * 保存后立即判断是否收齐一帧。
             * 注意：这里必须是独立的 if，不能使用 else if。
             */
            if (p_obj->index == frame_len)
            {
                if (Verify_CRC16_Check_Sum(p_obj->protocol_packet, frame_len))
                {
                    referee_read_data(p_obj->protocol_packet);
                }

                /* 无论校验是否通过，都准备接收下一帧 */
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
        }
        break;

        default:
        {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
        }
        break;
        }
    }
}
uint16_t cmd_id = 0;
void referee_read_data(uint8_t *frame)
{
    cmd_id = 0;

    uint8_t index = 0;

    memcpy(&data_referee.FrameHeader, frame, 5);

    index += 5;

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);
    switch (cmd_id)
    {
    case 0x0001:
        memcpy(&data_referee.GameState, frame + index, sizeof(ext_game_state_t));
        break;
    case 0x0002:
        memcpy(&data_referee.GameResult, frame + index, sizeof(ext_game_result_t));
        break;
    case 0x0003:
        memcpy(&data_referee.GameRobotHP, frame + index, sizeof(ext_game_robot_HP_t));
        break;
    case 0x0101:
        memcpy(&data_referee.EventData, frame + index, sizeof(ext_event_data_t));
        break;
    case 0x0201:
        memcpy(&data_referee.GameRobotState, frame + index, sizeof(ext_game_robot_state_t));
        break;
    case 0x0202:
        memcpy(&data_referee.PowerHeatData, frame + index, sizeof(ext_power_heat_data_t));
        break;
    case 0x0203:
        memcpy(&data_referee.GameRobotPos, frame + index, sizeof(ext_game_robot_pos_t));
        break;
    case 0x0204:
        memcpy(&data_referee.BuffMusk, frame + index, sizeof(ext_buff_musk_t));
        break;
    case 0x0206:
        memcpy(&data_referee.RobotHurt, frame + index, sizeof(ext_robot_hurt_t));
        break;
    case 0x0207:
        memcpy(&data_referee.ShootData, frame + index, sizeof(ext_shoot_data_t));
        break;
    case 0x0302:
    {
        /* 当前自定义控制协议的数据段长度应为30字节 */
        if (data_referee.FrameHeader.data_length != sizeof(custom_robot_data_t))
        {
            break;
        }

        memcpy(&data_referee.ReceiveData, frame + index, sizeof(custom_robot_data_t));

        /* 有效新帧解包完成后，直接更新机械臂目标位置 */
        unpack_custom_data(data_referee.ReceiveData.data);

        data_referee.init_flag[0] += 1;
    }
    break;
    default:
        break;
    }
}

void referee_usart6_idle_handler(void)
{
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);

        static uint16_t this_time_rx_len = 0;

        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char *)usart6_buf[0], this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char *)usart6_buf[1], this_time_rx_len);
        }
    }
}

void uart8_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    SET_BIT(huart8.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart8.Instance->CR3, USART_CR3_DMAT);

    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);

    __HAL_DMA_DISABLE(&hdma_uart8_rx);

    while (hdma_uart8_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart8_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_uart8_rx, DMA_LISR_TCIF1);

    hdma_uart8_rx.Instance->PAR = (uint32_t) & (UART8->DR);
    hdma_uart8_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    hdma_uart8_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    __HAL_DMA_SET_COUNTER(&hdma_uart8_rx, dma_buf_num);

    SET_BIT(hdma_uart8_rx.Instance->CR, DMA_SxCR_DBM);

    __HAL_DMA_ENABLE(&hdma_uart8_rx);

    __HAL_DMA_DISABLE(&hdma_uart8_tx);

    while (hdma_uart8_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart8_tx);
    }

    hdma_uart8_tx.Instance->PAR = (uint32_t) & (UART8->DR);
}

void referee_uart8_idle_handler(void)
{
    if (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart8);

        static uint16_t this_time_rx_len = 0;

        if ((huart8.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart8.hdmarx);
            this_time_rx_len = USART8_RX_BUF_LENGTH - __HAL_DMA_GET_COUNTER(huart8.hdmarx);
            __HAL_DMA_SET_COUNTER(huart8.hdmarx, USART8_RX_BUF_LENGTH);
            huart8.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart8.hdmarx);
            fifo_s_puts(&referee_fifo, (char *)uart8_buf[0], this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(huart8.hdmarx);
            this_time_rx_len = USART8_RX_BUF_LENGTH - __HAL_DMA_GET_COUNTER(huart8.hdmarx);
            __HAL_DMA_SET_COUNTER(huart8.hdmarx, USART8_RX_BUF_LENGTH);
            huart8.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart8.hdmarx);
            fifo_s_puts(&referee_fifo, (char *)uart8_buf[1], this_time_rx_len);
        }
    }
}

float pos[6] = {0};
float vel[6] = {0};
float tor[6] = {0};

/**
 * @brief  解析自定义串口数据帧
 *         帧格式: 6 × float (pos[0~5]) + 6字节 reserved = 30字节
 *         pos[0~4]: J6/J5/J4/J3/J1 目标位置（度）
 *         pos[5]:   Pitch1 目标位置（度）
 *         vel/tor:  当前帧不包含，保持为0
 */
void unpack_custom_data(uint8_t *frame)
{
    uint8_t *data_ptr = frame;

    /* 解析6个float位置数据 (6 × 4字节 = 24字节) */
    for (int i = 0; i < 6; i++)
    {
        /* STM32 小端序，直接按字节拷贝到 float */
        memcpy(&pos[i], data_ptr, sizeof(float));
        data_ptr += 4;
    }

    /* 剩余6字节为 reserved（全0），跳过 */
    /* vel[] 和 tor[] 保持为0，串口帧不包含速度/扭矩数据 */
}
