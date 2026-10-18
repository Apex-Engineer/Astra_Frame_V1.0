#ifndef __REFEREE_H__
#define __REFEREE_H__

#include "main.h"
#include "crc_ref.h"
#include "string.h"
#include "fifo.h"
#include "VT13_rc_ctrl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"



#define USART_RX_BUF_LENGHT     256
#define REFEREE_FIFO_BUF_LENGTH 512
#define USART8_RX_BUF_LENGTH	 256

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)  // 5个字节
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))




typedef  struct __attribute__((packed)) 
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;


typedef enum 
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} referee_unpack_step_e;


typedef struct __attribute__((packed)) 
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  referee_unpack_step_e  unpack_step;
  uint16_t       index;
} referee_unpack_data_t;


/*裁判系统回包结构体*/

/* ID: 0x0001  Byte:  11    比赛状态数据 */
typedef  struct __attribute__((packed))
{ 
	uint8_t game_type; 
	uint8_t game_progress; 
	uint16_t stage_remain_time; 
	uint64_t SyncTimeStamp; 
} ext_game_state_t;

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef struct __attribute__((packed))
{
	uint8_t winner;
} ext_game_result_t;

/* ID: 0x0003  Byte:  32    比赛机器人血量数据 */
typedef struct __attribute__((packed))
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef struct __attribute__((packed))
{
	uint32_t event_type;
} ext_event_data_t;

/* ID: 0X0201  Byte: 13    机器人状态数据 */
typedef struct __attribute__((packed))
{
	uint8_t robot_id; 
	uint8_t robot_level; 
	uint16_t current_HP; 
	uint16_t maximum_HP; 
	uint16_t shooter_barrel_cooling_value; 
	uint16_t shooter_barrel_heat_limit; 
	uint16_t chassis_power_limit; 
	uint8_t power_management_gimbal_output : 1; 
	uint8_t power_management_chassis_output : 1; 
	uint8_t power_management_ammo_output : 1; 
} ext_game_robot_state_t;

/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef struct __attribute__((packed))
{
	uint16_t chassis_voltage;                   //寄辣
	uint16_t chassis_current;                   //寄辣
	float chassis_power;                        //寄辣
	uint16_t buffer_energy; 
	uint16_t shooter_17mm_barrel_heat; 
	uint16_t shooter_42mm_barrel_heat; 
} ext_power_heat_data_t;

/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef struct __attribute__((packed))
{
	float x;
	float y;
	float yaw;
} ext_game_robot_pos_t;

/* ID: 0x0204  Byte:  6    机器人增益数据 */
typedef struct __attribute__((packed))
{
	uint8_t recovery_buff; 
	uint8_t cooling_buff; 
	uint8_t defence_buff; 
	uint8_t vulnerability_buff; 
	uint8_t attack_buff; 
    uint16_t remaining_energy;
} ext_buff_musk_t;

/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef struct __attribute__((packed))
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef struct __attribute__((packed))
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
} ext_shoot_data_t;

/* ID: 0x0302  Byte:  -    自定义控制器发包 */
typedef struct __attribute__((packed))
{ 
    uint8_t data[30]; 
}custom_robot_data_t;

/* ID: 0x0309  Byte:  -    机器人给自定义控制器的发包 */
typedef struct __attribute__((packed))
{ 
    uint8_t data[30];//100 
}receive_robot_data_t;

/* ID: 0x0304  Byte:  12    键鼠数据 */
typedef struct __attribute__((packed))
{ 
int16_t mouse_x; 
int16_t mouse_y; 
int16_t mouse_z; 
int8_t left_button_down; 
int8_t right_button_down; 
uint16_t keyboard_value; 
uint16_t reserved; 
}remote_control_t;

typedef struct __attribute__((packed))
{
    frame_header_struct_t FrameHeader;
	ext_game_state_t GameState;							   // 0x0001
	ext_game_result_t GameResult;						   // 0x0002
	ext_game_robot_HP_t GameRobotHP;					   // 0x0003
	ext_event_data_t EventData;							   // 0x0101
	ext_game_robot_state_t GameRobotState;				   // 0x0201
	ext_power_heat_data_t PowerHeatData;				   // 0x0202
	ext_game_robot_pos_t GameRobotPos;					   // 0x0203
	ext_buff_musk_t BuffMusk;							   // 0x0204
	ext_robot_hurt_t RobotHurt;							   // 0x0206
	ext_shoot_data_t ShootData;							   // 0x0207
	custom_robot_data_t ReceiveData;                       // 0x0302
    remote_control_t remote_control_t;                     // 0x0304 
	receive_robot_data_t receive_robot_data_t;             // 0x0309


	int init_flag[3];
	

} data_referee_t;

/*
 * 串口解析数据（30字节帧 = 6×float pos + 6字节reserved）
 * pos[0~5]: 6个关节目标位置（直接为float，单位：度）
 * vel/tor:  保留数组，当前串口帧不包含速度/扭矩数据
 */
extern float pos[6];
extern float vel[6];
extern float tor[6];


/*裁判系统回包结构体*/
void referee_unpack_fifo_data(void);
void Referee_Task(void const *argument);
void referee_usart6_idle_handler(void);
void referee_uart8_idle_handler(void);
void referee_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void referee_read_data(uint8_t *frame);
void uart8_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void unpack_custom_data(uint8_t *frame);

void Init(void);

#endif /* __REFEREE_H__ */

