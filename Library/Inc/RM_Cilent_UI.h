#ifndef __RM_CILENT_UI__
#define __RM_CILENT_UI__

#define Robot_ID UI_Data_RobotID_BEngineer
#define Cilent_ID UI_Data_CilentID_BEngineer       

#include "stm32f4xx.h"
#include "stdarg.h"
#include "usart.h"
#include "stdint.h"
#include "crc_ref.h"

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

#pragma pack(1)                           

#ifndef NULL
#define NULL 0
#endif
#define __FALSE 100 

/*******************************************************************/
#define UI_SOF 0xA5
/*******************************************************************/
#define UI_CMD_Robo_Exchange 0x0301    
/*******************************************************************/
#define UI_Data_ID_Del 0x100 
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/*******************************************************************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/*******************************************************************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/*******************************************************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/*******************************************************************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/*******************************************************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/*******************************************************************/
#define UI_Graph_NoOp 0
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/*******************************************************************/
#define UI_Graph_Line 0         
#define UI_Graph_Rectangle 1    
#define UI_Graph_Circle 2       
#define UI_Graph_Ellipse 3      
#define UI_Graph_Arc 4          
#define UI_Graph_Float 5        
#define UI_Graph_Int 6          
#define UI_Graph_Char 7         
/*******************************************************************/
#define UI_Color_Main 0         
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         
#define UI_Color_Black 7
#define UI_Color_White 8



typedef unsigned char Uint8_t;
typedef unsigned char U8;




typedef struct __attribute__((packed))
{
   u8 SOF;                    
   u16 Data_Length;           
   u8 Seq;                   
   u8 CRC8;                   
   u16 CMD_ID;              
} UI_Packhead;             

typedef struct __attribute__((packed))
{
   u16 Data_ID;               
   u16 Sender_ID;             
   u16 Receiver_ID;           
   //u8 user_data[112];          
} UI_Data_Operate;         

typedef struct __attribute__((packed))
{
   u8 Delete_Operate;         
   u8 Layer;                  
} UI_Data_Delete;          


typedef struct __attribute__((packed))
{ 
   uint8_t graphic_name[3]; 
   uint32_t operate_tpye:3; 
   uint32_t graphic_tpye:3; 
   uint32_t layer:4; 
   uint32_t color:4; 
   uint32_t start_angle:9;
   uint32_t end_angle:9;
   uint32_t width:10; 
   uint32_t start_x:11; 
   uint32_t start_y:11;
   float graph_Float;             
} Float_Data;


typedef struct __attribute__((packed))
{ 
// uint8_t graphic_name[3]; 
// uint32_t operate_tpye:3; 
// uint32_t graphic_tpye:3; 
// uint32_t layer:4; 
// uint32_t color:4; 
// uint32_t start_angle:9;
// uint32_t end_angle:9;
// uint32_t width:10; 
// uint32_t start_x:11; 
// uint32_t start_y:11;
// uint32_t radius:10; 
// uint32_t end_x:11; 
// uint32_t end_y:11;              
uint8_t figure_name[3];
uint32_t operate_tpye:3;
uint32_t figure_tpye:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t details_a:9;
uint32_t details_b:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t details_c:10;
uint32_t details_d:11;
uint32_t details_e:11;
} interaction_figure_t; 


typedef struct __attribute__((packed))
{
   interaction_figure_t Graph_Control;
   uint8_t show_Data[30];
} String_Data;                  



void UI_Delete(u8 Del_Operate,u8 Del_Layer);
void Line_Draw(interaction_figure_t *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y);
int UI_ReFresh(int cnt,...);
unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
void Circle_Draw(interaction_figure_t *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 Graph_Radius);
void Rectangle_Draw(interaction_figure_t *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y);
void Float_Draw(interaction_figure_t *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color, u32 Graph_Size, u32 Graph_Width, u32 Start_x, u32 Start_y, float Graph_Float);
void Char_Draw(String_Data  *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color, u32 Graph_Size, u32 Graph_length, u32 Graph_Width, u32 Start_x, u32 Start_y, char *Char_Data);
int Char_ReFresh(String_Data string_Data);
void Arc_Draw(interaction_figure_t *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_StartAngle,u32 Graph_EndAngle,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 x_Length,u32 y_Length);

#endif
