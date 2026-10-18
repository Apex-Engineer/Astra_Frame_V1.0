#include "RM_Cilent_UI.h"
#include <string.h>

unsigned char UI_Seq;

/****************************************???????????************************************/
void UI_SendByte(unsigned char ch)
{
    while (!(USART6->SR & UART_FLAG_TXE))
    {
    }
    USART6->DR = ch;
}

/********************************************???????*************************************
**参数：Del_Operate  对应头文件删除操作
        Del_Layer    要删除的层 取值0-9
*****************************************************************************************/

void UI_Delete(u8 Del_Operate, u8 Del_Layer)
{

    unsigned char *framepoint;
    u16 frametail = 0xFFFF;
    int loop_control;

    UI_Packhead framehead;
    UI_Data_Operate datahead;
    UI_Data_Delete del;

    framepoint = (unsigned char *)&framehead;

    framehead.SOF = UI_SOF;
    framehead.Data_Length = 8;
    framehead.Seq = UI_Seq;
    framehead.CRC8 = Get_CRC8_Check_Sum(framepoint, 4, 0xFF);
    framehead.CMD_ID = UI_CMD_Robo_Exchange;

    datahead.Data_ID = UI_Data_ID_Del;
    datahead.Sender_ID = Robot_ID;
    datahead.Receiver_ID = Cilent_ID;

    del.Delete_Operate = Del_Operate;
    del.Layer = Del_Layer;

    frametail = Get_CRC16_Check_Sum(framepoint, sizeof(framehead), frametail);
    framepoint = (unsigned char *)&datahead;
    frametail = Get_CRC16_Check_Sum(framepoint, sizeof(datahead), frametail);
    framepoint = (unsigned char *)&del;
    frametail = Get_CRC16_Check_Sum(framepoint, sizeof(del), frametail);

    framepoint = (unsigned char *)&framehead;
    for (loop_control = 0; loop_control < sizeof(framehead); loop_control++)
    {
        UI_SendByte(*framepoint);
        framepoint++;
    }
    framepoint = (unsigned char *)&datahead;
    for (loop_control = 0; loop_control < sizeof(datahead); loop_control++)
    {
        UI_SendByte(*framepoint);
        framepoint++;
    }
    framepoint = (unsigned char *)&del;
    for (loop_control = 0; loop_control < sizeof(del); loop_control++)
    {
        UI_SendByte(*framepoint);
        framepoint++;
    }
    framepoint = (unsigned char *)&frametail;
    for (loop_control = 0; loop_control < sizeof(frametail); loop_control++)
    {
        UI_SendByte(*framepoint);
        framepoint++;
    }

    UI_Seq++;
}

void Line_Draw(interaction_figure_t *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color,
               u32 Graph_Width, u32 Start_x, u32 Start_y, u32 End_x, u32 End_y)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
    {
        image->figure_name[2 - i] = imagename[i];
    }
    image->figure_tpye = UI_Graph_Line;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    // 对于直线：details_d = 终点x坐标，details_e = 终点y坐标
    image->details_d = End_x;
    image->details_e = End_y;
    // 对于直线，details_a、details_b、details_c可能不需要设置或设置为0
    image->details_a = 0;
    image->details_b = 0;
    image->details_c = 0;
}

void Rectangle_Draw(interaction_figure_t *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color,
                    u32 Graph_Width, u32 Start_x, u32 Start_y, u32 End_x, u32 End_y)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
    {
        image->figure_name[2 - i] = imagename[i];
    }

    image->figure_tpye = UI_Graph_Rectangle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    // 对于矩形：details_d = 对角顶点x坐标，details_e = 对角顶点y坐标
    image->details_d = End_x;
    // 对角顶点x坐标
    image->details_e = End_y;
    // 对角顶点y坐标
    // details_a、details_b、details_c对于矩形不需要设置
    image->details_a = 0;
    image->details_b = 0;
    image->details_c = 0;
}

void Circle_Draw(interaction_figure_t *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color,
                 u32 Graph_Width, u32 Start_x, u32 Start_y, u32 Graph_Radius)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
    {
        image->figure_name[2 - i] = imagename[i];
    }
    image->figure_tpye = UI_Graph_Circle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->details_c = Graph_Radius;

    image->details_a = 0;
    image->details_b = 0;
    image->details_d = 0;
    image->details_e = 0;
}

/**
 * @brief 画弧线
 */
void Arc_Draw(interaction_figure_t *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color,
              u32 Graph_StartAngle, u32 Graph_EndAngle, u32 Graph_Width, u32 Start_x, u32 Start_y, u32 x_Length,
              u32 y_Length)
{
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
    {
        image->figure_name[2 - i] = imagename[i];
    }
    image->figure_tpye = UI_Graph_Arc;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->details_a = Graph_StartAngle;
    image->details_b = Graph_EndAngle;
    image->details_d = x_Length;
    image->details_e = y_Length;

    image->details_c = 0;
}

void Float_Draw(interaction_figure_t *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color,
                u32 Graph_Size, u32 Graph_Width, u32 Start_x, u32 Start_y, float Graph_Float)
{
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
    {
        image->figure_name[2 - i] = imagename[i];
    }
    image->figure_tpye = UI_Graph_Float;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width; // 建议字体大小与线宽比例为 10：1
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->details_a = Graph_Size;
    image->details_b = 0;
    image->details_c = (int32_t)(Graph_Float * 1000);
    image->details_d = (int32_t)(Graph_Float * 1000);
    image->details_e = (int32_t)(Graph_Float * 1000);
}

void Char_Draw(String_Data *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color,
               u32 Graph_Size, u32 Graph_length, u32 Graph_Width, u32 Start_x, u32 Start_y, char *Char_Data)
{
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
    {
        image->Graph_Control.figure_name[2 - i] = imagename[i];
    }
    image->Graph_Control.figure_tpye = UI_Graph_Char;
    image->Graph_Control.operate_tpye = Graph_Operate;
    image->Graph_Control.layer = Graph_Layer;
    image->Graph_Control.color = Graph_Color;
    image->Graph_Control.width = Graph_Width;
    image->Graph_Control.start_x = Start_x;
    image->Graph_Control.start_y = Start_y;
    image->Graph_Control.details_a = Graph_Size;
    image->Graph_Control.details_b = Graph_length;

    image->Graph_Control.details_c = 0;
    image->Graph_Control.details_d = 0;
    image->Graph_Control.details_e = 0;

    memset(image->show_Data, 0, sizeof(image->show_Data));

    for (i = 0; i < Graph_length; i++)
    {
        image->show_Data[i] = *Char_Data;
        Char_Data++;
    }
}

/**
 * @brief 椭圆
 */
void Ellipse_Draw(interaction_figure_t *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color,
                  u32 Graph_Width, u32 Start_x, u32 Start_y, u32 x_Length, u32 y_Length)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
    {
        image->figure_name[2 - i] = imagename[i];
    }
    image->figure_tpye = UI_Graph_Ellipse;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->details_d = x_Length; // x半轴长度
    image->details_e = y_Length; // y半轴长度

    image->details_a = 0;
    image->details_b = 0;
    image->details_c = 0;
}

/**
 * @brief 整数显示
 */
void Int_Draw(interaction_figure_t *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color,
              u32 Graph_Size, u32 Graph_Width, u32 Start_x, u32 Start_y, int32_t Graph_Int)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->figure_name[2 - i] = imagename[i];
    image->figure_tpye = UI_Graph_Int;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->details_a = Graph_Size; // 字体大小

    image->details_b = 0;
    image->details_c = Graph_Int; // 整数值
    image->details_d = Graph_Int; //
    image->details_e = Graph_Int; //
}

/************************************************UI*******************************************************
**********************************************************************************************************/
int UI_ReFresh(int cnt, ...)
{
    int i, n;
    interaction_figure_t imageData;
    unsigned char *framepoint;
    u16 frametail = 0xFFFF;

    UI_Packhead framehead;
    UI_Data_Operate datahead;
    va_list ap;

    va_start(ap, cnt);

    framepoint = (unsigned char *)&framehead;
    framehead.SOF = UI_SOF;
    framehead.Data_Length = 6 + cnt * 15;
    framehead.Seq = UI_Seq;

    framehead.CRC8 = Get_CRC8_Check_Sum(framepoint, 4, 0xFF);
    framehead.CMD_ID = UI_CMD_Robo_Exchange;

    switch (cnt)
    {
    case 1:
        datahead.Data_ID = UI_Data_ID_Draw1;
        break;
    case 2:
        datahead.Data_ID = UI_Data_ID_Draw2;
        break;
    case 5:
        datahead.Data_ID = UI_Data_ID_Draw5;
        break;
    case 7:
        datahead.Data_ID = UI_Data_ID_Draw7;
        break;
    default:
        va_end(ap);
        return (-1);
    }
    datahead.Sender_ID = Robot_ID;
    datahead.Receiver_ID = Cilent_ID;

    framepoint = (unsigned char *)&framehead;
    frametail = Get_CRC16_Check_Sum(framepoint, sizeof(framehead), frametail);
    framepoint = (unsigned char *)&datahead;
    frametail = Get_CRC16_Check_Sum(framepoint, sizeof(datahead), frametail);

    framepoint = (unsigned char *)&framehead;
    for (i = 0; i < sizeof(framehead); i++)
    {
        UI_SendByte(*framepoint);
        framepoint++;
    }
    framepoint = (unsigned char *)&datahead;
    for (i = 0; i < sizeof(datahead); i++)
    {
        UI_SendByte(*framepoint);
        framepoint++;
    }

    for (i = 0; i < cnt; i++)
    {
        imageData = va_arg(ap, interaction_figure_t);
        framepoint = (unsigned char *)&imageData;
        frametail = Get_CRC16_Check_Sum(framepoint, sizeof(imageData), frametail);

        for (n = 0; n < sizeof(imageData); n++)
        {
            UI_SendByte(*framepoint);
            framepoint++;
        }
    }
    framepoint = (unsigned char *)&frametail;
    for (i = 0; i < sizeof(frametail); i++)
    {
        UI_SendByte(*framepoint);
        framepoint++;
    }

    va_end(ap);

    UI_Seq++;
    return 0;
}

/************************************************UI*******************************************************
**********************************************************************************************************/
int Char_ReFresh(String_Data string_Data)
{
    int i;
    String_Data imageData;
    unsigned char *framepoint;
    u16 frametail = 0xFFFF;

    UI_Packhead framehead;
    UI_Data_Operate datahead;
    imageData = string_Data;

    framepoint = (unsigned char *)&framehead;
    framehead.SOF = UI_SOF;
    framehead.Data_Length = 6 + 45;
    framehead.Seq = UI_Seq;
    framehead.CRC8 = Get_CRC8_Check_Sum(framepoint, 4, 0xFF);
    framehead.CMD_ID = UI_CMD_Robo_Exchange;

    datahead.Data_ID = UI_Data_ID_DrawChar;

    datahead.Sender_ID = Robot_ID;
    datahead.Receiver_ID = Cilent_ID;

    framepoint = (unsigned char *)&framehead;
    frametail = Get_CRC16_Check_Sum(framepoint, sizeof(framehead), frametail);
    framepoint = (unsigned char *)&datahead;
    frametail = Get_CRC16_Check_Sum(framepoint, sizeof(datahead), frametail);
    framepoint = (unsigned char *)&imageData;
    frametail = Get_CRC16_Check_Sum(framepoint, sizeof(imageData), frametail);

    framepoint = (unsigned char *)&framehead;
    for (i = 0; i < sizeof(framehead); i++)
    {
        UI_SendByte(*framepoint);
        framepoint++;
    }
    framepoint = (unsigned char *)&datahead;
    for (i = 0; i < sizeof(datahead); i++)
    {
        UI_SendByte(*framepoint);
        framepoint++;
    }
    framepoint = (unsigned char *)&imageData;
    for (i = 0; i < sizeof(imageData); i++)
    {
        UI_SendByte(*framepoint);
        framepoint++;
    }

    framepoint = (unsigned char *)&frametail;
    for (i = 0; i < sizeof(frametail); i++)
    {
        UI_SendByte(*framepoint);
        framepoint++;
    }

    UI_Seq++;
    return 0;
}
