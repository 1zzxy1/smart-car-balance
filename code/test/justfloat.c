/*
 * justfloat.c
 *
 *  Created on: 2025年4月29日
 *      Author: suiyungui
 */
/* 用于远程调试pid */
#include "justfloat.h"

#if JUSTFLOAT_USE_WIFI_SPI
    #define JUSTFLOAT_SEND(buf, len)  wifi_spi_send_buffer((buf), (len))
    #define JUSTFLOAT_FLUSH()         wifi_spi_udp_send_now()
#else
    #define JUSTFLOAT_SEND(buf, len)  wireless_uart_send_buffer((buf), (len))
    #define JUSTFLOAT_FLUSH()         ((void)0)
#endif

typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;

static void justfloat_send_values(const float *values, uint8 count)
{
    uint8 i;
    uint8 byte[4] = {0};
    uint8 tail[4] = {0x00, 0x00, 0x80, 0x7f};

    for(i = 0; i < count; i++)
    {
        Float_to_Byte(values[i], byte);
        JUSTFLOAT_SEND(byte, 4);
    }

    JUSTFLOAT_SEND(tail, 4);
    JUSTFLOAT_FLUSH();
}

uint8 justfloat_init(void)
{
#if JUSTFLOAT_USE_WIFI_SPI
    if(wifi_spi_init(JUSTFLOAT_WIFI_SSID, JUSTFLOAT_WIFI_PASSWORD) != 0)
    {
        return 1;
    }

    if(wifi_spi_socket_connect("UDP",
                               JUSTFLOAT_UDP_TARGET_IP,
                               JUSTFLOAT_UDP_TARGET_PORT,
                               JUSTFLOAT_UDP_LOCAL_PORT) != 0)
    {
        return 2;
    }
#endif
    return 0;
}

void Float_to_Byte(float f, unsigned char byte[])
{
    FloatLongType fl;
    fl.fdata = f;
    byte[0] = (unsigned char)fl.ldata;
    byte[1] = (unsigned char)(fl.ldata >> 8);
    byte[2] = (unsigned char)(fl.ldata >> 16);
    byte[3] = (unsigned char)(fl.ldata >> 24);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送单个float类型的数据通过串口发送
// 参数说明     a：要发送的浮点数据
// 返回参数     void
// 使用示例     JustFloat_Test_one(sin(t));
// 备注信息     通过vofa上位机显示波形图
//-------------------------------------------------------------------------------------------------------------------
void JustFloat_Test_one(float a)
{
    const float values[1] = {a};
    justfloat_send_values(values, 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送两个float类型的数据通过串口发送
// 参数说明     a、b：要发送的浮点数据
// 返回参数     void
// 使用示例     JustFloat_Test_two(sin(t),sin(2*t));
// 备注信息     通过vofa上位机显示波形图
//-------------------------------------------------------------------------------------------------------------------
void JustFloat_Test_two(float a, float b)
{
    const float values[2] = {a, b};
    justfloat_send_values(values, 2);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送三个float类型的数据通过串口发送
// 参数说明     a、b、c：要发送的浮点数据
// 返回参数     void
// 使用示例     JustFloat_Test_three(sin(t),sin(2*t),sin(3*t));
// 备注信息     通过vofa上位机显示波形图
//-------------------------------------------------------------------------------------------------------------------
void JustFloat_Test_three(float a, float b, float c)
{
    const float values[3] = {a, b, c};
    justfloat_send_values(values, 3);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送四个float类型的数据通过串口发送
// 参数说明     a、b、c、d：要发送的浮点数据
// 返回参数     void
// 使用示例     JustFloat_Test_four(a, b, c, d);
// 备注信息     通过vofa上位机显示波形图
//-------------------------------------------------------------------------------------------------------------------
void JustFloat_Test_four(float a, float b, float c, float d)
{
    const float values[4] = {a, b, c, d};
    justfloat_send_values(values, 4);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送五个float类型的数据通过串口发送
// 参数说明     a、b、c、d、e：要发送的浮点数据
// 返回参数     void
// 使用示例     JustFloat_Test_five(a, b, c, d, e);
// 备注信息     通过vofa上位机显示波形图
//-------------------------------------------------------------------------------------------------------------------
void JustFloat_Test_five(float a, float b, float c, float d, float e)
{
    const float values[5] = {a, b, c, d, e};
    justfloat_send_values(values, 5);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送六个float类型的数据通过串口发送
// 参数说明     a、b、c、d、e、f：要发送的浮点数据
// 返回参数     void
// 使用示例     JustFloat_Test_six(a, b, c, d, e, f);
// 备注信息     通过vofa上位机显示波形图
//-------------------------------------------------------------------------------------------------------------------
void JustFloat_Test_six(float a, float b, float c, float d, float e, float f)
{
    const float values[6] = {a, b, c, d, e, f};
    justfloat_send_values(values, 6);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送七个float类型的数据通过串口发送
// 参数说明     a、b、c、d、e、f、g：要发送的浮点数据
// 返回参数     void
// 使用示例     JustFloat_Test_seven(a, b, c, d, e, f, g);
// 备注信息     通过vofa上位机显示波形图
//-------------------------------------------------------------------------------------------------------------------
void JustFloat_Test_seven(float a, float b, float c, float d, float e, float f, float g)
{
    const float values[7] = {a, b, c, d, e, f, g};
    justfloat_send_values(values, 7);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送八个float类型的数据通过串口发送
// 参数说明     a、b、c、d、e、f、g、h：要发送的浮点数据
// 返回参数     void
// 使用示例     JustFloat_Test_eight(a, b, c, d, e, f, g, h);
// 备注信息     通过vofa上位机显示波形图
//-------------------------------------------------------------------------------------------------------------------
void JustFloat_Test_eight(float a, float b, float c, float d, float e, float f, float g, float h)
{
    const float values[8] = {a, b, c, d, e, f, g, h};
    justfloat_send_values(values, 8);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送九个float类型的数据通过串口发送
// 参数说明     a、b、c、d、e、f、g、h、i：要发送的浮点数据
// 返回参数     void
// 使用示例     JustFloat_Test_nine(a, b, c, d, e, f, g, h, i);
// 备注信息     通过vofa上位机显示波形图
//-------------------------------------------------------------------------------------------------------------------
void JustFloat_Test_nine(float a, float b, float c, float d, float e, float f, float g, float h, float i)
{
    const float values[9] = {a, b, c, d, e, f, g, h, i};
    justfloat_send_values(values, 9);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送十个float类型的数据通过串口发送
// 参数说明     a、b、c、d、e、f、g、h、i、j：要发送的浮点数据
// 返回参数     void
// 使用示例     JustFloat_Test_ten(a, b, c, d, e, f, g, h, i, j);
// 备注信息     通过vofa上位机显示波形图
//-------------------------------------------------------------------------------------------------------------------
void JustFloat_Test_ten(float a, float b, float c, float d, float e, float f, float g, float h, float i, float j)
{
    const float values[10] = {a, b, c, d, e, f, g, h, i, j};
    justfloat_send_values(values, 10);
}
