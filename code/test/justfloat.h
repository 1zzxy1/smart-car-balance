/*
 * justfloat.h
 *
 *  Created on: 2025年4月29日
 *      Author: suiyungui
 */

#ifndef CODE_JUSTFLOAT_H_
#define CODE_JUSTFLOAT_H_

#include "zf_common_headfile.h"

//==============================================================================
// 调试通道选择 (二选一)
//==============================================================================
#define JUSTFLOAT_USE_WIFI_SPI      0   // 1: 使用WiFi SPI模块  0: 使用无线串口

//==============================================================================
// WiFi SPI 配置参数 (仅当 JUSTFLOAT_USE_WIFI_SPI = 1 时有效)
//==============================================================================
#if JUSTFLOAT_USE_WIFI_SPI
    #define JUSTFLOAT_WIFI_SSID         "exin"  // WiFi名称
    #define JUSTFLOAT_WIFI_PASSWORD     "88888888"              // WiFi密码
    #define JUSTFLOAT_UDP_TARGET_IP     "192.168.137.1"     // VOFA+上位机IP
    #define JUSTFLOAT_UDP_TARGET_PORT   "1347"              // VOFA+监听端口
    #define JUSTFLOAT_UDP_LOCAL_PORT    "6666"              // 本地端口
#endif

//==============================================================================
// 公共API
//==============================================================================
void Float_to_Byte(float f, unsigned char byte[]);
void JustFloat_Test_one(float a);
void JustFloat_Test_two(float a, float b);
void JustFloat_Test_three(float a, float b, float c);
void JustFloat_Test_four(float a, float b, float c, float d);
void JustFloat_Test_five(float a, float b, float c, float d, float e);
void JustFloat_Test_six(float a, float b, float c, float d, float e, float f);
void JustFloat_Test_seven(float a, float b, float c, float d, float e, float f, float g);
void JustFloat_Test_eight(float a, float b, float c, float d, float e, float f, float g, float h);
void JustFloat_Test_nine(float a, float b, float c, float d, float e, float f, float g, float h, float i);
void JustFloat_Test_ten(float a, float b, float c, float d, float e, float f, float g, float h, float i, float j);
//==============================================================================
// 初始化函数 (WiFi SPI模式必须调用)
//==============================================================================
uint8 justfloat_init(void);

#endif /* CODE_JUSTFLOAT_H_ */
