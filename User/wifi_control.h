#ifndef __WIFI_CONTROL_H
#define __WIFI_CONTROL_H

#include "stm32f10x.h"
#include "car_config.h"

/* WiFi控制相关函数声明 */
void USART1_Init(uint32_t baud);
void Usart1_SendChar(char c);
void Usart1_SendString(const char *s);
void USART1_SendChar(uint8_t ch);
void USART1_SendString(const char *s);

uint8_t ESP8266_SendCmd(const char *cmd, const char *ack, uint32_t timeout_ms);
uint8_t ESP8266_Init(void);

/* 串口数据处理函数声明 */
int SerialRxAvailable(void);
int SerialReadChar(void);
void SerialClearBuffer(void);

/* WiFi TCP数据发送函数声明 */
void WiFi_SendToClient(const char *data);
void WiFi_SendResponse(const char *response);

#endif