#include "wifi_control.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "delay.h"
#include <string.h>
#include <stdio.h>

/* 外部变量声明 */
extern volatile uint8_t serialRxBuf[SERIAL_RX_BUF_LEN];
extern volatile uint16_t serialRxHead, serialRxTail;
extern volatile uint8_t WIFI_Connected;

/*
 * 串口发送单个字符
 * c 要发送的字符
 */
void Usart1_SendChar(char c)
{
    USART_SendData(USART1, (uint16_t)c);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

/*
 * 串口发送字符串
 * s 要发送的字符串
 */
void Usart1_SendString(const char *s)
{
    while (*s)
    {
        Usart1_SendChar(*s++);
    }
}

/*
 * 串口发送单个字符(重载)
 * ch 要发送的字符
 */
void USART1_SendChar(uint8_t ch)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, ch);
}

/*
 * 串口发送字符串(重载) - 修改为同时发送到串口和WiFi客户端
 * s 要发送的字符串
 */
void USART1_SendString(const char *s)
{
    // 发送到串口（电脑端）
    const char *p = s;
    while(*p) USART1_SendChar((uint8_t)*p++);
    
    // 如果WiFi已连接，同时发送到WiFi客户端（手机端）
    if (WIFI_Connected)
    {
        WiFi_SendToClient(s);
    }
}

/*
 * 通过WiFi TCP连接发送数据到客户端
 * data 要发送的数据
 */
void WiFi_SendToClient(const char *data)
{
    if (!WIFI_Connected) return;
    
    char cmd[256];
    int len = strlen(data);
    
    // 构造AT+CIPSEND命令
    sprintf(cmd, "AT+CIPSEND=0,%d", len);
    
    // 直接发送CIPSEND命令到串口，避免递归调用
    const char *p = cmd;
    while(*p) USART1_SendChar((uint8_t)*p++);
    USART1_SendChar('\r');
    USART1_SendChar('\n');
    
    // 等待">"提示符
    Delay_ms(50);
    
    // 直接发送实际数据到串口
    p = data;
    while(*p) USART1_SendChar((uint8_t)*p++);
    
    // 等待发送完成
    Delay_ms(10);
}

/*
 * WiFi响应发送函数 - 专门用于发送回复消息
 * response 要发送的回复消息
 */
void WiFi_SendResponse(const char *response)
{
    // 发送到串口（电脑端）
    const char *p = response;
    while(*p) USART1_SendChar((uint8_t)*p++);
    
    // 如果WiFi已连接，发送到WiFi客户端（手机端）
    WiFi_SendToClient(response);
}

/*
 * USART1初始化
 * baud 波特率
 */
void USART1_Init(uint32_t baud)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef gpio;
    /* PA9  TX  复用推挽输出 */
    gpio.GPIO_Pin   = GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PA10 RX 浮空输入 */
    gpio.GPIO_Pin   = GPIO_Pin_10;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    USART_InitTypeDef us;
    us.USART_BaudRate            = baud;
    us.USART_WordLength          = USART_WordLength_8b;
    us.USART_StopBits            = USART_StopBits_1;
    us.USART_Parity              = USART_Parity_No;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    us.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &us);

    /* 开启接收中断 */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART1, ENABLE);
}

/*
 * 检查串口缓冲区是否有数据
 * 返回: 1:有数据, 0:无数据
 */
int SerialRxAvailable(void)
{
    return (serialRxHead != serialRxTail);
}

/*
 * 从串口缓冲区读取一个字节
 * 返回: 读取的字节，-1表示无数据
 */
int SerialReadChar(void)
{
    if (serialRxHead == serialRxTail) return -1;
    uint8_t c = serialRxBuf[serialRxTail++];
    if (serialRxTail >= SERIAL_RX_BUF_LEN) serialRxTail = 0;
    return c;
}

/*
 * 清空串口接收缓冲区
 */
void SerialClearBuffer(void)
{
    serialRxHead = serialRxTail = 0;
}

/*
 * 发送AT指令并等待响应
 * cmd 要发送的AT指令
 * ack 期望的响应字符串
 * timeout_ms 超时时间(毫秒)
 * 返回: 1:成功, 0:失败
 */
uint8_t ESP8266_SendCmd(const char *cmd, const char *ack, uint32_t timeout_ms)
{
    char rxbuf[128];
    uint16_t idx = 0;
    memset(rxbuf, 0, sizeof(rxbuf));

    Usart1_SendString(cmd);
    Usart1_SendString("\r\n");

    while (timeout_ms--)
    {
        if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
        {
            char c = (char)USART_ReceiveData(USART1);
            if (idx < sizeof(rxbuf) - 1)
            {
                rxbuf[idx++] = c;
                rxbuf[idx]   = '\0';
            }
            if (strstr(rxbuf, ack) != NULL)
            {
                return 1;   // 成功
            }
        }
        Delay_ms(1);
    }
    return 0;               // 超时失败
}

/*
 * ESP8266初始化 - 配置为AP模式并开启TCP服务器
 * 返回: 1:成功, 0:失败
 */
uint8_t ESP8266_Init(void)
{
    uint8_t ok = 1;
    ok &= ESP8266_SendCmd("AT", "OK", 500);
    ok &= ESP8266_SendCmd("ATE0", "OK", 500);                         // 关闭回显
    ok &= ESP8266_SendCmd("AT+CWMODE=2", "OK", 500);                  // AP 模式
    ok &= ESP8266_SendCmd("AT+CWSAP=\"ESP8266AP_lisixian\",\"12345678\",5,3", "OK", 2000);
    ok &= ESP8266_SendCmd("AT+CIPMUX=1", "OK", 500);                  // 多连接
    ok &= ESP8266_SendCmd("AT+CIPSERVER=1,8080", "OK", 2000);         // 开 TCP Server
    return ok;
}