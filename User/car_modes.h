#ifndef __CAR_MODES_H
#define __CAR_MODES_H

#include "stm32f10x.h"
#include "car_config.h"

/* 工作模式函数声明 */
void Mode1_MotorTest(void);
void Mode2_IRTracking(void);
void Mode3_US_Follow(void);

/* 命令解析函数声明 */
void Car_ParseUSART_CMD(void);

/* 状态重置函数声明 */
void Reset_IR_Tracking_State(void);
void Reset_Motor_Test_State(void);

/* WiFi响应发送函数声明 */
void WiFi_SendResponse(const char *response);

#endif