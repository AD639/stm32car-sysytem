#ifndef __SENSOR_H
#define __SENSOR_H

#include "stm32f10x.h"
#include "car_config.h"

/* 红外传感器函数声明 */
void Sensor_Init(void);
uint8_t Sensor1_Get_State(void);
uint8_t Sensor2_Get_State(void);
uint8_t Sensor3_Get_State(void);
uint8_t Sensor4_Get_State(void);

/* 超声波传感器函数声明 */
void HC_SR04_GPIO_Init(void);
void HC_SR04_TIM3_Init(void);
float HC_SR04_MeasureDistance(void);

#endif