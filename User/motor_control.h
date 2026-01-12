#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "stm32f10x.h"
#include "car_config.h"

/* 电机控制函数声明 */
void TIM2_PWM_Init(void);
void TIM4_PWM_Init(void);

void Motor1_SetSpeed(uint8_t dir, uint16_t speed);
void Motor2_SetSpeed(uint8_t dir, uint16_t speed);
void Motor3_SetSpeed(uint8_t dir, uint16_t speed);
void Motor4_SetSpeed(uint8_t dir, uint16_t speed);

/* 小车运动控制函数声明 */
void Car_Stop(void);
void Car_Forward(uint16_t spd);
void Car_Backward(uint16_t spd);
void Car_TransLeft(uint16_t spd);
void Car_TransRight(uint16_t spd);
void Car_TurnRight(uint16_t spd);
void Car_TurnLeft(uint16_t spd);

#endif