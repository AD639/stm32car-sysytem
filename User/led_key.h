#ifndef __LED_KEY_H
#define __LED_KEY_H

#include "stm32f10x.h"

/* LED和按键函数声明 */
void LED_Init(void);
void LED_On(void);
void LED_Off(void);
void KEY_K2_Init(void);

/* 模式切换指示灯函数声明 */
void Mode_Indicator_Init(void);
void Mode_Switch_Blink(void);

#endif