#include "motor_control.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

/* 全局变量 */
extern volatile CarState_t Car_State;

/*
 * TIM2 PWM初始化 - 控制电机1和电机2
 */
void TIM2_PWM_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    TIM_TimeBaseInitTypeDef tb;
    tb.TIM_Prescaler     = 720 - 1;           // 72MHz / 720 = 100kHz
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    tb.TIM_Period        = PWM_MAX - 1;      // 100k / 1000 = 100Hz
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tb);

    TIM_OCInitTypeDef oc;
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = 0;

    TIM_OC1Init(TIM2, &oc);
    TIM_OC2Init(TIM2, &oc);
    TIM_OC3Init(TIM2, &oc);
    TIM_OC4Init(TIM2, &oc);

    TIM_Cmd(TIM2, ENABLE);
}

/*
 * TIM4 PWM初始化 - 控制电机3和电机4
 */
void TIM4_PWM_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    TIM_TimeBaseInitTypeDef tb;
    tb.TIM_Prescaler     = 720 - 1;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    tb.TIM_Period        = PWM_MAX - 1;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &tb);

    TIM_OCInitTypeDef oc;
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = 0;

    TIM_OC1Init(TIM4, &oc);
    TIM_OC2Init(TIM4, &oc);
    TIM_OC3Init(TIM4, &oc);
    TIM_OC4Init(TIM4, &oc);

    TIM_Cmd(TIM4, ENABLE);
}

/*
 * 电机1速度控制
 * dir 方向 (1:正转, 0:反转)
 * speed 速度值 (0-PWM_MAX)
 */
void Motor1_SetSpeed(uint8_t dir, uint16_t speed)
{
    if (speed > PWM_MAX) speed = PWM_MAX;
    if (dir) { // 正转
        TIM_SetCompare1(TIM2, speed);
        TIM_SetCompare2(TIM2, 0);
    } else {   // 反转
        TIM_SetCompare1(TIM2, 0);
        TIM_SetCompare2(TIM2, speed);
    }
}

/*
 * 电机2速度控制
 * dir 方向 (1:正转, 0:反转)
 * speed 速度值 (0-PWM_MAX)
 */
void Motor2_SetSpeed(uint8_t dir, uint16_t speed)
{
    if (speed > PWM_MAX) speed = PWM_MAX;
    if (dir) { // 正转
        TIM_SetCompare4(TIM2, speed);
        TIM_SetCompare3(TIM2, 0);
    } else {   // 反转
        TIM_SetCompare4(TIM2, 0);
        TIM_SetCompare3(TIM2, speed);
    }
}

/*
 * 电机3速度控制
 * dir 方向 (1:正转, 0:反转)
 * speed 速度值 (0-PWM_MAX)
 */
void Motor3_SetSpeed(uint8_t dir, uint16_t speed)
{
    if (speed > PWM_MAX) speed = PWM_MAX;
    if (dir) { // 正转
        TIM_SetCompare2(TIM4, speed);
        TIM_SetCompare1(TIM4, 0);
    } else {   // 反转
        TIM_SetCompare2(TIM4, 0);
        TIM_SetCompare1(TIM4, speed);
    }
}

/*
 * 电机4速度控制
 * dir 方向 (1:正转, 0:反转)
 * speed 速度值 (0-PWM_MAX)
 */
void Motor4_SetSpeed(uint8_t dir, uint16_t speed)
{
    if (speed > PWM_MAX) speed = PWM_MAX;
    if (dir) {
        TIM_SetCompare3(TIM4, speed);
        TIM_SetCompare4(TIM4, 0);
    } else {
        TIM_SetCompare3(TIM4, 0);
        TIM_SetCompare4(TIM4, speed);
    }
}

/*
 * 小车停止
 */
void Car_Stop(void)
{
    Motor1_SetSpeed(1, 0);
    Motor2_SetSpeed(1, 0);
    Motor3_SetSpeed(1, 0);
    Motor4_SetSpeed(1, 0);
    Car_State = CAR_STATE_STOP;
}

/*
 * 小车前进
 * spd 速度值
 */
void Car_Forward(uint16_t spd)
{
    Motor1_SetSpeed(1, spd);
    Motor2_SetSpeed(1, spd);
    Motor3_SetSpeed(1, spd);
    Motor4_SetSpeed(1, spd);
    Car_State = CAR_STATE_FORWARD;
}

/*
 * 小车后退
 * spd 速度值
 */
void Car_Backward(uint16_t spd)
{
    Motor1_SetSpeed(0, spd);
    Motor2_SetSpeed(0, spd);
    Motor3_SetSpeed(0, spd);
    Motor4_SetSpeed(0, spd);
    Car_State = CAR_STATE_BACKWARD;
}

/*
 * 小车左平移
 * spd 速度值
 */
void Car_TransLeft(uint16_t spd)
{
    Motor1_SetSpeed(1, spd);
    Motor2_SetSpeed(0, spd);
    Motor3_SetSpeed(1, spd);
    Motor4_SetSpeed(0, spd);
    Car_State = CAR_STATE_LEFT;
}

/*
 * 小车右平移
 * spd 速度值
 */
void Car_TransRight(uint16_t spd)
{
    Motor1_SetSpeed(0, spd);
    Motor2_SetSpeed(1, spd);
    Motor3_SetSpeed(0, spd);
    Motor4_SetSpeed(1, spd);
    Car_State = CAR_STATE_RIGHT;
}

/*
 * 小车顺时针原地转
 * spd 速度值
 */
void Car_TurnRight(uint16_t spd)
{
    Motor1_SetSpeed(1, spd);
    Motor2_SetSpeed(0, spd);
    Motor3_SetSpeed(0, spd);
    Motor4_SetSpeed(1, spd);
    Car_State = CAR_STATE_TURN_RIGHT;
}

/*
 * 小车逆时针原地转
 * spd 速度值
 */
void Car_TurnLeft(uint16_t spd)
{
    Motor1_SetSpeed(0, spd);
    Motor2_SetSpeed(1, spd);
    Motor3_SetSpeed(1, spd);
    Motor4_SetSpeed(0, spd);
    Car_State = CAR_STATE_TURN_LEFT;
}