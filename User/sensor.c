#include "sensor.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "delay.h"

/* 全局变量 */
extern uint32_t TimeCount;
extern uint8_t MeasureFlag;

/*
 * 红外传感器初始化
 */
void Sensor_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = SENSOR1_PIN | SENSOR2_PIN | SENSOR3_PIN | SENSOR4_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SENSOR_PORT, &GPIO_InitStructure);
}

/*
 * 读取传感器1状态
 * 返回: 传感器状态 (0:检测到黑线, 1:未检测到)
 */
uint8_t Sensor1_Get_State(void) 
{ 
    return GPIO_ReadInputDataBit(SENSOR_PORT, SENSOR1_PIN); 
}

/*
 * 读取传感器2状态
 * 返回: 传感器状态 (0:检测到黑线, 1:未检测到)
 */
uint8_t Sensor2_Get_State(void) 
{ 
    return GPIO_ReadInputDataBit(SENSOR_PORT, SENSOR2_PIN); 
}

/*
 * 读取传感器3状态
 * 返回: 传感器状态 (0:检测到黑线, 1:未检测到)
 */
uint8_t Sensor3_Get_State(void) 
{ 
    return GPIO_ReadInputDataBit(SENSOR_PORT, SENSOR3_PIN); 
}

/*
 * 读取传感器4状态
 * 返回: 传感器状态 (0:检测到黑线, 1:未检测到)
 */
uint8_t Sensor4_Get_State(void) 
{ 
    return GPIO_ReadInputDataBit(SENSOR_PORT, SENSOR4_PIN); 
}

/*
 * 超声波传感器GPIO初始化
 */
void HC_SR04_GPIO_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = ULTRASONIC_TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ULTRASONIC_TRIG_PORT, &GPIO_InitStructure);
    GPIO_WriteBit(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, Bit_RESET);

    GPIO_InitStructure.GPIO_Pin = ULTRASONIC_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(ULTRASONIC_ECHO_PORT, &GPIO_InitStructure);
    GPIO_WriteBit(ULTRASONIC_ECHO_PORT, ULTRASONIC_ECHO_PIN, Bit_RESET);
}

/*
 * 超声波传感器定时器初始化
 */
void HC_SR04_TIM3_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_InternalClockConfig(TIM3);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 10 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM3, DISABLE);
}

/*
 * 超声波测距
 * 返回: 距离值(mm)
 */
float HC_SR04_MeasureDistance(void)
{
    float Distance = 0;
    uint32_t Time_end = 0;

    GPIO_WriteBit(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, Bit_SET);
    Delay_us(15);
    GPIO_WriteBit(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, Bit_RESET);

    while (GPIO_ReadInputDataBit(ULTRASONIC_ECHO_PORT, ULTRASONIC_ECHO_PIN) == 0);
    TimeCount = 0;
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
    while (GPIO_ReadInputDataBit(ULTRASONIC_ECHO_PORT, ULTRASONIC_ECHO_PIN) == 1);
    TIM_Cmd(TIM3, DISABLE);
    Time_end = TimeCount * 10;

    if (Time_end < 24000) Distance = Time_end * 0.34f / 2;
    else Distance = 0;

    MeasureFlag = 1;
    return Distance;
}