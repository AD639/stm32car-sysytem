#include "led_key.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "delay.h"

/*
 * LED初始化 - PA8作为LED输出（已禁用）
 */
void LED_Init(void)
{
    // PA8 LED功能已禁用，不进行初始化
}

/*
 * 点亮LED（已禁用）
 */
void LED_On(void)
{
    // PA8 LED功能已禁用
}

/*
 * 熄灭LED（已禁用）
 */
void LED_Off(void)
{
    // PA8 LED功能已禁用
}

/*
 * 模式切换指示灯初始化 - PA11和PA12作为输出
 */
void Mode_Indicator_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin   = GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    GPIO_ResetBits(GPIOA, GPIO_Pin_11 | GPIO_Pin_12);  // 初始熄灭
}

/*
 * 模式切换时PA11和PA12快速闪烁一秒钟
 */
void Mode_Switch_Blink(void)
{
    int i;
    for (i = 0; i < 10; i++)  // 闪烁10次，每次100ms
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_11 | GPIO_Pin_12);    // 点亮
        Delay_ms(50);
        GPIO_ResetBits(GPIOA, GPIO_Pin_11 | GPIO_Pin_12);  // 熄灭
        Delay_ms(50);
    }
}

/*
 * K2按键初始化 - PB1作为按键输入，下降沿中断
 */
void KEY_K2_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin   = GPIO_Pin_1;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;       // 上拉输入
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    /* EXTI1 连接到 PB1 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

    EXTI_InitTypeDef exti;
    exti.EXTI_Line    = EXTI_Line1;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿触发
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = EXTI1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}