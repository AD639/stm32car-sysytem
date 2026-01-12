#include "delay.h"

/*
 * 毫秒延时函数
 * ms 延时时间(毫秒)
 */
void Delay_ms(uint32_t ms)
{
    /* HCLK = 72MHz, 1ms 需要 72 000 次计数 */
    SysTick->LOAD = 72000 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while (ms--)
    {
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    }
    SysTick->CTRL = 0;
}

/*
 * 毫秒延时函数(重载)
 * ms 延时时间(毫秒)
 */
void delay_ms(uint32_t ms)
{
    uint32_t i;
    SysTick_Config(SystemCoreClock / 1000);
    for(i = 0; i < ms; i++)
    {
        while(!(SysTick->CTRL & (1 << 16)));
    }
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

/*
 * 微秒延时函数
 * us 延时时间(微秒)
 */
void Delay_us(uint32_t us)
{
    uint32_t i, j;
    for (i = 0; i < us; i++)
        for (j = 0; j < 8; j++);
}