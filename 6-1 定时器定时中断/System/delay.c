#include "delay.h"

// 计数个数
int f_us = 9;  // 每微秒需要的计数个数
int f_ms = 9000; // 每毫秒需要的计数个数

void delay_init(void)
{
    // 设置外部源时钟 72MHz / 8 = 9MHz
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); // 9MHz
}

// 延时微秒
void delay_us(int nus)
{
    uint32_t temp;
    // 重装载寄存器赋值
    SysTick->LOAD = nus * f_us - 1;

    // 计数器赋值为0，防止误差，只有VAL为0时，重装载寄存器才会赋值给它
    SysTick->VAL = 0x00;

    // 使能定时器
    SysTick->CTRL |= (0x01 << 0);

    do
    {
        temp = SysTick->CTRL;
    } while ((temp & (0x01 << 0)) && (!(temp & (0x01 << 16)))); // 判断定时器是否开启，以及定时器是否计算完毕

    // 关闭定时器
    SysTick->CTRL &= ~(0x01 << 0);
}

// 延时毫秒
void delay_ms(int nms)
{
    uint32_t temp;
    // 重装载寄存器赋值
    SysTick->LOAD = nms * f_ms - 1;

    // 计数器赋值为0，防止误差，只有VAL为0时，重装载寄存器才会赋值给它
    SysTick->VAL = 0x00;

    // 使能定时器
    SysTick->CTRL |= (0x01 << 0);

    do
    {
        temp = SysTick->CTRL;
    } while ((temp & (0x01 << 0)) && (!(temp & (0x01 << 16)))); // 判断定时器是否开启，以及定时器是否计算完毕

    // 关闭定时器
    SysTick->CTRL &= ~(0x01 << 0);
}

// 延时秒
void delay_s(int ns)
{
    int n;
    for (n = 0; n < ns; n++)
    {
        delay_ms(1000); // 延时1000毫秒，即1秒
    }
}

