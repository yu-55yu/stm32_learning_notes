#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h" 

void delay_init(void);
void delay_us(int nus); // 延时微秒
void delay_ms(int nms); // 延时毫秒
void delay_s(int ns);   // 延时秒

#endif /* __DELAY_H */
