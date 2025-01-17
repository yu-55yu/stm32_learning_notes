#ifndef __ENCODER_H
#define __ENCODER_H

void Encoder_Init(void);
void EXTI1_IRQHandler(void);
void EXTI0_IRQHandler(void);
uint16_t EnCoder_Get(void);

#endif