#include "stm32f10x.h"  // Device header
#include "delay.h"
#include "OLED.h"
#include "Timer.h"
#include "Encoder.h"

int16_t Speed;

int main(){
	
	OLED_Init();
	Timer_Init();
	Encode_Init();
	
	OLED_ShowString(1,1,"Cnt:");
	
	while(1)
	{
		OLED_ShowSignedNum(1,5,Speed,5);
	}
}




void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
	{
		Speed = Encoder_Get();
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}

