#include "stm32f10x.h"  // Device header
#include "delay.h"
#include "OLED.h"
#include "PWM.h"
#include "IC.h"

uint8_t i;

int main(){
	
	OLED_Init();
	PWM_Init();
	delay_init();
	IC_Init();
	OLED_ShowString(1,1,"Freq:00000Hz");
	
	PWM_SetPrescaler(720-1);	//Freq=72M/PSC+1/100
	PWM_SetCompare1(50);		//Duty=CCR/100;
	
	while(1)
	{
		OLED_ShowNum(1,6,IC_GetFreq(),5);
	}
}
