#include "stm32f10x.h"  // Device header
#include "delay.h"
#include "OLED.h"
#include "PWM.h"

uint8_t i;

int main(){
	

	OLED_Init();
	PWM_Init();
	delay_init();
	
	while(1)
	{
		for(i=0; i<=100; i++)
		{
			PWM_SetCompare1(i);
			delay_ms(10);
		}
		for(i=0; i<=100; i++)
		{
			PWM_SetCompare1(100-i);
			delay_ms(10);
		}
	}
}
