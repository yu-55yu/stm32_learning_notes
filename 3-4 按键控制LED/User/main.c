#include "stm32f10x.h"  // Device header
#include "delay.h"
#include "LED.h"
#include "key.h"

uint8_t KeyNum;

int main(){
	
	LED_Init();
	Key_Init();

	while(1)
	{
		KeyNum = Key_GetNum();
		if(KeyNum == 1)
		{
			LED1_TURN();
	
		}
		if(KeyNum ==2)
		{
			LED2_TURN();
	
		}
		
	/*	LED1_ON();
		LED2_OFF();
		delay_ms(500);
		
		LED2_ON();
		LED1_OFF();
		delay_ms(500);*/
	}
}
