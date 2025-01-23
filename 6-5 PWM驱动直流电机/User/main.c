#include "stm32f10x.h"  // Device header
#include "delay.h"
#include "OLED.h"
#include "Motor.h"
#include "key.h"

uint8_t KeyNum;
int8_t Speed;

int main(){
	
	OLED_Init();
	Motor_Init();
	Key_Init();
	Motor_SetSpeed(-50);
	
	OLED_ShowString(1,1,"Speed:");
	
	while(1)
	{
		KeyNum= Key_GetNum();
		if(KeyNum==1)
		{
			Speed+=20;
			if(Speed>100)
			{
				Speed=-100;
			}
		}
		Motor_SetSpeed(Speed);
		OLED_ShowSignedNum(1,7,Speed,3);
	}
}
