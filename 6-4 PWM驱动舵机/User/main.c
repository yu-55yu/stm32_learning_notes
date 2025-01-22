#include "stm32f10x.h"  // Device header
#include "delay.h"
#include "OLED.h"
#include "PWM.h"

uint8_t i;

int main(){

	OLED_Init();
	PWM_Init();
	PWM_SetCompare2(500);
	
	while(1)
	{
	}
}
