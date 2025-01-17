#include "stm32f10x.h"  // Device header
#include "delay.h"
#include "OLED.h"
#include "Encoder.h"

int16_t NUM; 
int main(){
	
	OLED_Init();
	Encoder_Init();
	
	OLED_ShowString(1,1,"NUM:");

	

	while(1)
	{
		NUM +=EnCoder_Get();
		OLED_ShowSignedNum(2,1,NUM,5);
	}
}
