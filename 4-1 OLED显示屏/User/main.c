#include "stm32f10x.h"  // Device header
#include "delay.h"
#include "OLED.h"

int main(){
	
	OLED_Init();
	
	OLED_ShowChar(1,1,'A');
	OLED_ShowString(1,3,"helloword");
	OLED_ShowSignedNum(2,1,12345,5);
	OLED_ShowNum(2,7,12345,5);
	OLED_ShowBinNum(3,1,0xaa55,16);
	OLED_ShowHexNum(4,1,0XAA55,4);
	OLED_Clear();

	

	while(1)
	{
		
	}
}
