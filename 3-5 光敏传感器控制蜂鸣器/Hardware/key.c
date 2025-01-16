#include "stm32f10x.h"                  // Device header
#include "delay.h"


//初始化
void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//输入情况下speed其实没有用，不过写上没事
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


//没按下0，1按下1，11按下2
uint8_t Key_GetNum(void)
{
	uint8_t Keynum = 0;
	if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1))
	{
		delay_ms(20);
		while(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1));
			delay_ms(20);
		Keynum = 1;
	}
	if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11))
	{
		delay_ms(20);
		while(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11));
			delay_ms(20);
		Keynum = 2;
	}
	return Keynum;
}



