#include "stm32f10x.h"  // Device header
#include "delay.h"


int main(){
	
	//使用RCC开启GPIO的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	//使用 `GPIO_Init` 初始化
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	
	
	while(1)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		delay_ms(100);
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
		delay_ms(100);
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		delay_ms(100);
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
		delay_ms(700);
		
	}
}
