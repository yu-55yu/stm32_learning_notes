#include "stm32f10x.h"  // Device header


int main(){
	
	//使用rcc开启gpio的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	//使用 `GPIO_Init` 初始化
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//使用输出或输入函数控制gpio口
	//GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	//GPIO_SetBits(GPIOA, GPIO_Pin_0);
	GPIO_WriteBit(GPIOA, GPIO_Pin_0,Bit_RESET);

	
	while(1)
	{
		
	}
}
