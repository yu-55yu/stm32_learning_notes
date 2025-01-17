#include "stm32f10x.h"                  // Device header

int16_t Encoder_Count;

void Encoder_Init(void)
{
	//GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//AFIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource0);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource1);

	//配置外部中断
	//将EXTI的第14个线路配置成中断模式，下降沿触发
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line1 | EXTI_Line0;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);
	
	//配置NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);
	
}


void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) == SET)
	{
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==0)
		{
			Encoder_Count++;
		}
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==0)
		{
			Encoder_Count--;
		}
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}
int16_t EnCoder_Get(void)
{
	uint16_t Temp;
	Temp = Encoder_Count;
	Encoder_Count = 0;
	return Temp;
}









