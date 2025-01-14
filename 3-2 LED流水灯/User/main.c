#include "stm32f10x.h"  // Device header

//计数个数
int f_us=9;
int f_ms=9000;

void delay_init(void)
{
	//设置外部源时钟 72MHZ/8=9MHZ
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//9MHZ
}

//最多能计16,777,216个数，延时最多延时1,864,135us
int delay_us(int nus)//1us计9个数，500us计500*9
{
	u32 temp;
	//重装载寄存器赋值
	SysTick->LOAD = nus*f_us-1;
	
	//计数器赋值为0，防止误差,只有VAL为0时，重装载寄存器才会赋值给它
	SysTick->VAL=0X00;
	
	//使能定时器
	SysTick->CTRL|=(0X01<<0);
	
	do
	{
		temp=SysTick->CTRL;
	}//判断定时器是否开启，以及定时器是否计算完毕
	while((temp&(0x01<<0))&&(!(temp&(0x01<<16))));
	
	//关闭定时器
	SysTick->CTRL&=~(0X01<<0);
}

//最多能计16,777,216个数，延时最多延时1,864ms
int delay_ms(int nms)
{
	u32 temp;
	//重装载寄存器赋值
	SysTick->LOAD = nms*f_ms-1;
	
	//计数器赋值为0，防止误差,只有VAL为0时，重装载寄存器才会赋值给它
	SysTick->VAL=0X00;
	
	//使能定时器
	SysTick->CTRL|=(0X01<<0);
	
	do
	{
		temp=SysTick->CTRL;
	}//判断定时器是否开启，以及定时器是否计算完毕
	while((temp&(0x01<<0))&&(!(temp&(0x01<<16))));
	
	//关闭定时器
	SysTick->CTRL&=~(0X01<<0);
}

int delay_s(int ns)
{
	int n;
	for(n=0;n<ns;n++)
	{
		delay_ms(1000);
	}
}




int main(){
	
	//使用rcc开启gpio的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	//使用 `GPIO_Init` 初始化
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	

	
	while(1)
	{
		GPIO_Write(GPIOA, ~0X0001);//0000 0000 0000 0001取反
		delay_ms(500);
		GPIO_Write(GPIOA, ~0X0002);
		delay_ms(500);
		GPIO_Write(GPIOA, ~0X0004);
		delay_ms(500);
		GPIO_Write(GPIOA, ~0X0008);
		delay_ms(500);
		GPIO_Write(GPIOA, ~0X0010);
		delay_ms(500);
		GPIO_Write(GPIOA, ~0X0020);
		delay_ms(500);
		GPIO_Write(GPIOA, ~0X0040);
		delay_ms(500);
		GPIO_Write(GPIOA, ~0X0080);
		delay_ms(500);
	}
}
