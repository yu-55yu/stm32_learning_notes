#include "stm32f10x.h"                  // Device header

void AD_Init(void)
{
	//1、开启ADC\GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	//配置adcclk分频器
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	//2、配置GPIO（模拟输入）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
	//3、配置多路开关（接入规则组）(可选择开启中断)
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_55Cycles5);
	
	
	//4、配置ADC转换器（结构体）
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;			//单次转换
	ADC_InitStructure.ADC_ScanConvMode =DISABLE;				//非扫描模式
	ADC_InitStructure.ADC_NbrOfChannel =1;//在非扫描模式下写多少都是1
	ADC_Init(ADC1,&ADC_InitStructure);
	
	
	//5.ADCCMD
	ADC_Cmd(ADC1,ENABLE);
	
	//对ADC校准
	ADC_ResetCalibration(ADC1);//复位
	while(ADC_GetResetCalibrationStatus(ADC1)==SET);//1
	ADC_StartCalibration(ADC1);//开始校准
	while(ADC_GetCalibrationStatus(ADC1)==SET);
	 
}

uint16_t AD_GetValue(void)
{
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)== RESET);//(1/12MHz)*(55+12.5)~5.6us
	return ADC_GetConversionValue(ADC1);
}

