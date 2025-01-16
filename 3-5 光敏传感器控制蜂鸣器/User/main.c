#include "stm32f10x.h"  // Device header
#include "delay.h"
#include "Buzzer.h"
#include "LightSensor.h"



int main(){
	
	Buzzer_Init();
	LightSensor_Init();

	while(1)
	{
		if(LightSensor_Get())
		{
			Buzzer_ON();
		}
		else
		Buzzer_OFF();
		
	}
}
