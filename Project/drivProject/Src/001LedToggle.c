/*
 * 001LedToggle.c
 *
 *  Created on: 01-Oct-2020
 *      Author: Training
 */
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void Delay(void)
{
	for(uint32_t i=0;i<0xFFFFF;i++);
}
int main(void)
{
	GPIO_Handle_t GPIOLed;
	GPIOLed.pGPIOx=GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_Pin_No_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode=GPIO_Mode_Out;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_Speed_VHigh;

	PeriClkCtrl(GPIOD,SET);
	GPIO_init(&GPIOLed);
	while(1)
	{
		GPIO_ToggleOpPin(GPIOD, GPIO_Pin_No_12);
		Delay();
	}
	return 0;
}


