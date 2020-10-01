/*
 * STM32F407XX_GPIO_DRIVER.c
 *
 *  Created on: 30-Sep-2020
 *      Author: Training
 */
#include "stm32f407xx_gpio_driver.h"

void PeriClkCtrl(GPIO_REGDEF_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi==ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLCK_EN();
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLCK_EN();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLCK_EN();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLCK_EN();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLCK_EN();
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLCK_EN();
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLCK_EN();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLCK_EN();
		}
		else if(pGPIOx==GPIOI)
		{
			GPIOI_PCLCK_EN();
		}
	}
	else
	{ //disable options
	}

}

void GPIO_init(GPIO_Handle_t *GPIO_Handle)
{
	//Init Mode
	uint32_t temp=0;
	temp=GPIO_Handle->GPIO_PinConfig.GPIO_PinMode<<(2*GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	(GPIO_Handle->pGPIOx->MODER) &=~(0x3<<GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	(GPIO_Handle->pGPIOx->MODER) |=temp;


	//Config speed
    temp=0;
    temp=(GPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed<<(2*GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
    (GPIO_Handle->pGPIOx->OSPEEDR) &=~(0x3<<GPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed);
    (GPIO_Handle->pGPIOx->OSPEEDR) |=temp;

	//config pull up or pull down
    temp=0;
    temp=(GPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
    (GPIO_Handle->pGPIOx->PUPDR) &=~(0x3<<GPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl);
    GPIO_Handle->pGPIOx->PUPDR |=temp;

    //config o/p type
    temp=0;
    temp=(GPIO_Handle->GPIO_PinConfig.GPIO_PinOPType<<(GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
    (GPIO_Handle->pGPIOx->OTYPER) &=~(0x1<<GPIO_Handle->GPIO_PinConfig.GPIO_PinOPType);
    GPIO_Handle->pGPIOx->OTYPER |=temp;

    //config alt funct
    temp=0;

    if(GPIO_Handle->GPIO_PinConfig.GPIO_PinMode==GPIO_Mode_AF)
    {
    	uint8_t temp1=0,temp2=0;
    	temp1=(GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)/8;
    	temp2=(GPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)%8;
    	GPIO_Handle->pGPIOx->AFR[temp1] &=~(0xF<<temp2);
    	GPIO_Handle->pGPIOx->AFR[temp1]=(GPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*temp2));
    }
}

void GPIO_Dinit(GPIO_REGDEF_t *pGPIOx)
{
	if(pGPIOx==GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx==GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx==GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx==GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx==GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx==GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx==GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx==GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx==GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

uint8_t ReadFromIpPin(GPIO_REGDEF_t *pGPIOx, uint8_t PinNo)
{
	uint8_t val=0;
	val=(uint8_t)((pGPIOx->IDR>>PinNo)&(0x00000001));
	return val;
}

uint8_t GPIO_ReadFromIpPort(GPIO_REGDEF_t *pGPIOx)
{
	uint8_t val=0;
	val=(uint8_t)(pGPIOx->IDR);
	return val;
}

void GPIO_WriteToOpPin(GPIO_REGDEF_t *pGPIOx,uint8_t PinNo, uint16_t value)
{
	if(value==GPIO_PinSet)
	{
		pGPIOx->ODR |=(1<<PinNo);
	}
	else
	{
		pGPIOx->ODR &=~(1<<PinNo);
	}
}
void GPIO_WriteToOpPort(GPIO_REGDEF_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR=value;
}
void GPIO_ToggleOpPin(GPIO_REGDEF_t *pGPIOx, uint16_t PinNo)
{
	pGPIOx->ODR ^=(1<<PinNo);
}

