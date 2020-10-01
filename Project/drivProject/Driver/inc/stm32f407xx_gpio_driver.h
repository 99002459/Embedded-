/*
 * STM32F407XX_GPIO_DRIVER.H
 *
 *  Created on: 30-Sep-2020
 *      Author: Training
 */
#include <stdint.h>
#include "stm32f407xx.h"
#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


typedef struct{
	uint8_t GPIO_PinNumber;  		//@This refers to the various pin numbers of GPIOx
	uint8_t GPIO_PinMode;	 		//@This refers to the various modes of GPIOx
	uint8_t GPIO_PinSpeed;	 		//@This refers to the various pin speeds of GPIOx
	uint8_t GPIO_PinOPType;			//@This refers to the various pin output type of GPIOx
	uint8_t GPIO_PinPuPdControl;	//@This refers to the pin pull up and pull down control of GPIOx
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct{
	GPIO_REGDEF_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*
 * @This refers to the various pin numbers of GPIOx
 */
#define GPIO_Pin_No_0		0
#define GPIO_Pin_No_1		1
#define GPIO_Pin_No_2		2
#define GPIO_Pin_No_3		3
#define GPIO_Pin_No_4		4
#define GPIO_Pin_No_5		5
#define GPIO_Pin_No_6		6
#define GPIO_Pin_No_7		7
#define GPIO_Pin_No_8		8
#define GPIO_Pin_No_9		9
#define GPIO_Pin_No_10		10
#define GPIO_Pin_No_11		11
#define GPIO_Pin_No_12		12
#define GPIO_Pin_No_13		13
#define GPIO_Pin_No_14		14
#define GPIO_Pin_No_15		15

  /*
   *@This refers to the various modes of GPIOx
   */
#define GPIO_Mode_In		0
#define GPIO_Mode_Out		1
#define GPIO_Mode_AF		2
#define GPIO_Mode_ANLG		3

  /*
   * @This refers to the various pin speeds of GPIOx
   */
#define GPIO_Speed_Low		0
#define GPIO_Speed_Med		1
#define GPIO_Speed_High		2
#define GPIO_Speed_VHigh	3

  /*
   * This refers to the various pin output type of GPIOx
   */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

  /*
   * @This refers to the pin pull up and pull down control of GPIOx
   */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2
#define GPIO_PIN_Res		3

void GPIO_init(GPIO_Handle_t *GPIO_Handle);
void GPIO_Dinit(GPIO_REGDEF_t *pGPIOx);

void PeriClkCtrl(GPIO_REGDEF_t *pGPIOx,uint8_t EnOrDi);
uint8_t ReadFromIpPin(GPIO_REGDEF_t *pGPIOx, uint8_t PinNo);
uint8_t GPIO_ReadFromIpPort(GPIO_REGDEF_t *pGPIOx);
void GPIO_WriteToOpPin(GPIO_REGDEF_t *pGPIOx,uint8_t PinNo, uint16_t value);
void GPIO_WriteToOpPort(GPIO_REGDEF_t *pGPIOx, uint16_t value);
void GPIO_ToggleOpPin(GPIO_REGDEF_t *pGPIOx, uint16_t PinNo);
#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
