/*
 * stm32f407xx.h
 *
 *  Created on: Sep 29, 2020
 *      Author: Training
 */
#include<stdint.h>
        ////base address of the MEMORY
#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#define FLASH_BASE_ADDR 	0x08000000U
#define SRAM1_BASE_ADDR 	0x20000000U
#define SRAM2_BASE_ADDR 	0x20010000U
#define SRAM 				SRAM1_BASE_ADDR
#define ROM_BASE_ADDR 		0x1FFF0000U

          //base address of the bus
#define APB1_BASE_ADDR		0x40000000U
#define APB2_BASE_ADDR		0x40010000U
#define AHB1_BASE_ADDR		0x40020000U
#define AHB2_BASE_ADDR		0x50000000U

		//Register structure of GPIOA peripheral
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];

}GPIO_REGDEF_t;

         //base address of the peripherals HANGING ON AHB1
#define GPIOA_ADDR				(AHB1_BASE_ADDR + 0X00U)
#define GPIOB_ADDR				(AHB1_BASE_ADDR + 0X0400U)
#define GPIOC_ADDR				(AHB1_BASE_ADDR + 0X0800U)
#define GPIOD_ADDR				(AHB1_BASE_ADDR + 0X0C00U)
#define GPIOE_ADDR				(AHB1_BASE_ADDR + 0X1000U)
#define GPIOF_ADDR				(AHB1_BASE_ADDR + 0X1400U)
#define GPIOG_ADDR				(AHB1_BASE_ADDR + 0X1800U)
#define GPIOH_ADDR				(AHB1_BASE_ADDR + 0X1C00U)
#define GPIOI_ADDR				(AHB1_BASE_ADDR + 0X2000U)
#define RCC_BASEADDR			0x40023800

		//GPIO Peripheral definition
#define GPIOA 				((GPIO_REGDEF_t*)GPIOA_ADDR)
#define GPIOB 				((GPIO_REGDEF_t*) GPIOB_ADDR)
#define GPIOC 				((GPIO_REGDEF_t*) GPIOC_ADDR)
#define GPIOD 				((GPIO_REGDEF_t*)GPIOD_ADDR)
#define GPIOE 				((GPIO_REGDEF_t*) GPIOE_ADDR)
#define GPIOF 				((GPIO_REGDEF_t*) GPIOF_ADDR)
#define GPIOG 				((GPIO_REGDEF_t*) GPIOG_ADDR)
#define GPIOH 				((GPIO_REGDEF_t*) GPIOH_ADDR)
#define GPIOI 				((GPIO_REGDEF_t*) GPIOI_ADDR)

		//base address of the peripherals HANGING ON APB1
#define I2C1				0x40005400U
#define I2C2				0x40005800U
#define I2C3				0x40005C00U
#define SPI2				0x40003800U
#define SPI3				0x40003C00U
#define USART2				0x40004400U
#define USART3				0x40004800U


		//base address of the peripherals HANGING ON APB2
#define SPI1				0x40013000U
#define USART1				0x40011000U
#define USART6				0x40011400U
#define EXTI				0x40013C00U
#define SYSCFG				0x40013800U



typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t RESERVED[6];
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;


}RCC_REGDEF_t;

#define RCC			((RCC_REGDEF_t* )RCC_BASEADDR)

/*
 * CLOCK ENABLING
 */
#define GPIOA_PCLCK_EN() (RCC->AHB1ENR |=(1<<0))
#define GPIOB_PCLCK_EN() (RCC->AHB1ENR |=(1<<1))
#define GPIOC_PCLCK_EN() (RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLCK_EN() (RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLCK_EN() (RCC->AHB1ENR |=(1<<4))
#define GPIOF_PCLCK_EN() (RCC->AHB1ENR |=(1<<5))
#define GPIOG_PCLCK_EN() (RCC->AHB1ENR |=(1<<6))
#define GPIOH_PCLCK_EN() (RCC->AHB1ENR |=(1<<7))
#define GPIOI_PCLCK_EN() (RCC->AHB1ENR |=(1<<8))
/*
 * CLOCK DISABLING
 */
#define GPIOA_PCLCK_DIS() (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLCK_DIS() (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLCK_DIS() (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLCK_DIS() (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLCK_DIS() (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLCK_DIS() (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLCK_DIS() (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLCK_DIS() (RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLCK_DIS() (RCC->AHB1ENR &= ~(1<<8))
/*
 * Some Important Macros
 */
#define ENABLE					1
#define DISABLE					0
#define SET						ENABLE
#define RESET					DISBALE
#define GPIO_PinSet				SET
#define GPIO_PinReset			RESET

#define	GPIOA_REG_RESET()		do{(RCC->AHB1RSTR |=(1<<0)); (RCC->AHB1RSTR &=~(1<<0));}while(0)
#define	GPIOB_REG_RESET()		do{(RCC->AHB1RSTR |=(1<<1)); (RCC->AHB1RSTR &=~(1<<1));}while(0)
#define	GPIOC_REG_RESET()		do{(RCC->AHB1RSTR |=(1<<2)); (RCC->AHB1RSTR &=~(1<<2));}while(0)
#define	GPIOD_REG_RESET()		do{(RCC->AHB1RSTR |=(1<<3)); (RCC->AHB1RSTR &=~(1<<3));}while(0)
#define	GPIOE_REG_RESET()		do{(RCC->AHB1RSTR |=(1<<4)); (RCC->AHB1RSTR &=~(1<<4));}while(0)
#define	GPIOF_REG_RESET()		do{(RCC->AHB1RSTR |=(1<<5)); (RCC->AHB1RSTR &=~(1<<5));}while(0)
#define	GPIOG_REG_RESET()		do{(RCC->AHB1RSTR |=(1<<6)); (RCC->AHB1RSTR &=~(1<<6));}while(0)
#define	GPIOH_REG_RESET()		do{(RCC->AHB1RSTR |=(1<<7)); (RCC->AHB1RSTR &=~(1<<7));}while(0)
#define	GPIOI_REG_RESET()		do{(RCC->AHB1RSTR |=(1<<8)); (RCC->AHB1RSTR &=~(1<<8));}while(0)


#endif /* INC_STM32F407XX_H_ */

