/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */


#include<stdio.h>
extern void initialise_monitor_handles(void);

int main(void)
{
	int num1=10,num2=13,sum=0;
	sum=num1+num2;
	initialise_monitor_handles();   //Semi Hosting
		printf("Sum = %d \n",sum);
    /* Loop forever */
	for(;;);
}
