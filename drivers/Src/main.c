/*
 * main.c
 *
 *  Created on: 10-Sep-2022
 *      Author: amogh
 */


#include"L47XX.h"

int main(void)
{
return(0);
}

void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(0);
}
