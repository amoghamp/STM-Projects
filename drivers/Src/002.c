/*
 * 002.c
 *
 *  Created on: Sep 2, 2022
 *      Author: amogh
 */


#include <stdint.h>
#include "L47XX.h"

void delay(void)
{
	for(uint32_t i=0; i<500000/2; i++);
}

int main(void)
{
   GPIO_Handle_t GpioLed,GpioBtn;
   GpioLed.pGPIOx= GPIOA;
   GpioLed.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_5;
   GpioLed.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
   GpioLed.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OP_TYPE_OD;
   GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PIN_PU;
   GpioLed.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_SPEED_FAST;

   GPIO_PeriClockControl(GPIOA, ENABLE);
   GPIO_Init(&GpioLed);
   GpioBtn.pGPIOx= GPIOC;
   GpioBtn.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_13;
   GpioBtn.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IN;
   GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_NO_PuPd;
   GpioBtn.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_SPEED_FAST;

   GPIO_PeriClockControl(GPIOA, ENABLE);
   GPIO_Init(&GpioLed);
   GPIO_PeriClockControl(GPIOC, ENABLE);
   GPIO_Init(&GpioBtn);

   while(1)
   {
	   if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13))
	   {
		   delay();
	   GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
	   }

   }

	return(1);
}
