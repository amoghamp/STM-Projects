
/*
 * 005.c
 *
 *  Created on: 10-Sep-2022
 *      Author: amogh
 */
#include <stdint.h>
#include "L47XX.h"

int main(void)
{
   GPIO_Handle_t GpioLed,GpioBtn;
   memset(&GpioLed,0,sizeof(GpioLed));
   memset(&GpioBtn,0,sizeof(GpioBtn));
   GPIO_PeriClockControl(GPIOA, ENABLE);
   GPIO_PeriClockControl(GPIOC, ENABLE);
   GpioLed.pGPIOx= GPIOA;
   GpioLed.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_5;
   GpioLed.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
   GpioLed.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OP_TYPE_PP;
   GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_NO_PuPd;
   GpioLed.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_SPEED_FAST;

   GpioBtn.pGPIOx= GPIOC;
   GpioBtn.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_13;
   GpioBtn.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IT_FT;
   GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PIN_PU;
   GpioBtn.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_SPEED_FAST;

  // GPIO_PeriClockControl(GPIOA, ENABLE);
   GPIO_Init(&GpioLed);
 //  GPIO_PeriClockControl(GPIOC, ENABLE);
   GPIO_Init(&GpioBtn);


	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
	return(1);
}

void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
}


