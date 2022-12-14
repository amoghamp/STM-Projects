/*
 * gpio_driver.h
 *
 *  Created on: Sep 2, 2022
 *      Author: amogh
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_
#include<stdint.h>
#include"L47XX.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode; //@GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;



typedef struct
{
	GPIO_RegDef_t *pGPIOx ;// hold the base addr of gpio port
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


// init and de init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// for clock
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

// for read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// for interrupt
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi );
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


// gpio modes
/*
 * @GPIO_PIN_MODES
 */
#define 	GPIO_MODE_IN    	0
#define 	GPIO_MODE_OUT		1
#define 	GPIO_MODE_ALTFN		2
#define 	GPIO_MODE_ANALOG	3
#define 	GPIO_MODE_IT_FT		4
#define 	GPIO_MODE_IT_RT		5
#define 	GPIO_MODE_IT_RFT	6

#define 	GPIO_OP_TYPE_PP 	0
#define 	GPIO_OP_TYPE_OD 	1

#define 	GPIO_SPEED_LOW		0
#define 	GPIO_SPEED_MEDIUM	1
#define 	GPIO_SPEED_FAST		2
#define 	GPIO_SPEED_HIGH		3

#define 	GPIO_NO_PuPd		0
#define 	GPIO_PIN_PU			1
#define 	GPIO_PD				2



#define 	GPIO_PIN_NO_0		0
#define 	GPIO_PIN_NO_1		1
#define 	GPIO_PIN_NO_2		2
#define 	GPIO_PIN_NO_3		3
#define 	GPIO_PIN_NO_4		4
#define 	GPIO_PIN_NO_5		5
#define 	GPIO_PIN_NO_6		6
#define 	GPIO_PIN_NO_7		7
#define 	GPIO_PIN_NO_8		8
#define 	GPIO_PIN_NO_9		9
#define 	GPIO_PIN_NO_10		10
#define 	GPIO_PIN_NO_11		11
#define 	GPIO_PIN_NO_12		12
#define 	GPIO_PIN_NO_13		13
#define 	GPIO_PIN_NO_14		14
#define 	GPIO_PIN_NO_15		15


#endif /* INC_GPIO_DRIVER_H_ */
