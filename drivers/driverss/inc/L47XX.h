/*
 * L47XX.h
 *
 *  Created on: Sep 2, 2022
 *      Author: amogh
 */

#ifndef INC_L47XX_H_
#define INC_L47XX_H_

#include<stdint.h>
#include <stddef.h>
#include <stdio.h>
#include<string.h>





 #define __vo volatile

 #define 	NVIC_ISER0					((__vo uint32_t *)0xE000E100U)
 #define	NVIC_ISER1					((__vo uint32_t *)0xE000E104U)
 #define	NVIC_ISER2					((__vo uint32_t *)0xE000E108U)
 #define	NVIC_ISER3					((__vo uint32_t *)0xE000E10CU)


 #define 	NVIC_ICER0					((__vo uint32_t *)0XE000E180U)
 #define	NVIC_ICER1					((__vo uint32_t *)0xE000E184U)
 #define	NVIC_ICER2					((__vo uint32_t *)0xE000E188U)
 #define	NVIC_ICER3					((__vo uint32_t *)0xE000E18CU)

#define 	NVIC_PR_BASE_ADDR 			((__vo uint32_t*)0xE000E400U)
#define 	NO_PR_BITS_IMPLEMENTED 		4


 #define 	FLASH_BASEADDR  			0x08000000U
 #define 	SRAM1_BASEAADR				0x20000000U
 #define 	SRAM2_BASEADDR        		0x10000000U
 #define 	ROM_BASEADDR 				0x1FFF0000U
 #define 	SRAM						SRAM1_BASEAADR

// base address of busses
 #define 	APB1PERIPH_BASEADDR			0x40000000U
 #define 	APB2PERIPH_BASEADDR			0x40010000U
 #define 	AHB1PERIPH_BASEADDR 		0x40020000U
 #define 	AHB2PERIPH_BASEADDR			0x48000000U
 #define 	AHB3PERIPH_BASEADDR			0xA0000000U
 #define 	AHB4PERIPH_BASEADDR			0xA0001000U

// GPIO base address

 #define	GPIOA_BASEADDR  			0x48000000U
 #define 	GPIOB_BASEADDR  			0x48000400U
 #define 	GPIOC_BASEADDR 				0x48000800U
 #define 	GPIOD_BASEADDR  			0x48000C00U
 #define	GPIOE_BASEADDR  			0x48001000U
 #define 	GPIOF_BASEADDR  			0x48001400U
 #define 	GPIOG_BASEADDR  			0x48001800U
 #define 	GPIOH_BASEADDR  			0x48001C00U


// apb1 peripherals
 #define	I2C1_BASEADDR				0x40005400U
 #define	I2C2_BASEADDR				0x40005800U
 #define	I2C3_BASEADDR				0x40005C00U
 #define	SPI2_BASEADDR				0x40003800U
 #define	SPI3_BASEADDR				0x40003C00U
 #define	USART2_BASEADDR				0x40004400U
 #define	USART3_BASEADDR				0x40004800U
 #define	UART4_BASEADDR				0x40004C00U
 #define	UART5_BASEADDR				0x40005000U

// apb2 bus
#define 	SPI1_BASEADDR		 		0x40013000U
#define 	USART1_BASEADDR		 		0x40013800U
#define 	SYSCFG_BASEADDR		 		0x40010000U
#define		EXTI_BASEADDR				0x40010400U


// RCC macro
#define 	RCC_BASEADDR 				0x40021000


// exti


// REGISTER definition

typedef struct
{
  __vo uint32_t MODER;       /*!< GPIO port mode register,               Address offset: 0x00      */
  __vo uint32_t OTYPER;      /*!< GPIO port output type register,        Address offset: 0x04      */
  __vo uint32_t OSPEEDR;     /*!< GPIO port output speed register,       Address offset: 0x08      */
  __vo uint32_t PUPDR;       /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __vo uint32_t IDR;         /*!< GPIO port input data register,         Address offset: 0x10      */
  __vo uint32_t ODR;         /*!< GPIO port output data register,        Address offset: 0x14      */
  __vo uint32_t BSRR;        /*!< GPIO port bit set/reset  register,     Address offset: 0x18      */
  __vo uint32_t LCKR;        /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __vo uint32_t AFR[2];      /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
  __vo uint32_t BRR;         /*!< GPIO Bit Reset register,               Address offset: 0x28      */
  __vo uint32_t ASCR;        /*!< GPIO analog switch control register,   Address offset: 0x2C     */

} GPIO_RegDef_t;

typedef struct
{
  __vo uint32_t CR;          /*!< RCC clock control register,                                              Address offset: 0x00 */
  __vo uint32_t ICSCR;       /*!< RCC internal clock sources calibration register,                         Address offset: 0x04 */
  __vo uint32_t CFGR;        /*!< RCC clock configuration register,                                        Address offset: 0x08 */
  __vo uint32_t PLLCFGR;     /*!< RCC system PLL configuration register,                                   Address offset: 0x0C */
  __vo uint32_t PLLSAI1CFGR; /*!< RCC PLL SAI1 configuration register,                                     Address offset: 0x10 */
  __vo uint32_t PLLSAI2CFGR; /*!< RCC PLL SAI2 configuration register,                                     Address offset: 0x14 */
  __vo uint32_t CIER;        /*!< RCC clock interrupt enable register,                                     Address offset: 0x18 */
  __vo uint32_t CIFR;        /*!< RCC clock interrupt flag register,                                       Address offset: 0x1C */
  __vo uint32_t CICR;        /*!< RCC clock interrupt clear register,                                      Address offset: 0x20 */
  uint32_t      RESERVED0;   /*!< Reserved,                                                                Address offset: 0x24 */
  __vo uint32_t AHB1RSTR;    /*!< RCC AHB1 peripheral reset register,                                      Address offset: 0x28 */
  __vo uint32_t AHB2RSTR;    /*!< RCC AHB2 peripheral reset register,                                      Address offset: 0x2C */
  __vo uint32_t AHB3RSTR;    /*!< RCC AHB3 peripheral reset register,                                      Address offset: 0x30 */
  uint32_t      RESERVED1;   /*!< Reserved,                                                                Address offset: 0x34 */
  __vo uint32_t APB1RSTR1;   /*!< RCC APB1 peripheral reset register 1,                                    Address offset: 0x38 */
  __vo uint32_t APB1RSTR2;   /*!< RCC APB1 peripheral reset register 2,                                    Address offset: 0x3C */
  __vo uint32_t APB2RSTR;    /*!< RCC APB2 peripheral reset register,                                      Address offset: 0x40 */
  uint32_t      RESERVED2;   /*!< Reserved,                                                                Address offset: 0x44 */
  __vo uint32_t AHB1ENR;     /*!< RCC AHB1 peripheral clocks enable register,                              Address offset: 0x48 */
  __vo uint32_t AHB2ENR;     /*!< RCC AHB2 peripheral clocks enable register,                              Address offset: 0x4C */
  __vo uint32_t AHB3ENR;     /*!< RCC AHB3 peripheral clocks enable register,                              Address offset: 0x50 */
  uint32_t      RESERVED3;   /*!< Reserved,                                                                Address offset: 0x54 */
  __vo uint32_t APB1ENR1;    /*!< RCC APB1 peripheral clocks enable register 1,                            Address offset: 0x58 */
  __vo uint32_t APB1ENR2;    /*!< RCC APB1 peripheral clocks enable register 2,                            Address offset: 0x5C */
  __vo uint32_t APB2ENR;     /*!< RCC APB2 peripheral clocks enable register,                              Address offset: 0x60 */
  uint32_t      RESERVED4;   /*!< Reserved,                                                                Address offset: 0x64 */
  __vo uint32_t AHB1SMENR;   /*!< RCC AHB1 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x68 */
  __vo uint32_t AHB2SMENR;   /*!< RCC AHB2 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x6C */
  __vo uint32_t AHB3SMENR;   /*!< RCC AHB3 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x70 */
  uint32_t      RESERVED5;   /*!< Reserved,                                                                Address offset: 0x74 */
  __vo uint32_t APB1SMENR1;  /*!< RCC APB1 peripheral clocks enable in sleep mode and stop modes register 1, Address offset: 0x78 */
  __vo uint32_t APB1SMENR2;  /*!< RCC APB1 peripheral clocks enable in sleep mode and stop modes register 2, Address offset: 0x7C */
  __vo uint32_t APB2SMENR;   /*!< RCC APB2 peripheral clocks enable in sleep mode and stop modes register, Address offset: 0x80 */
  uint32_t      RESERVED6;   /*!< Reserved,                                                                Address offset: 0x84 */
  __vo uint32_t CCIPR;       /*!< RCC peripherals independent clock configuration register,                Address offset: 0x88 */
  uint32_t      RESERVED7;   /*!< Reserved,                                                                Address offset: 0x8C */
  __vo uint32_t BDCR;        /*!< RCC backup domain control register,                                      Address offset: 0x90 */
  __vo uint32_t CSR;         /*!< RCC clock control & status register,                                     Address offset: 0x94 */
} RCC_RegDef_t;
/**
 *
 *
 */

typedef struct
{
	__vo uint32_t EXTI_IMR1;
	__vo uint32_t EXTI_EMR1;
	__vo uint32_t EXTI_RTSR1;
	__vo uint32_t EXTI_FTSR1;
	__vo uint32_t EXTI_SWIER1;
	__vo uint32_t EXTI_PR1;
	__vo uint32_t EXTI_IMR2;
	__vo uint32_t EXTI_EMR2;
	__vo uint32_t EXTI_RTSR2;
	__vo uint32_t EXTI_FTSR2;
	__vo uint32_t EXTI_SWIER2;
	__vo uint32_t EXTI_PR2;
}EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t SYSCFG_MEMRMP;
	__vo uint32_t SYSCFG_CFGR1;
	__vo uint32_t SYSCFG_EXTICR[3];
	__vo uint32_t SYSCFG_SCSR;
	__vo uint32_t SYSCFG_CFGR2;
	__vo uint32_t SYSCFG_SWPR;
	__vo uint32_t SYSCFG_SKR;
	__vo uint32_t SYSCFG_SWPR2;
}SYSCFG_RegDef_t;


// type casted value peripheral definition

 #define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR)
 #define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASEADDR)
 #define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASEADDR)
 #define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASEADDR)
 #define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASEADDR)
 #define GPIOF 		((GPIO_RegDef_t*)GPIOF_BASEADDR)
 #define GPIOG 		((GPIO_RegDef_t*)GPIOG_BASEADDR)
 #define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASEADDR)





// macro for RCC
 #define RCC 		((RCC_RegDef_t*)RCC_BASEADDR)

 #define EXTI 		((EXTI_RegDef_t*)EXTI_BASEADDR)
 #define SYSCFG     ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
//  enable clock
// Gpio cloclks
 #define 	GPIOA_PCLK_EN() 	(RCC->AHB2ENR |= (1<<0))
 #define 	GPIOB_PCLK_EN() 	(RCC->AHB2ENR |= (1<<1))
 #define 	GPIOC_PCLK_EN() 	(RCC->AHB2ENR |= (1<<2))
 #define 	GPIOD_PCLK_EN() 	(RCC->AHB2ENR |= (1<<3))
 #define 	GPIOE_PCLK_EN() 	(RCC->AHB2ENR |= (1<<4))
 #define 	GPIOF_PCLK_EN() 	(RCC->AHB2ENR |= (1<<5))
 #define 	GPIOG_PCLK_EN() 	(RCC->AHB2ENR |= (1<<6))
 #define 	GPIOH_PCLK_EN() 	(RCC->AHB2ENR |= (1<<7))

// apb1 clocks
 #define 	I2C1_PCLK_EN() 		(RCC->APB1ENR1 |= (1<<21))
 #define 	I2C2_PCLK_EN() 		(RCC->APB1ENR1 |= (1<<22))
 #define 	I2C3_PCLK_EN() 		(RCC->APB1ENR1 |= (1<<23))
 #define 	SPI2_PCLK_EN() 		(RCC->APB1ENR1 |= (1<<14))
 #define 	SPI3_PCLK_EN() 		(RCC->APB1ENR1 |= (1<<15))
 #define 	USART2_PCLK_EN() 	(RCC->APB1ENR1 |= (1<<17))
 #define 	USART3_PCLK_EN() 	(RCC->APB1ENR1 |= (1<<18))
 #define 	UART4_PCLK_EN() 	(RCC->APB1ENR1 |= (1<<19))
 #define 	UART5_PCLK_EN() 	(RCC->APB1ENR1 |= (1<<20))
//apb2 clocks
 #define 	SPI1_PCLK_EN() 		(RCC->APB2ENR |= (1<<12))
 #define 	USART1_PCLK_EN() 	(RCC->APB2ENR |= (1<<14))
 #define 	EXTI_PCLK_EN() 		(RCC->APB2ENR |= (1<<21))
 #define 	SYSCFG_PCLK_EN() 	(RCC->APB2ENR |= (1<<0))

// diable clocks
// Gpio cloclks
 #define 	GPIOA_PCLK_DI() 	(RCC->AHB2ENR &= ~ (1<<0))
 #define 	GPIOB_PCLK_DI() 	(RCC->AHB2ENR &= ~(1<<1))
 #define 	GPIOC_PCLK_DI() 	(RCC->AHB2ENR &= ~(1<<2))
 #define 	GPIOD_PCLK_DI() 	(RCC->AHB2ENR &= ~(1<<3))
 #define 	GPIOE_PCLK_DI() 	(RCC->AHB2ENR &= ~(1<<4))
 #define 	GPIOF_PCLK_DI() 	(RCC->AHB2ENR &= ~(1<<5))
 #define 	GPIOG_PCLK_DI() 	(RCC->AHB2ENR &= ~(1<<6))
 #define 	GPIOH_PCLK_DI() 	(RCC->AHB2ENR &= ~(1<<7))
 #define 	GPIOI_PCLK_DI() 	(RCC->AHB2ENR &= ~(1<<8))
// apb1 clocks
 #define 	I2C1_PCLK_DI() 		(RCC->APB1ENR1 &= ~(1<<21))
 #define 	I2C2_PCLK_DI() 		(RCC->APB1ENR1 &= ~(1<<22))
 #define 	I2C3_PCLK_DI() 		(RCC->APB1ENR1 &= ~(1<<23))
 #define 	SPI2_PCLK_DI() 		(RCC->APB1ENR1 &= ~(1<<14))
 #define 	SPI3_PCLK_DI() 		(RCC->APB1ENR1 &= ~(1<<15))
 #define 	USART2_PCLK_DI() 	(RCC->APB1ENR1 &= ~(1<<17))
 #define 	USART3_PCLK_DI() 	(RCC->APB1ENR1 &= ~(1<<18))
 #define 	UART4_PCLK_DI() 	(RCC->APB1ENR1 &= ~(1<<19))
 #define 	UART5_PCLK_DI() 	(RCC->APB1ENR1 &= ~(1<<20))
//apb2 clocks
 #define 	SPI1_PCLK_DI() 		(RCC->APB2ENR &= ~(1<<12))
 #define 	USART1_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<14))
 #define 	EXTI_PCLK_DI() 		(RCC->APB2ENR &= ~(1<<21))
 #define 	SYSCFG_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<0))


// reset cclock
// gpio clocks
#define 	GPIOA_REG_RESET()		do{ (RCC->AHB2RSTR |= (1<<0)); (RCC->AHB2RSTR &= ~(1<<0));} while(0)
#define 	GPIOB_REG_RESET()		do{ (RCC->AHB2RSTR |= (1<<1)); (RCC->AHB2RSTR &= ~(1<<0));} while(0)
#define 	GPIOC_REG_RESET()		do{ (RCC->AHB2RSTR |= (1<<2)); (RCC->AHB2RSTR &= ~(1<<0));} while(0)
#define 	GPIOD_REG_RESET()		do{ (RCC->AHB2RSTR |= (1<<3)); (RCC->AHB2RSTR &= ~(1<<0));} while(0)
#define 	GPIOE_REG_RESET()		do{ (RCC->AHB2RSTR |= (1<<4)); (RCC->AHB2RSTR &= ~(1<<0));} while(0)
#define 	GPIOF_REG_RESET()		do{ (RCC->AHB2RSTR |= (1<<5)); (RCC->AHB2RSTR &= ~(1<<0));} while(0)
#define 	GPIOG_REG_RESET()		do{ (RCC->AHB2RSTR |= (1<<6)); (RCC->AHB2RSTR &= ~(1<<0));} while(0)
#define 	GPIOH_REG_RESET()		do{ (RCC->AHB2RSTR |= (1<<7)); (RCC->AHB2RSTR &= ~(1<<0));} while(0)
#define 	GPIOI_REG_RESET()		do{ (RCC->AHB2RSTR |= (1<<8)); (RCC->AHB2RSTR &= ~(1<<0));} while(0)


#define 	GPIO_BASEADDR_TO_CODE(x)  	(	(x== GPIOA)?0:\
									  		(x==GPIOB)?1:\
									  		(x==GPIOC)?2:\
									  		(x==GPIOD)?3:\
									  		(x==GPIOE)?4:\
									  		(x==GPIOF)?5:\
									  		(x==GPIOG)?6:\
									  		(x==GPIOH)?7:0)
#define 	ENABLE 					1
#define 	DISABLE 				0
#define 	SET 					ENABLE
#define 	RESET 					DISABLE
#define 	GPIO_PIN_SET 			SET
#define 	GPIO_PIN_RESET 			RESET




#define		IRQ_NO_EXTI0			6
#define		IRQ_NO_EXTI1			7
#define		IRQ_NO_EXTI2			8
#define		IRQ_NO_EXTI3			9
#define		IRQ_NO_EXTI4			10
#define		IRQ_NO_EXTI9_5			23
#define		IRQ_NO_EXTI15_10		40

#define 	NVIC_IRQ_PRIO0			0
#define 	NVIC_IRQ_PRIO1			1
#define 	NVIC_IRQ_PRIO2			2
#define 	NVIC_IRQ_PRIO3			3
#define 	NVIC_IRQ_PRIO4			4
#define 	NVIC_IRQ_PRIO5			5
#define 	NVIC_IRQ_PRIO6			6
#define 	NVIC_IRQ_PRIO7			7
#define 	NVIC_IRQ_PRIO8			8
#define 	NVIC_IRQ_PRIO9			9
#define 	NVIC_IRQ_PRIO10			10
#define 	NVIC_IRQ_PRIO11			11
#define 	NVIC_IRQ_PRIO12			12
#define 	NVIC_IRQ_PRIO13			13
#define 	NVIC_IRQ_PRIO14			14
#define 	NVIC_IRQ_PRIO15			15

#include "gpio_driver.h"

#endif /* INC_L47XX_H_ */
