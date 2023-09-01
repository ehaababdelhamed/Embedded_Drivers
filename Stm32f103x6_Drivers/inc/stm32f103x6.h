 /******************************************************************************
 * File Name: stm32f103x6.h
 * Description: MCU device header for stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

#ifndef INC_STM32F103X6_H_
#define INC_STM32F103X6_H_

/*******************************************************************************
 *                             		INCLUDES                            		*
 *******************************************************************************/
#include "Platform_Types.h"


/*******************************************************************************
 *						Base addresses for Memories                       		*
 *******************************************************************************/
#define FLASH_MEMORY_BASE							(0x08000000UL)
#define SYSTEM_MEMORY_BASE							(0x1FFFF000UL)
#define SRAM_MEMORY_BASE							(0x20000000UL)
#define PERIPHERALS_MEMORY_BASE						(0x40000000UL)
#define CORTEX_M3_INTERNAL_PERIPHERALS_MEMORY_BASE	(0xE0000000UL)

/*******************************************************************************
 *						Base addresses for Cortex M3 internal Peripherals                    	*
 *******************************************************************************/
//NVIC
#define NVIC_BASE		(0xE000E100UL)

/*******************************************************************************
 *						Base addresses for AHP Peripherals                    	*
 *******************************************************************************/
#define RCC_BASE		(0x40021000UL)



/*******************************************************************************
 *						Base addresses for APB1 Peripherals                    	*
 *******************************************************************************/

//GPIO
//LQF48 Package fully included PORTA and PORTB
#define GPIOA_BASE		(0x40010800UL)
#define GPIOB_BASE		(0x40010C00UL)

//LQF48 Package partially included PORTC and PORTD
#define GPIOC_BASE		(0x40011000UL)
#define GPIOD_BASE		(0x40011400UL)

//LQF48 Package not included PORTE
#define GPIOE_BASE		(0x40011800UL)

//EXTI
#define EXTI_BASE		(0x40010400UL)

//AFIO

#define AFIO_BASE		(0x40010000UL)

/*******************************************************************************
 *						Base addresses for APB2 Peripherals                    	*
 *******************************************************************************/





//===============================================================================
/*******************************************************************************
 *							Peripherals Registers				               	*
 *******************************************************************************/
//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: GPIO
//-*-*-*-*-*-*-*-*-*-*-*
typedef struct{
	volatile uint32 CRL;
	volatile uint32 CRH;
	volatile uint32 IDR;
	volatile uint32 ODR;
	volatile uint32 BSRR;
	volatile uint32 BRR;
	volatile uint32 LCKR;
}GPIO_TypeDef;

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: RCC
//-*-*-*-*-*-*-*-*-*-*-*

typedef struct{
	volatile uint32 CR;
	volatile uint32 CFGR;
	volatile uint32 CIR;
	volatile uint32 APB2RSTR;
	volatile uint32 APB1RSTR;
	volatile uint32 AHBENR;
	volatile uint32 APB2ENR;
	volatile uint32 APB1ENR;
	volatile uint32 BDCR;
	volatile uint32 CSR;
	volatile uint32 AHBSTR;
	volatile uint32 CFGR2;
}RCC_TypeDef;

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: EXTI
//-*-*-*-*-*-*-*-*-*-*-*

typedef struct{
	volatile uint32 IMR;
	volatile uint32 EMR;
	volatile uint32 RTSR;
	volatile uint32 FTSR;
	volatile uint32 SWIER;
	volatile uint32 PR;
}EXTI_TypeDef;


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: AFIO
//-*-*-*-*-*-*-*-*-*-*-*

typedef struct{
	volatile uint32 EVCR;
	volatile uint32 MAPR;
	volatile uint32 EXTICR[4];
	volatile uint32 RESERVED0;
	volatile uint32 MAPR2;
}AFIO_TypeDef;

//======================================================================================
//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants: GPIO
//-*-*-*-*-*-*-*-*-*-*-*

#define GPIOA		((GPIO_TypeDef*)(GPIOA_BASE))
#define GPIOB		((GPIO_TypeDef*)(GPIOB_BASE))
#define GPIOC		((GPIO_TypeDef*)(GPIOC_BASE))
#define GPIOD		((GPIO_TypeDef*)(GPIOD_BASE))
#define GPIOE		((GPIO_TypeDef*)(GPIOE_BASE))
//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants: EXTI
//-*-*-*-*-*-*-*-*-*-*-*

#define EXTI		((EXTI_TypeDef*)(EXTI_BASE))
//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants: AFIO
//-*-*-*-*-*-*-*-*-*-*-*

#define AFIO		((AFIO_TypeDef*)(AFIO_BASE))

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants: RCC
//-*-*-*-*-*-*-*-*-*-*-*
#define RCC			((RCC_TypeDef*)(RCC_BASE))

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: NVIC
//-*-*-*-*-*-*-*-*-*-*-*
//REG0 >> [IRQ0 - IRQ31]
//REG1 >> [IRQ32 - IRQ63]
//REG2 >> [IRQ64 - IRG67]
#define NVIC_ISER0	(*(volatile uint32*)(NVIC_BASE+0X00))
#define NVIC_ISER1	(*(volatile uint32*)(NVIC_BASE+0X04))
#define NVIC_ISER2	(*(volatile uint32*)(NVIC_BASE+0X08))
#define NVIC_ICER0	(*(volatile uint32*)(NVIC_BASE+0X80))
#define NVIC_ICER1	(*(volatile uint32*)(NVIC_BASE+0X84))
#define NVIC_ICER2	(*(volatile uint32*)(NVIC_BASE+0X88))


/*******************************************************************************
 *							Clock Enable Macros									*
 *******************************************************************************/
/*Macros for pins of RCC_APB2RSTR*/
#define AFIOEN		(1U<<0)
#define IOPAEN		(1U<<2)
#define IOPBEN		(1U<<3)
#define IOPCEN		(1U<<4)
#define IOPDEN		(1U<<5)
#define IOPEEN		(1U<<6)

#define ADC1EN		(1U<<9)
#define ADC2EN		(1U<<10)
#define TIM1EN		(1U<<11)
#define SPI1EN		(1U<<12)
#define USART1EN	(1U<<14)

/*Enable Clock*/
#define RCC_AFIO_CLK_EN()		(RCC->APB2ENR|= AFIOEN)
#define RCC_GPIOA_CLK_EN()		(RCC->APB2ENR |= IOPAEN)
#define RCC_GPIOB_CLK_EN()		(RCC->APB2ENR |= IOPBEN)
#define RCC_GPIOC_CLK_EN()		(RCC->APB2ENR |= IOPCEN)
#define RCC_GPIOD_CLK_EN()		(RCC->APB2ENR |= IOPDEN)
#define RCC_GPIOE_CLK_EN()		(RCC->APB2ENR |= IOPEEN)

#define RCC_ADC1_CLK_EN()		(RCC->APB2ENR |= ADC1EN)
#define RCC_ADC2_CLK_EN()		(RCC->APB2ENR |= ADC2EN)
#define RCC_TIMER1_CLK_EN()		(RCC->APB2ENR |= TIM1EN)
#define RCC_SPI1_CLK_EN()		(RCC->APB2ENR |= SPI1EN)
#define RCC_USART1_CLK_EN()		(RCC->APB2ENR |= USART1EN)
/*Disable Clock*/

/*Reset Modules*/
#define RCC_AFIO_RESET()		(RCC->APB2RSTR |= AFIOEN)
#define RCC_GPIOA_RESET()		(RCC->APB2RSTR |= IOPAEN)
#define RCC_GPIOB_RESET()		(RCC->APB2RSTR |= IOPBEN)
#define RCC_GPIOC_RESET()		(RCC->APB2RSTR |= IOPCEN)
#define RCC_GPIOD_RESET()		(RCC->APB2RSTR |= IOPDEN)
#define RCC_GPIOE_RESET()		(RCC->APB2RSTR |= IOPEEN)

#define RCC_ADC1_RESET()		(RCC->APB2RSTR |= ADC1EN)
#define RCC_ADC2_RESET()		(RCC->APB2RSTR |= ADC2EN)
#define RCC_TIMER1_RESET()		(RCC->APB2RSTR |= TIM1EN)
#define RCC_SPI1_RESET()		(RCC->APB2RSTR |= SPI1EN)
#define RCC_USART1_RESET()		(RCC->APB2RSTR |= USART1EN)

/*******************************************************************************
 *							NCIC IRQ Enable/Disable Macros									*
 *******************************************************************************/
#define NVIC_IRQ6_EXTI0_Enable()		(NVIC_ISER0 |= 1<<6)
#define NVIC_IRQ7_EXTI1_Enable()		(NVIC_ISER0 |= 1<<7)
#define NVIC_IRQ8_EXTI2_Enable()		(NVIC_ISER0 |= 1<<8)
#define NVIC_IRQ9_EXTI3_Enable()		(NVIC_ISER0 |= 1<<9)
#define NVIC_IRQ10_EXTI4_Enable()		(NVIC_ISER0 |= 1<<10)
#define NVIC_IRQ23_EXTI5_9_Enable()		(NVIC_ISER0 |= 1<<23)
#define NVIC_IRQ40_EXTI10_15_Enable()	(NVIC_ISER1 |= 1<<8)	//40-32 = 8


#define NVIC_IRQ6_EXTI0_Disable()		(NVIC_ICER0 |= 1<<6)
#define NVIC_IRQ7_EXTI1_Disable()		(NVIC_ICER0 |= 1<<7)
#define NVIC_IRQ8_EXTI2_Disable()		(NVIC_ICER0 |= 1<<8)
#define NVIC_IRQ9_EXTI3_Disable()		(NVIC_ICER0 |= 1<<9)
#define NVIC_IRQ10_EXTI4_Disable()		(NVIC_ICER0 |= 1<<10)
#define NVIC_IRQ23_EXTI5_9_Disable()	(NVIC_ICER0 |= 1<<23)
#define NVIC_IRQ40_EXTI10_15_Disable()	(NVIC_ICER1 |= 1<<8)	//40-32 = 8


/*******************************************************************************
 *									IVT											*
 *******************************************************************************/
#define EXTI0_IRQ		(6U)
#define EXTI1_IRQ		(7U)
#define EXTI2_IRQ		(8U)
#define EXTI3_IRQ		(9U)
#define EXTI4_IRQ		(10U)
#define EXTI5_IRQ		(23U)
#define EXTI6_IRQ		(23U)
#define EXTI7_IRQ		(23U)
#define EXTI8_IRQ		(23U)
#define EXTI9_IRQ		(23U)
#define EXTI10_IRQ		(40U)
#define EXTI11_IRQ		(40U)
#define EXTI12_IRQ		(40U)
#define EXTI13_IRQ		(40U)
#define EXTI14_IRQ		(40U)
#define EXTI15_IRQ		(40U)


#endif /* INC_STM32F103X6_H_ */
