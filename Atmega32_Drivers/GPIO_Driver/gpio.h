 /******************************************************************************
 * Module: GPIO
 * File Name: gpio.h
 * Description: Header file for GPIO driver for Atmega32
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/
#ifndef INC_GPIO_H_
#define INC_GPIO_H_

/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "Common_Macros.h"
#include "Platform_Types.h"

/*******************************************************************************
 *                             	GPIO Data Types                             	*
 *******************************************************************************/

//@REF : GPIO_Modes
/*Supported mode in GPIO
1)Input mode
2) Output mode
*/

typedef enum{
	GPIO_INPUT_FLOATING,GPIO_OUTPUT_PP,GPIO_INPUT_PU
}GPIO_Mode_t;

typedef struct{
	uint8 Pin_Num;				//Specifies GPIO pin to be configured
								//This parameter can be a value of @REF: GPIO_PINS_NUMBER
	GPIO_Mode_t Pin_Mode;		//Specifies Operation mode of GPIO pin
								//This parameter can be a value of @REF: GPIO_Modes
}GPIO_PinConfig_t;

typedef struct{
	uint8 Pin_Num;				//Specifies GPIO pin to be configured
								//This parameter can be a value of @REF: GPIO_Speed
	uint8 Offest;				//To select the position of start bit from LSB
	uint8 mask;					//to select the number of bits in group
}GPIO_GroupConfig_t;

/*******************************************************************************
 *                             	GPIO Definition                          		*
 *******************************************************************************/
//@REF: GPIO_PINS_NUMBER
#define GPIO_PIN_0			(0U)
#define GPIO_PIN_1			(1U)
#define GPIO_PIN_2			(2U)
#define GPIO_PIN_3			(3U)
#define GPIO_PIN_4			(4U)
#define GPIO_PIN_5			(5U)
#define GPIO_PIN_6			(6U)
#define GPIO_PIN_7			(7U)


//@REF: GPIO_NUMBER
#define GPIO_PORTA			(0U)
#define GPIO_PORTB			(1U)
#define GPIO_PORTC			(2U)
#define GPIO_PORTD			(3U)
//@REF:GPIO_PIN_LEVEL
#define PIN_HIGH			(1U)
#define PIN_LOW				(0U)

/*******************************************************************************
 *					APIs Supported by "MCAL GPIO DRIVER"                       *
 *******************************************************************************/


void MCAL_GPIO_init(uint8 GPIOx_Num,GPIO_PinConfig_t *pinConfig);
void MCAL_GPIO_deInit(uint8 GPIOx_Num);

uint8 MCAL_GPIO_readPin(uint8 GPIOx_Num,uint8 Pin_Num);
uint8 MCAL_GPIO_readPort(uint8 GPIOx_Num);

void MCAL_GPIO_writePin(uint8 GPIOx_Num,uint8 Pin_Num,uint8 Pin_Value);
void MCAL_GPIO_writePort(uint8 GPIOx_Num,uint8 Port_Value);

void MCAL_GPIO_writeGroub(uint8 GPIO_Portx,uint8 mask,uint8 offsets,uint8 value);
uint8 MCAL_GPIO_readGroup(uint8 GPIO_Portx,uint8 mask,uint8 offset);

void MCAL_GPIO_togglePin(uint8 GPIOx_Num,uint8 Pin_Num);

#endif /* INC_GPIO_H_ */
