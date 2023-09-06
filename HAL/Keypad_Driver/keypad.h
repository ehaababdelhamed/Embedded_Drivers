/******************************************************************************
 * Module: Keypad
 * File Name: keypad.h
 * Description: header file for Keypad HAL driver
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

#ifndef KEYPAD_H_
#define KEYPAD_H_
/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "Platform_Types.h"
/*******************************************************************************
 *                             User Configuration                              *
 *******************************************************************************/
//Configure number of rows and columns
#define NUMOFCOL 	(4U)
#define NUMOFROW	(4U)
//Configure Keypad logic
#define KEY_PAD_LOGIC		CHECK_LOW		//[CHECK_LOW - CHECK_HIGH]
//Configure rows mode
#define KEYPAD_ROWS_MODE	GPIO_INPUT_PD 	//[GPIO_INPUT_PD - GPIO_INPUT_PU - GPIO_INPUT_FLOATING]

//Configure rows and columns ports and pins
#define KEYPAD_COL_PORT		GPIO_PORTB
#define KEYPAD_COL0			GPIO_PIN_5
#define KEYPAD_COL1			GPIO_PIN_6
#define KEYPAD_COL2			GPIO_PIN_7
#define KEYPAD_COL3			GPIO_PIN_8

#define KEYPAD_ROW_PORT		GPIO_PORTB
#define KEYPAD_ROW0			GPIO_PIN_0
#define KEYPAD_ROW1			GPIO_PIN_1
#define KEYPAD_ROW2			GPIO_PIN_3
#define KEYPAD_ROW3			GPIO_PIN_4

#define KEY_ROWS		{KEYPAD_ROW0,KEYPAD_ROW1,KEYPAD_ROW2,KEYPAD_ROW3}
#define KEY_COLS		{KEYPAD_COL0,KEYPAD_COL1,KEYPAD_COL2,KEYPAD_COL3}

#define KEYPAD_KEYS    {{'7','8','9','/'},\
						{'4','5','6','X'},\
						{'1','2','3','-'},\
						{' ','0','=','+'}}
/*******************************************************************************
 *                            Module Definition                            *
 *******************************************************************************/
#define KEYPAD_NOT_PRESSED 	('N')
#define CHECK_HIGH			(1U)
#define CHECK_LOW			(0U)

#if KEY_PAD_LOGIC == CHECK_HIGH
#define ALL_COLS_LEVEL 	PIN_HIGH
#define	COL_LEVEL		PIN_LOW
#define ROW_CHECK_LEVEL	PIN_HIGH
#elif KEY_PAD_LOGIC == CHECK_LOW
#define ALL_COLS_LEVEL 	PIN_LOW
#define	COL_LEVEL		PIN_HIGH
#define ROW_CHECK_LEVEL	PIN_HIGH
#endif


/*******************************************************************************
 *                      	Function Prototypes				                	*
 *******************************************************************************/
void Keypad_init();
uint8 Keypad_getKey();


#endif /* KEYPAD_H_ */
