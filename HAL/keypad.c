/******************************************************************************
 * Module: Keypad
 * File Name: keypad.c
 * Description: Source file for keypad HAL driver
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

/*******************************************************************************
 *                             		Includes			                       *
 *******************************************************************************/
#include "keypad.h"
#include "stm32f103x6.h"
#include "Common_Macros.h"
#include "gpio.h"
/*******************************************************************************
 *                             Global Variables			                       *
 *******************************************************************************/
//Keypad Keys
uint8 keys[NUMOFROW][NUMOFCOL]=KEYPAD_KEYS;
//For Keypad Pins
uint8 KeyRows[NUMOFROW]=KEY_ROWS;
uint8 KeyCols[NUMOFCOL]=KEY_COLS;
/*******************************************************************************
 *                            	Function Definition	                   			*
 *******************************************************************************/

/************************************************************************************
 * Function Name	: Keypad_init
 * Description		: Function to initialize keypad pins
 * Parameters (in)	: None
 * Return value		: None
 ************************************************************************************/
void Keypad_init(){
	uint32 count=0;
	GPIO_PinConfig_t Pin_Config={KeyCols[0],GPIO_OUTPUT_PP,GPIO_SPEED_2_MHZ};
	//Configure Columns pins as output pins
	for(count=0;count<NUMOFCOL;count++){
		Pin_Config.Pin_Num=KeyCols[count];
		MCAL_GPIO_init(KEYPAD_COL_PORT, &Pin_Config);
	}
	//Configure rows pins as input pull up
	Pin_Config.Pin_Mode=KEYPAD_ROWS_MODE;
	Pin_Config.Pin_Speed=GPIO_SPEED_INPUT;
	for(count=0;count<NUMOFROW;count++){
		Pin_Config.Pin_Num=KeyRows[count];
		MCAL_GPIO_init(KEYPAD_ROW_PORT, &Pin_Config);
	}
}
/************************************************************************************
 * Function Name	: Keypad_getKey
 * Description		: Function to get the pressed key in the keypad
 * Parameters (in)	: None
 * Return value		: Key value if pressed or KEYPAD_NOT_PRESSED
************************************************************************************/
uint8 Keypad_getKey(){
	uint8 row,col,i;
	for(col =0;col<NUMOFCOL;col++){
		//Make all columns pins logic (HIGH OR LOW)
		for(i=0;i<NUMOFCOL;i++){
			MCAL_GPIO_writePin(KEYPAD_COL_PORT,KeyCols[i] , ALL_COLS_LEVEL);
		}
		//Make one column pin by different logic (HIGH OR LOW)
		MCAL_GPIO_writePin(KEYPAD_COL_PORT,KeyCols[col], COL_LEVEL);
		for(row =0 ;row<NUMOFROW;row++){
			//When the key pressed the row connect with column
			if( MCAL_GPIO_readPin(KEYPAD_ROW_PORT, KeyRows[row]) == ROW_CHECK_LEVEL ){
				//when key  not pressed wait until key not pressed
				while(MCAL_GPIO_readPin(KEYPAD_ROW_PORT, KeyRows[row]) == PIN_HIGH);
				//return the key value
				return keys[row][col];
			}
		}
	}
	return KEYPAD_NOT_PRESSED;
}
