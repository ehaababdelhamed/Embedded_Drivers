/******************************************************************************
 * Module: LCD
 * File Name: lcd.h
 * Description: Header file for LCD driver
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

#ifndef LCD_DRIVER_LCD_H_
#define LCD_DRIVER_LCD_H_

#include "Platform_Types.h"


/*******************************************************************************
 *                             User Configuration                              *
 *******************************************************************************/

//Select (LCD_8_BIT_MODE - LCD_4_BIT_MODE)
#define LCD_BIT_MODE 	LCD_4_BIT_MODE
//Select (LCD_2X16 - LCD_4X16 - LCD_4X20)
#define LCD_TYPE		LCD_2X16

#define LCD_DATA_PORT  	GPIO_PORTA
//For LCD_8_BIT_MODE Only
#define LCD_D0_PIN	  	GPIO_PIN_0
#define LCD_D1_PIN	  	GPIO_PIN_1
#define LCD_D2_PIN	  	GPIO_PIN_2
#define LCD_D3_PIN	  	GPIO_PIN_3

#define LCD_D4_PIN	  	GPIO_PIN_4
#define LCD_D5_PIN	  	GPIO_PIN_5
#define LCD_D6_PIN	  	GPIO_PIN_6
#define LCD_D7_PIN	  	GPIO_PIN_7

#define LCD_RS_PORT		GPIO_PORTA
#define LCD_RS_PIN		GPIO_PIN_8

#define LCD_RW_PORT		GPIO_PORTA
#define LCD_RW_PIN		GPIO_PIN_9

#define LCD_EN_PORT		GPIO_PORTA
#define LCD_EN_PIN		GPIO_PIN_10

#define DELAY_FUNC_NAME(i) 	delay_ms(i)
/*******************************************************************************
 *                            Module Definition                            *
 *******************************************************************************/

#define LCD_8_BIT_MODE	(8U)
#define LCD_4_BIT_MODE	(4U)

#define LCD_2X16		(0U)
#define LCD_4X16		(1U)
#define LCD_4X20		(2U)

/********************** LCD Commands *******************************/
#define LCD_ENTRY_MODE 					(0x06)
//Clearing display without clearing DDRAM content)
#define LCD_DISPLAY_OFF_CURSER_OFF		(0x08)
#define LCD_DISPLAY_ON_CURSER_ON		(0x0E)
#define LCD_DISPLAY_ON_CURSER_OFF		(0x0C)
#define LCD_DISPLAY_ON_CURSER_BLINKING	(0x0F)
#define LCD_SHIFT_DISPLYAY_LEFT			(0x18)
#define LCD_SHIFT_DISPLYAY_RIGHT		(0x1C)
#define LCD_MOVE_CURSER_LEFT			(0x10)
#define LCD_MOVE_CURSER_RIGHT			(0x14)
//Clearing display and also clear DDRAM content
#define LCD_CLEAR_DISPLAY				(0x01)
#define LCD_RETURN_HOME					(0x02)

#define LCD_MODE_8_BITS_1_LINES			(0x30)
#define LCD_MODE_8_BITS_2_LINES			(0x38)
#define LCD_MODE_4_BITS_1_LINES			(0x20)
#define LCD_MODE_4_BITS_2_LINES			(0x28)
#define LCD_MODE_4_BITS_INIT			(0x32)
//==================================================================
#define LCD_FIRST_POSITION				(0x80)
#define LCD_Line_1_ADDRESS				(0x00)
#define LCD_Line_2_ADDRESS				(0x40)
#define LCD_Line_3_ADDRESS				(0x10)
#define LCD_Line_4_ADDRESS				(0x50)
#define LCD_delay(i)	DELAY_FUNC_NAME(i)
/*******************************************************************************
 *                      	Function Prototypes				                	*
 *******************************************************************************/
void LCD_init();
void LCD_sendCommand(uint8 command);
void LCD_displayCharacter(uint8 character);
void LCD_displayString(char* string);
void LCD_displayNumber(uint32 integerNum);
void LCD_cleanScreen();
void LCD_customCharacter(uint8 *custom);
void LCD_isBusy();
void LCD_goToSpecficPosition(uint8 line,uint8 position);


#endif /* LCD_DRIVER_LCD_H_ */
