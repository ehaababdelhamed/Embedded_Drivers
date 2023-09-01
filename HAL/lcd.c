/******************************************************************************
 * Module: LCD
 * File Name: lcd.c
 * Description: Source file for LCD driver
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/
#include "lcd.h"
#include <stdlib.h>
#include "stm32f103x6.h"
#include "Common_Macros.h"
#include "gpio.h"
#include "delay.h"

/*******************************************************************************
 *                      Private Function Prototypes			                   *
 *******************************************************************************/
static void LCD_enablePulse();

/*******************************************************************************
 *                      Function Definition                                    *
 *******************************************************************************/

/************************************************************************************
 * Function Name	: LCD_init
 * Description		: Function to initialize LCD driver
 * 					  Setup LCD data,EN,RS,RW Pin
 * 					  Setup LCD mode(8 bit or 4 bit)
 * Parameters (in)	: None
 * Return value		: None
 ************************************************************************************/
void LCD_init(){
	//Configure EN,RS,RW Pins as output pins
	GPIO_PinConfig_t Pin_Config={LCD_RS_PIN,GPIO_OUTPUT_PP,GPIO_SPEED_2_MHZ};
	MCAL_GPIO_init(LCD_RS_PORT, &Pin_Config);
	Pin_Config.Pin_Num = LCD_RW_PIN;
	MCAL_GPIO_init(LCD_RW_PORT, &Pin_Config);
	Pin_Config.Pin_Num = LCD_EN_PIN;
	MCAL_GPIO_init(LCD_EN_PORT, &Pin_Config);
	//Configure Data Pins as output pins
#if  LCD_BIT_MODE == LCD_8_BIT_MODE
	Pin_Config.Pin_Num = LCD_D0_PIN;
	MCAL_GPIO_init(LCD_DATA_PORT, &Pin_Config);
	Pin_Config.Pin_Num=LCD_D1_PIN;
	MCAL_GPIO_init(LCD_DATA_PORT, &Pin_Config);
	Pin_Config.Pin_Num=LCD_D2_PIN;
	MCAL_GPIO_init(LCD_DATA_PORT, &Pin_Config);
	Pin_Config.Pin_Num=LCD_D3_PIN;
	MCAL_GPIO_init(LCD_DATA_PORT, &Pin_Config);
#endif
	Pin_Config.Pin_Num=LCD_D4_PIN;
	MCAL_GPIO_init(LCD_DATA_PORT, &Pin_Config);
	Pin_Config.Pin_Num=LCD_D5_PIN;
	MCAL_GPIO_init(LCD_DATA_PORT, &Pin_Config);
	Pin_Config.Pin_Num=LCD_D6_PIN;
	MCAL_GPIO_init(LCD_DATA_PORT, &Pin_Config);
	Pin_Config.Pin_Num=LCD_D7_PIN;
	MCAL_GPIO_init(LCD_DATA_PORT, &Pin_Config);
	LCD_delay(20); 		//LCD Power On delay
#if  LCD_BIT_MODE == LCD_8_BIT_MODE
	LCD_sendCommand(LCD_MODE_8_BITS_2_LINES);
#elif LCD_BIT_MODE == LCD_4_BIT_MODE
	LCD_sendCommand(LCD_MODE_4_BITS_INIT);
	LCD_sendCommand(LCD_MODE_4_BITS_2_LINES);
#endif
	LCD_delay(5);
	LCD_sendCommand(LCD_DISPLAY_ON_CURSER_BLINKING);
	LCD_sendCommand(LCD_CLEAR_DISPLAY);
	LCD_sendCommand(LCD_ENTRY_MODE);
}

/************************************************************************************
 * Function Name	: LCD_sendCommand
 * Description		: Function to send command to LCD
 * Parameters (in)	: command : command to be send
 * Return value		: None
 ************************************************************************************/
void LCD_sendCommand(uint8 command){
	MCAL_GPIO_writePin(LCD_RW_PORT, LCD_RW_PIN, PIN_LOW); 	//LCD Write Mode
	LCD_delay(1);											//delay for RW --|__ failing
	MCAL_GPIO_writePin(LCD_RS_PORT, LCD_RS_PIN, PIN_LOW);	//Command Mode
	LCD_delay(1);											//delay for RS --|__ failing
#if  LCD_BIT_MODE == LCD_8_BIT_MODE
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D0_PIN, GET_BIT(command,0));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D1_PIN, GET_BIT(command,1));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D2_PIN, GET_BIT(command,2));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D3_PIN, GET_BIT(command,3));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D4_PIN, GET_BIT(command,4));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D5_PIN, GET_BIT(command,5));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D6_PIN, GET_BIT(command,6));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D7_PIN, GET_BIT(command,7));
	LCD_delay(1);
	LCD_enablePulse();
#elif  LCD_BIT_MODE == LCD_4_BIT_MODE
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D4_PIN, GET_BIT(command,4));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D5_PIN, GET_BIT(command,5));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D6_PIN, GET_BIT(command,6));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D7_PIN, GET_BIT(command,7));
	LCD_delay(1);
	LCD_enablePulse();
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D4_PIN, GET_BIT(command,0));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D5_PIN, GET_BIT(command,1));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D6_PIN, GET_BIT(command,2));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D7_PIN, GET_BIT(command,3));
	LCD_delay(1);
	LCD_enablePulse();
#endif
}

/************************************************************************************
 * Function Name	: LCD_displayCharacter
 * Description		: Function to display character on LCD screen
 * Parameters (in)	: character -   character to be displayed on LCD
 * Return value		: None
 ************************************************************************************/
void LCD_displayCharacter(uint8 character){
	MCAL_GPIO_writePin(LCD_RW_PORT, LCD_RW_PIN, PIN_LOW); 	//LCD Write Mode
	LCD_delay(1);											//delay for RW --|__ failing
	MCAL_GPIO_writePin(LCD_RS_PORT, LCD_RS_PIN, PIN_HIGH);	//LCD Data Mode
	LCD_delay(1);											//delay for RS --|__ failing
#if  LCD_BIT_MODE == LCD_8_BIT_MODE
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D0_PIN, GET_BIT(character,0));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D1_PIN, GET_BIT(character,1));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D2_PIN, GET_BIT(character,2));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D3_PIN, GET_BIT(character,3));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D4_PIN, GET_BIT(character,4));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D5_PIN, GET_BIT(character,5));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D6_PIN, GET_BIT(character,6));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D7_PIN, GET_BIT(character,7));
	LCD_delay(1);
	LCD_enablePulse();
#elif  LCD_BIT_MODE == LCD_4_BIT_MODE
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D4_PIN, GET_BIT(character,4));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D5_PIN, GET_BIT(character,5));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D6_PIN, GET_BIT(character,6));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D7_PIN, GET_BIT(character,7));
	LCD_delay(1);
	LCD_enablePulse();
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D4_PIN, GET_BIT(character,0));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D5_PIN, GET_BIT(character,1));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D6_PIN, GET_BIT(character,2));
	MCAL_GPIO_writePin(LCD_DATA_PORT,LCD_D7_PIN, GET_BIT(character,3));
	LCD_delay(1);
	LCD_enablePulse();
#endif
}

/************************************************************************************
 * Function Name	: LCD_displayString
 * Description		: Function to display string on LCD screen
 * Parameters (in)	: string - string to be displayed on LCD
 * Return value		: None
 ************************************************************************************/
void LCD_displayString(char* string){
	uint32 count;
	if(string != NULL){
		for(count=0;string[count]!= '\0';count++){
			LCD_displayCharacter(string[count]);
			if(count == 15){
				LCD_goToSpecficPosition(2,0);
			}
#if (LCD_TYPE == LCD_4X16 || LCD_TYPE == LCD_4X20)
			if(count == 31){
				LCD_goToSpecficPosition(3,0);
			}
			if(count == 47){
				LCD_goToSpecficPosition(4,0);
			}
#endif
		}
	}else{
		LCD_displayString("NULL STRING ERROR");
	}
}

/************************************************************************************
 * Function Name	: LCD_displayNumber
 * Description		: Function to display integer number on LCD screen
 * Parameters (in)	: integerNum - integer number to be displayed on LCD
 * Return value		: None
 ************************************************************************************/
void LCD_displayNumber(uint32 integerNum){
	char buffer[16];
	itoa(integerNum,(char*)buffer,10);
	LCD_displayString(buffer);
}

/************************************************************************************
 * Function Name	: LCD_cleanScreen
 * Description		: Function to clean LCD screen
 * Parameters (in)	: integerNum - integer number to be displayed on LCD
 * Return value		: None
 ************************************************************************************/
void LCD_cleanScreen(){
	LCD_sendCommand(LCD_CLEAR_DISPLAY);
}

/************************************************************************************
 * Function Name	: LCD_goToSpecficPosition
 * Description		: Function to move courser to specific position
 * Parameters (in)	: line - line number
 * 					  position - position number
 * Return value		: None
 ************************************************************************************/
void LCD_goToSpecficPosition(uint8 line,uint8 position){
	uint8 RequriedPostion = LCD_FIRST_POSITION;
#if	(LCD_TYPE == LCD_2X16)
	if(line < 0 || line >=2 || position < 0 || position >= 16 ){
		LCD_displayString("ERROR POSITION");
		return;
	}
#elif (LCD_TYPE == LCD_4X16)
	if(line < 0 || line >=4 || position < 0 || position >= 16 ){
		LCD_displayString("ERROR POSITION");
		return;
	}
#elif (LCD_TYPE == LCD_4X20)
	if(line < 0 || line >=4 || position < 0 || position >= 20 ){
		LCD_displayString("ERROR POSITION");
		return;
	}
#endif
	switch(line){
	case 0:
		RequriedPostion|=LCD_Line_1_ADDRESS;
		break;
	case 1:
		RequriedPostion|=LCD_Line_2_ADDRESS;
		break;

#if (LCD_TYPE == LCD_4X16 || LCD_TYPE == LCD_4X20)
	case 2:
		RequriedPostion|=LCD_Line_3_ADDRESS;
		break;
	case 3:
		RequriedPostion|=LCD_Line_4_ADDRESS;
		break;
#endif
	}
	RequriedPostion += position;
	LCD_sendCommand(RequriedPostion);
}

/************************************************************************************
 * Function Name	: LCD_customCharacter
 * Description		: Function to Custom Shape on LCD
 * Parameters (in)	: line - line number
 * 					  position - position number
 * Return value		: None
 ************************************************************************************/
void LCD_customCharacter(uint8 *custom){
	LCD_sendCommand(0x40);
	LCD_displayCharacter(custom[0]);
	LCD_displayCharacter(custom[2]);
	LCD_displayCharacter(custom[3]);
	LCD_displayCharacter(custom[4]);
	LCD_displayCharacter(custom[5]);
	LCD_displayCharacter(custom[6]);
	LCD_displayCharacter(custom[7]);
	LCD_displayCharacter(custom[8]);
	LCD_sendCommand(LCD_FIRST_POSITION);
	LCD_delay(5);
	LCD_displayCharacter(0);
}


/************************************************************************************
 * Function Name	: LCD_enablePulse
 * Description		: Private Function to send pulse in enable line
 * Parameters (in)	: None
 * Return value		: None
 ************************************************************************************/
static void LCD_enablePulse(){
	MCAL_GPIO_writePin(LCD_EN_PORT, LCD_EN_PIN, PIN_HIGH);
	LCD_delay(1);						//delay for EN _|-- rising
	MCAL_GPIO_writePin(LCD_EN_PORT, LCD_EN_PIN, PIN_LOW);
	LCD_delay(1);						//delay for EN  --|__ failing
}
