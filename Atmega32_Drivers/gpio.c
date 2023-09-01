/******************************************************************************
 * Module: GPIO
 * File Name: gpio.c
 * Description: Source file for GPIO driver for Atmega32
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "gpio.h"
#include "Atmega32.h"

/*******************************************************************************
 *                            Module Macros	                   *
 *******************************************************************************/
#define GPIO_GET_ADDRESS(NAME,PORT_NUM)	(PORT_NUM == GPIO_PORTA)? NAME##A :\
										(PORT_NUM == GPIO_PORTB)? NAME##B :\
										(PORT_NUM == GPIO_PORTC)? NAME##C :\
										(PORT_NUM == GPIO_PORTD)? NAME##D :NULL

/*******************************************************************************
 *                            	Function Definition	                   			*
 *******************************************************************************/

/************************************************************************************
 * Function Name	: MCAL_GPIO_init
 * Description		: Function to initialize the Specific pin in specific GPIO module.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  pinConfig - Pointer to pin configuration date
 * Return value		: None
 * Note				:
 ************************************************************************************/

void MCAL_GPIO_init(uint8 GPIO_Portx,GPIO_PinConfig_t *pinConfig){
	boolean error=FALSE;				//to check the if inputs is valid
	volatile uint8 *GPIOx;				//Store Address of DDRx
	volatile uint8 *PORTx;				//Store Address of PORTx
	GPIOx=GPIO_GET_ADDRESS(DDR,GPIO_Portx);
	PORTx=GPIO_GET_ADDRESS(PORT,GPIO_Portx);
	if(GPIOx == NULL || pinConfig->Pin_Num < GPIO_PIN_0 || pinConfig->Pin_Num > GPIO_PIN_7){
		error=TRUE;
	}
	if(error == FALSE){
		CLEAR_BIT(*GPIOx,pinConfig->Pin_Num);
		*GPIOx |= ((pinConfig->Pin_Mode&(1U))<<pinConfig->Pin_Num);
		if(pinConfig->Pin_Mode == GPIO_INPUT_PU){
			SET_BIT(*PORTx,pinConfig->Pin_Num);
		}
	}
}
/************************************************************************************
 * Function Name	: MCAL_GPIO_deInit
 * Description		: Function to reset all registers of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * Return value		: None
 * Note				: None
 ************************************************************************************/

void MCAL_GPIO_deInit(uint8 GPIO_Portx){
	volatile uint8* DDRx;
	volatile uint8* PORTx;
	//Assign all registers to reset value
	DDRx=GPIO_GET_ADDRESS(DDR,GPIO_Portx);
	PORTx=GPIO_GET_ADDRESS(PORT,GPIO_Portx);
	*PORTx = 0X00;
	*DDRx = 0X00;
}

/************************************************************************************
 * Function Name	: MCAL_GPIO_readPin
 * Description		: Function to read specific pin of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Pin_Num	- Pin number according @REF:GPIO_PINS_NUMBER
 * Return value		: PinLevelType according  @REF:GPIO_PIN_LEVEL
 * Note				: None
 ************************************************************************************/
uint8 MCAL_GPIO_readPin(uint8 GPIO_Portx,uint8 Pin_Num){
	uint8 PinLevelType=PIN_LOW;			//To store the pin value
	boolean error=FALSE;				//To check the if inputs is valid
	volatile uint8 *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GET_ADDRESS(PIN,GPIO_Portx);
	if(GPIOx == NULL || Pin_Num< GPIO_PIN_0 || Pin_Num > GPIO_PIN_7){
		error=TRUE;
	}
	//if input valid >> Read the pin value from GPIO_IDR Register
	if(error == FALSE){
		if(BIT_IS_SET(*GPIOx,Pin_Num) == PIN_LOW ){
			PinLevelType = PIN_LOW;
		}else{
			PinLevelType = PIN_HIGH;
		}
	}
	return PinLevelType;
}


/************************************************************************************
 * Function Name	:  MCAL_GPIO_readPort
 * Description		: Function to read all port of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * Return value		: PortLevelType
 * Note				: None
 ************************************************************************************/
uint8 MCAL_GPIO_readPort(uint8 GPIO_Portx){
	uint8 PortLevelType=0;
	boolean error=FALSE;				//to check the if inputs is valid
	volatile uint8 *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GET_ADDRESS(PIN,GPIO_Portx);
	if(GPIOx == NULL ){
		error=TRUE;
	}
	//if input valid >> Read the port value from GPIO_IDR Register
	if(error == FALSE){
		PortLevelType = (uint8)(*GPIOx);
	}
	return PortLevelType;
}

/************************************************************************************
 * Function Name	: MCAL_GPIO_writePin
 * Description		: Function to write in specific pin of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Pin_Num	- Pin number according @REF:GPIO_PINS_NUMBER
 * 					  Pin_Value - Pin value according @REF:GPIO_PIN_LEVEL
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_GPIO_writePin(uint8 GPIO_Portx,uint8 Pin_Num,uint8 Pin_Value){
	boolean error=FALSE;				//to check the if inputs is valid
	volatile uint8 *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GET_ADDRESS(PORT,GPIO_Portx);
	if(GPIOx == NULL || Pin_Num < GPIO_PIN_0 || Pin_Num > GPIO_PIN_7){
		error=TRUE;
	}
	//if input valid >> Write the value in the required pin
	if(error == FALSE){
		if(Pin_Value == PIN_HIGH){
			SET_BIT(*GPIOx,Pin_Num);
		}else if(Pin_Value == PIN_LOW){
			CLEAR_BIT(*GPIOx,Pin_Num);
		}
	}
}

/************************************************************************************
 * Function Name	: MCAL_GPIO_writePort
 * Description		: Function to write in all port of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Port_Value - Port value
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_GPIO_writePort(uint8 GPIO_Portx,uint8 Port_Value){
	boolean error=FALSE;				//to check the if inputs is valid
	volatile uint8 *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GET_ADDRESS(PORT,GPIO_Portx);
	//if input valid >> Write the value in the required port
	if(GPIOx == NULL){
		error=TRUE;
	}
	if(error == FALSE){
		*GPIOx = (uint8)(Port_Value);
	}
}



/************************************************************************************
 * Function Name	: MCAL_GPIO_writeGroub
 * Description		: Function to write in Group Of pins of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Port_Value - Port value
 * 					  offset - To select the position of from LSB
					  mask 	 - To Mask group of bits From start position
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_GPIO_writeGroub(uint8 GPIO_Portx,uint8 mask,uint8 offset,uint8 value){
	boolean error=FALSE;				//to check the if inputs is valid
	volatile uint8 *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GET_ADDRESS(PORT,GPIO_Portx);
	//if input valid >> Write the value in the required port
	if(GPIOx == NULL){
		error=TRUE;
	}
	if(error == FALSE){
		*GPIOx &= ~(mask<<offset);
		*GPIOx |= (value & mask) << offset;
	}
}

/************************************************************************************
 * Function Name	: MCAL_GPIO_writeGroub
 * Description		: Function to read Group Of pins of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Port_Value - Port value
 * 					  offset - To select the position of from LSB
					  mask 	 - To Select Pins to read
 * Return value		: None
 * Note				: None
 ************************************************************************************/
uint8 MCAL_GPIO_readGroup(uint8 GPIO_Portx,uint8 mask,uint8 offset){
	uint8 PortLevelType=0;
	boolean error=FALSE;				//to check the if inputs is valid
	volatile uint8 *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GET_ADDRESS(PIN,GPIO_Portx);
	if(GPIOx == NULL ){
		error=TRUE;
	}
	//if input valid >> Read the port value from GPIO_IDR Register
	if(error == FALSE){
		PortLevelType = (((uint8)(*GPIOx) & (mask<<offset)) >> offset);
	}
	return PortLevelType;
}

/************************************************************************************
 * Function Name	: MCAL_GPIO_togglePin
 * Description		: Function to toggle specific pin of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Pin_Num	- Pin number according @REF:GPIO_PINS_NUMBER
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_GPIO_togglePin(uint8 GPIO_Portx,uint8 Pin_Num){
	boolean error=FALSE;				//to check the if inputs is valid
	volatile uint8 *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GET_ADDRESS(PORT,GPIO_Portx);
	//if input valid >> Write the value in the required port
	if(GPIOx == NULL){
		error=TRUE;
	}
	if(error == FALSE){
		TOGGLE_BIT(*GPIOx,Pin_Num);
	}
}
//=============================================================================================
