/******************************************************************************
 * Module: EXTI
 * File Name: EXTI.c
 * Description: Source file for EXTI MCal driver for stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/
#include "EXTI.h"
#include "gpio.h"
#include "Atmega32.h"
#include <avr/interrupt.h>

/*******************************************************************************
 *                            Global variables	                   *
 *******************************************************************************/
//to store the call back function that set by call back function
void (*g_EXTI_callBackfunc[INT2])(void);

/*******************************************************************************
 *                            Private Function Prototypes	                   *
 *******************************************************************************/
static void EXTI_Update(EXTI_Config_t* INTConfig);

/*******************************************************************************
 *                      Function Definition                                    *
 *******************************************************************************/
/************************************************************************************
 * Function Name	: MCAL_EXTI_init
 * Description		: Function to initialize EXTI driver by Setup GPIO Pin,trigger edge
 * 					  ,map EXTI to GPIO pin
 * Parameters (in)	: INTConfig - Pointer to configuration data :EXTI Pin based on @REF EXTI_Defin
 * 					  and trigger edge
 * Return value		:
 * Note 			: INT2 has two trigger mode only [RISING_EDGE - FAILING_EDGE]
 ************************************************************************************/
void MCAL_EXTI_init(EXTI_Config_t* INTConfig){
	EXTI_Update(INTConfig);
}
/************************************************************************************
 * Function Name	: MCAL_EXTI_deInit
 * Description		: Function to reset all EXTI registers
 * 					  Disable EXTI From NVIC
 * Parameters (in)	: None
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_EXTI_deInit(){
	//Assign Reset Value to all registers
	MCUCR &= 0xf0; 	//CLEAR ISC11 ISC10 ISC01 ISC00 Bits[0-3]
	MCUCSR &= ~(1<<6); //Clear ISC2 bit [6]
	GICR &=0x1f;		//Clear INT1 INT0 INT2 bits [5-7]
}
/************************************************************************************
 * Function Name	: MCAL_EXTI_enable
 * Description		: Function to enable Specific EXTI
 * Parameters (in)	: INT_Number -EXTI Number to be enabled
 * Return value		: None
 * Note				: The selected EXTI must be initialized before enable
 ************************************************************************************/
void MCAL_EXTI_enable(EXTI_NUM_e INT_Number){
	//Enable interrupt request for EXTIx
	/*General Interrupt Control Register – GICR:
	 * Bit 5 – INT2: External Interrupt Request 2 Enable
	 * Bit 6 – INT0: External Interrupt Request 0 Enable
	 * Bit 7 – INT1: External Interrupt Request 1 Enable
	 * When the x bit is set (one) and the I-bit in the Status Register (SREG) is set (one),
	 * the external pin interrupt is enabled
	 * .*/
	if(INT_Number >= INT_0 && INT_Number <= INT_2){
		switch(INT_Number){
		case INT_0:
			SET_BIT(GICR , (6));
			break;
		case INT_1:
			SET_BIT(GICR , (7));
			break;
		case INT_2:
			SET_BIT(GICR , (5));
			break;
		}
	}
}
/************************************************************************************
 * Function Name	: MCAL_EXTI_disable
 * Description		: Function to disable Specific EXTI
 * Parameters (in)	: INT_Number -EXTI Number to be disabled
 * Return value		: None
 ************************************************************************************/
void MCAL_EXTI_disable(EXTI_NUM_e INT_Number){
	//Disable interrupt request for EXTIx
	/*General Interrupt Control Register – GICR:
	 * Bit 5 – INT2: External Interrupt Request 2 Enable
	 * Bit 6 – INT0: External Interrupt Request 0 Enable
	 * Bit 7 – INT1: External Interrupt Request 1 Enable
	 * .*/
	if(INT_Number >= INT_0 && INT_Number <= INT_2){
		switch(INT_Number){
		case INT_0:
			CLEAR_BIT(GICR , (6));
			break;
		case INT_1:
			CLEAR_BIT(GICR , (7));
			break;
		case INT_2:
			CLEAR_BIT(GICR , (5));
			break;
		}
	}
}
/************************************************************************************
 * Function Name	: MCAL_EXTI_update
 * Description		: Function to Update EXTI driver by update GPIO Pin,trigger edge
 * 					  and mapping EXTI to GPIO pin.
 * Parameters (in)	: INTConfig - Pointer to configuration data :EXTI Pin based on @REF EXTI_Defin
 * 					  and trigger edge
 * Return value		: None
 * Note				: Stm32f103x6 supported GPIOA,GPIOB,GPIOC,GPIOD,GPIOE
 * 					  but LQF48 Package supported fully GPIOA,GPIOB
 * 					  and partially GPIOC[13-15] and GPIOD[0-1]
 ************************************************************************************/
void MCAL_EXTI_update(EXTI_Config_t* INTConfig){
	EXTI_Update(INTConfig);
}
/************************************************************************************
 * Function Name	: MCAL_EXTI_setCallBackFun
 * Description		: Function to setup call back function that be called  when interrupt occur
 * Parameters (in)	: INT_Number -EXTI Number to Set call back function for it
 * Return value		: None
 ************************************************************************************/
void MCAL_EXTI_setCallBackFun(EXTI_NUM_e INT_Number ,void (*callBackFunc)(void)){
	//set Call back function to global pointer to function
	if(callBackFunc != NULL || (INT_Number >= INT0 && INT_Number <= INT2) ){
		g_EXTI_callBackfunc[INT_Number]=callBackFunc;
	}
}

//=====================================================================================
/************************************************************************************
 * Function Name	: EXTI_Update
 * Description		: Private Function that configure EXTI driver
 * Parameters (in)	: INTConfig - Pointer to configuration data :EXTI Pin based on @REF EXTI_Defin
 * 					  and trigger edge
 * Return value		: None
 ************************************************************************************/
static void EXTI_Update(EXTI_Config_t* INTConfig){
	//1-Configure GPIO to be AF input
	GPIO_PinConfig_t pinConfig={INTConfig->EXTI_Pin.GPIO_Pin,GPIO_INPUT_FLOATING};
	boolean error = FALSE;
	if(INTConfig == NULL){
		error = TRUE;
	}
	if( error == FALSE){
		MCAL_GPIO_init(INTConfig->EXTI_Pin.GPIO_Port,&pinConfig);
		switch(INTConfig->EXTI_Pin.INT_Num){
		case INT_0:
		case INT_1:
			/*The MCUCR Register contains control bits for interrupt 0,1 sense control
			 * 0 0 The low level of INT1 generates an interrupt request.
			 * 0 1 Any logical change on INT1 generates an interrupt request.
			 * 1 0 The falling edge of INT1 generates an interrupt request.
			 * 1 1 The rising edge of INT1 generates an interrupt request
			 * */
			//Set the required edge in [ISC] bits
			MCUCR &= ((0xFC)<<(INTConfig->EXTI_Pin.INT_Num*2));
			//Configure Triggered Edge
			MCUCR |= ((INTConfig->Edge & 0x03)<<(INTConfig->EXTI_Pin.INT_Num*2));
			break;
		case INT_2:
			/*MCUCSR >> Bit 6 – ISC2: Interrupt Sense Control 2
			 *  If ISC2 is written to zero, a falling edge on INT2 activates the interrupt.
			 *  If ISC2 is written to one, a rising edge on INT2 activates the interrupt.
			 */
			if(INTConfig->Edge == FAILING_EGDE || INTConfig->Edge == RAISING_EDGE){
				MCUCSR |= ((INTConfig->Edge&(0x01))<<6);
			}
			break;
		}
	}
}

/*******************************************************************************
 *                     				ISR		                                   *
 *******************************************************************************/
ISR(INT0_vect){
	(*g_EXTI_callBackfunc[INT_0])();
}

ISR(INT1_vect){
	(*g_EXTI_callBackfunc[INT_1])();
}
ISR(INT2_vect){
	(*g_EXTI_callBackfunc[INT_2])();
}

//=========================================================================================
