 /******************************************************************************
 * Module: USART
 * File Name: USART.h
 * Description: Header file for USART driver for Atmega32
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/
#ifndef INC_USART_H_
#define INC_USART_H_

/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "Common_Macros.h"
#include "Platform_Types.h"
/*******************************************************************************
 *                             	USART Data Types                             	*
 *******************************************************************************/

//@REF: USART_Mode_e
typedef enum{
	USART_MODE_TX=1,
	USART_MODE_RX,
	USART_MODE_TX_RX
}USART_Mode_e;

//@REF: USART_BAUDRATEs_t
/*
 * For F_CPU 1-2 MHZ Preferred baud rate up to 9600
 * For F_CPU 4 MHZ Preferred baud rate up to 38400
 * For F_CPU 8 MHZ Preferred baud rate up to 76800
 * For F_CPU 16 MHZ Preferred baud rate up to 115200
 *
 */
typedef enum{
	BAUDRATE_2400=2400,BAUDRATE_4800=4800,BAUDRATE_9600=9600,BAUDRATE_14400=14400,
	BAUDRATE_19200=19200,BAUDRATE_28800=28800,BAUDRATE_38400=38400,
	BAUDRATE_57600=57600,BAUDRATE_76800=76800,BAUDRATE_115200=115200,
}USART_BoudRate_e;

//@REF: USART_DataBit
typedef enum{
	DATE_5B,DATE_6B,DATE_7B,DATE_8B,DATE_9B=7
}USART_DateLength_e;

//@REF: USART_Parity
typedef enum{
	PARITY_DISABLE,PARITY_EVEN=2,PARITY_ODD
}USART_Parity_e;

//@REF: USART_StopBit
typedef enum{
	STOP_1B,STOP_2B
}USART_StopBit_e;

typedef struct{
	USART_Mode_e USART_Mode;		//Specifies USART Mode as TX/RX
									//This parameter can be a value of @REF: USART_Mode
	uint32 Boud_Rate;				//Specifies baud rate of USART
									//This parameter can be a value of @REF: USART_BAUDRATEs_t
	USART_DateLength_e Data_Length;	//Specifies the Number of data bits of USART frame
									//This parameter can be a value of @REF: USART_DataBit
	USART_Parity_e Parity;			//Specifies the parity of USART  frame
									//This parameter can be a value of @REF: USART_Parity
	USART_StopBit_e Stop_Bit;		//Specifies the number of stop bits of USART  frame
									//This parameter can be a value of @REF: USART_StopBit
}USART_Config_t;


typedef enum{
	POOLING_ENABLE,
	POOLING_DISABLE,
}USART_mechanism_e;

//@REF: USART_IRQ_Types
typedef enum{
	USART_IRQ_ENABLE_UDRE=5,
	USART_IRQ_ENABLE_TXC,
	USART_IRQ_ENABLE_RXC
}USART_IRQ_e;

/*******************************************************************************
 *                             	USART Definition                             	*
 *******************************************************************************/
#define F_CPU (8000000U)
#define TerminateChar '\r'

/*******************************************************************************
 *					APIs Supported by "MCAL USART DRIVER" 	                   *
 *******************************************************************************/

/************************************************************************************
 * Function Name	: MCAL_USART_init
 * Description		: Functional responsible for Initialize the USART device by:
 * 1. Setup the Frame format like number of data bits, parity bit type and number of stop bits.
 * 2. Setup the USART baud rate.
 * 3. Enable the USART.
 * Parameters (in)	: USARTConfig- pointer to USART configuration parameters
 * [baud rate - Number of data bits -Parity type - Number of stop bits]
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_USART_init(USART_Config_t *USARTConfig);

/************************************************************************************
 * Function Name	: MCAL_USART_deInit
 * Description		: Functional responsible for reset the all USART register.
 * 					  Disable USART.
 * Parameters (in)	: None
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_USART_deInit();

/************************************************************************************
 * Function Name	: MCAL_USART_sendData
 * Description		: Functional responsible to send Data by USART.
 * Parameters (in)	: data : character to be send by USART
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_USART_sendData(uint16 data,USART_mechanism_e Mechanism);

/************************************************************************************
 * Function Name	: MCAL_USART_receiveData
 * Description		: Functional responsible to receive Data by USART.
 * Parameters (in)	: None
 * Return value		: Character to be received
 * Note				: period
 ************************************************************************************/
uint16 MCAL_USART_receiveData(USART_mechanism_e Mechanism);

/************************************************************************************
 * Function Name	: MCAL_USART_sendDataPeriodicCheck
 * Description		: Functional responsible to send Data by USART by checking Buffer.
 * Parameters (in)	: data : character to be send by USART
 * Return value		: Status of operation
 * Note				: None
 ************************************************************************************/
boolean MCAL_USART_sendDataPeriodicCheck(uint16 data);

/************************************************************************************
 * Function Name	: MCAL_USART_sendDataPeriodicCheck
 * Description		: Functional responsible to send Data by USART by checking Buffer.
 * Parameters (in)	: data : pointer to save receive data
 * Return value		: Status of operation
 * Note				: None
 ************************************************************************************/
boolean MCAL_USART_receiveDataPeriodicCheck(uint16 *data);

/************************************************************************************
 * Function Name	: MCAL_USART_sendString
 * Description		: Functional responsible to send string by USART.
 * Parameters (in)	: data : pointer to start character of string to be send by USART
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_USART_sendString(char* data);

/************************************************************************************
 * Function Name		: MCAL_USART_receiveCharacter
 * Description			: Functional responsible to receive string by USART.
 * Parameters (in/out)	: string : pointer to store received string
 * Return value			: None
 * Note					: None
 ************************************************************************************/
void MCAL_USART_receiveString(char *string);

/************************************************************************************
 * Function Name		: MCAL_USART_sendSInteger
 * Description			: Functional responsible to Send integer by USART.
 * Parameters (in/out)	: string : pointer to store received string
 * Return value			: None
 * Note					: None
 ************************************************************************************/
void MCAL_USART_sendSInteger(uint32 data);

/************************************************************************************
 * Function Name		: MCAL_USART_ReceiveInteger
 * Description			: Functional responsible to receive string by USART.
 * Parameters (in/out)	: string : pointer to store received string
 * Return value			: None
 * Note					: None
 ************************************************************************************/
uint32 MCAL_USART_ReceiveInteger();

/************************************************************************************
 * Function Name		: MCAL_USART_setCallBackFun
 * Description			: Functional responsible to set function that
 * 						  will be called when interrupt occur.
 * Parameters (in/out)	: IRQx - interrupt source
 * 						  callback - call back function
 * Return value			: None
 * Note					: None
 ************************************************************************************/
void MCAL_USART_setCallBackFun(USART_IRQ_e IRQx,void (*callback)());

/************************************************************************************
 * Function Name		: MCAL_USART_EnableInterrupt
 * Description			: Functional responsible to enable interrupt
 * Parameters (in/out)	: IRQx - interrupt source
 * Return value			: None
 * Note					: None
 ************************************************************************************/
void MCAL_USART_EnableInterrupt(USART_IRQ_e IRQx);

/************************************************************************************
 * Function Name		: MCAL_USART_DisableInterrupt
 * Description			: Functional responsible to Disable interrupt
 * Parameters (in/out)	: IRQx - interrupt source
 * Return value			: None
 * Note					: None
 ************************************************************************************/
void MCAL_USART_DisableInterrupt(USART_IRQ_e IRQx);


#endif /* INC_USART_H_ */
