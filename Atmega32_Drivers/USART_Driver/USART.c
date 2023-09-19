/******************************************************************************
 * Module: USART
 * File Name: USART.c
 * Description: Source file for USART driver for Atmega32
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "Atmega32.h"
#include <math.h>
#include "lcd.h"
#include "USART.h"
#include "Atmega32_interrupt.h"

void (*GP_callBackFUNC[3])();
/*******************************************************************************
 *                            	Function Definition	                   			*
 *******************************************************************************/
/************************************************************************************
 * Function Name	: MCAL_USART_init
 * Description		: Functional responsible for Initialize the USART device by:
 * 1. Enable the USART.
 * 2. Setup the Frame format >> data bits, parity bit type and number of stop bits.
 * 3. Setup the USART baud rate.
 * Parameters (in)	: USARTConfig- pointer to USART configuration parameters
 * [baud rate - Number of data bits -Parity type - Number of stop bits]
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_USART_init(USART_Config_t *USARTConfig){
	uint16 UBRR_Value;

	UCSRA->bit.U2X = 1; //Asynchronous Double Speed Mode(U2X = 1)

	//1. Enable the USART.
	UCSRB->bit.TXEN_RXEN = USARTConfig->USART_Mode;

	//2. Setup the Frame format >> data bits, parity bit type and number of stop bits.

	/*The UCSRC Register shares the same I/O location as the UBRRH Register. See the
	 * “Accessing UBRRH/ UCSRC Registers” on page 156 section which describes how to
	 * access this register.
	 * • Bit 7 – URSEL: Register Select
	 * This bit selects between accessing the UCSRC or the UBRRH Register
	 * 1 For UCSRC , 0 for UBRRH */
	UCSRC =(1<<7)|(((USARTConfig->Parity)&0x03)<<4)|(((USARTConfig->Stop_Bit)&0x01)<<3)|((USARTConfig->Data_Length&0x03)<<1);
	if(USARTConfig->Data_Length  == USART_DATE_9B){
		UCSRB->bit.UCSZ2 = 1; //for 9 data bits
	}

	//3. Setup the USART baud rate.
	UBRR_Value = (uint16)(((float)F_CPU/(8.0*USARTConfig->Boud_Rate))-1);
	UBRRH = ((UBRR_Value & 0xff00)>>8);
	UBRRL = UBRR_Value;
}

/************************************************************************************
 * Function Name	: MCAL_USART_deInit
 * Description		: Functional responsible for reset the all USART register.
 * 					  Disable USART.
 * Parameters (in)	: None
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_USART_deInit(){
	//reset the all USART register;
	UCSRA->ALL_REG = 0x20;
	UCSRB->ALL_REG = 0x00;
	UCSRC = 0x86;
	UBRRL = 0x00;
	UBRRH = 0x00;

}
/************************************************************************************
 * Function Name	: MCAL_USART_sendData
 * Description		: Functional responsible to send Data by USART.
 * Parameters (in)	: data : Data to be send by USART
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_USART_sendData(uint16 data,USART_mechanism_e Mechanism){
	if(Mechanism == USART_POOLING_ENABLE){
		while(UCSRA->bit.UDRE != TRUE ){}
	}
	UDR = (uint8)data;

}

/************************************************************************************
 * Function Name	: MCAL_USART_sendData
 * Description		: Functional responsible to receive character by USART.
 * Parameters (in)	: None
 * Return value		: Character to be received
 * Note				: None
 ************************************************************************************/
uint8 MCAL_USART_receiveData(USART_mechanism_e Mechanism){
	if(Mechanism == USART_POOLING_ENABLE){
		while(UCSRA->bit.RXC != TRUE ){}
	}
	return UDR;
}

/************************************************************************************
 * Function Name	: MCAL_USART_sendDataPeriodicCheck
 * Description		: Functional responsible to send Data by USART by checking Buffer.
 * Parameters (in)	: data : character to be send by USART
 * Return value		: Status of operation
 * Note				: None
 ************************************************************************************/
boolean MCAL_USART_sendDataPeriodicCheck(uint16 data){
	boolean status = FALSE;
	if(UCSRA->bit.UDRE == TRUE){
		UDR = data;
		status = TRUE;
	}
	return status;
}
/************************************************************************************
 * Function Name	: MCAL_USART_sendDataPeriodicCheck
 * Description		: Functional responsible to send Data by USART by checking Buffer.
 * Parameters (in)	: data : pointer to save receive data
 * Return value		: Status of operation
 * Note				: None
 ************************************************************************************/
boolean MCAL_USART_receiveDataPeriodicCheck(uint16 *data){
	boolean status = FALSE;
	if(UCSRA->bit.RXC == TRUE){
		*data = UDR;
		status = TRUE;
	}
	return status;
}
/************************************************************************************
 * Function Name	: MCAL_USART_sendString
 * Description		: Functional responsible to send string by USART.
 * Parameters (in)	: data : pointer to start character of string to be send by USART
 * Return value		: None
 * Note				: Terminate string by character '#'
 ************************************************************************************/
void MCAL_USART_sendString(char* data){
	uint32 count=0;
	//Send Character by character
	while(data[count]!= '\0'){
		MCAL_USART_sendData(data[count],USART_POOLING_ENABLE);
		count++;
	}
}
/************************************************************************************
 * Function Name		: MCAL_USART_sendData
 * Description			: Functional responsible to receive string by USART.
 * Parameters (in/out)	: string : pointer to store received string
 * Return value			: None
 * Note					: None
 ************************************************************************************/
void MCAL_USART_receiveString(char *string){
	uint8 count=0;
	do{
		string[count]=MCAL_USART_receiveData(USART_POOLING_ENABLE);
	}while(string[count++] != '\0');
}

/************************************************************************************
 * Function Name		: MCAL_USART_sendSInteger
 * Description			: Functional responsible to Send integer by USART.
 * Parameters (in/out)	: string : pointer to store received string
 * Return value			: None
 * Note					: None
 ************************************************************************************/
void MCAL_USART_sendSInteger(uint32 data){
	uint8 *PtoCh = (uint8*)&data;
	uint8 count;
	//Send Character by character
	for(count=0;count<4;count++){
		MCAL_USART_sendData(*PtoCh,USART_POOLING_ENABLE);
		PtoCh++;
	};
}

/************************************************************************************
 * Function Name		: MCAL_USART_ReceiveInteger
 * Description			: Functional responsible to receive string by USART.
 * Parameters (in/out)	: string : pointer to store received string
 * Return value			: None
 * Note					: None
 ************************************************************************************/
uint32 MCAL_USART_ReceiveInteger(){
	uint8 count;
	uint32 intNum;
	uint8 *PtoCh = (uint8*)&intNum;
	//Send Character by character
	for(count=0;count<4;count++){
		*PtoCh=MCAL_USART_receiveData(USART_POOLING_ENABLE);
		PtoCh++;
	};
	return intNum;
}
/************************************************************************************
 * Function Name		: MCAL_USART_setCallBackFun
 * Description			: Functional responsible to set function that
 * 						  will be called when interrupt occur.
 * Parameters (in/out)	: IRQx - interrupt source
 * 						  callback - call back function
 * Return value			: None
 * Note					: None
 ************************************************************************************/
void MCAL_USART_setCallBackFun(USART_IRQ_e IRQx,void (*callback)()){
	if(callback != NULL){
		GP_callBackFUNC[IRQx-5] = callback;
	}

}
/************************************************************************************
 * Function Name		: MCAL_USART_EnableInterrupt
 * Description			: Functional responsible to enable interrupt
 * Parameters (in/out)	: IRQx - interrupt source
 * Return value			: None
 * Note					: None
 ************************************************************************************/
void MCAL_USART_EnableInterrupt(USART_IRQ_e IRQx){
	SET_BIT(UCSRB->ALL_REG,IRQx);
}
/************************************************************************************
 * Function Name		: MCAL_USART_DisableInterrupt
 * Description			: Functional responsible to Disable interrupt
 * Parameters (in/out)	: IRQx - interrupt source
 * Return value			: None
 * Note					: None
 ************************************************************************************/
void MCAL_USART_DisableInterrupt(USART_IRQ_e IRQx){
	CLEAR_BIT(UCSRB->ALL_REG,IRQx);
}

/*******************************************************************************
 *                             			ISR			                           *
 *******************************************************************************/
/* USART Data Register Empty interrupt */
ISR(USART_UDRE_vect){
	if(GP_callBackFUNC[0] != NULL){
		(*GP_callBackFUNC[0])();
	}
}

/* USART, RX Complete interrupt*/
ISR(USART_RXC_vect){
	if(GP_callBackFUNC[1] != NULL){
		(*GP_callBackFUNC[1])();
	}
}

/* USART, TX Complete interrupt*/
ISR(USART_TXC_vect){
	if(GP_callBackFUNC[2] != NULL){
		(*GP_callBackFUNC[2])();
	}
}
