/******************************************************************************
 * Module: SPI
 * File Name: SPI.c
 * Description: Source file for SPI driver for Stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "Atmega32.h"
#include "Atmega32_interrupt.h"
#include "SPI.h"
#include "GPIO.h"

/*******************************************************************************
 *								Global Variables	                   			*
 *******************************************************************************/
static SPI_Config_t Global_SPI_Config;
static void (*GP_SPI_CallBackFunc)() = NULL;

/*******************************************************************************
 *                            	Function Definition	                   			*
 *******************************************************************************/
/************************************************************************************
 * Function Name	: MCAL_SPI_init
 * Description		: Functional responsible for Initialize the SPI device by:
 * 1. Setup the SPI baud rate prescaler
 * 2. The data order (MSB-first or LSB-first).
 * 3. Select the clock polarity
 * 4. Select the clock phase
 * 5. Setup the SPI device mode Master/Slave.
 * 6. Select SPI mechanism polling/interrupt
 * 7. Enable SPI
 * Parameters (in)	: SPIConfig- pointer to SPI configuration parameters
 * Return value		: None
 * Note				: Stm32f103x6 Support only SPI1 and SPI2.
 * 					  We should initialize module before use the remain function.
 ************************************************************************************/
void MCAL_SPI_init(SPI_Config_t *SPIConfig){
	if(SPIConfig !=NULL){
		Global_SPI_Config = (SPI_Config_t)(*SPIConfig);
		SPCR->bit.SPE = FALSE; //Disable SPI

		//1. Setup the SPI baud rate prescaler, in bits SPR0 SPR1 at SPCR and bit SPI2X at SPSR
		SPCR->bit.SPR = (SPIConfig->SPI_Prescaler & 0x3);
		SPSR->bit.SPI2X = ((SPIConfig->SPI_Prescaler >> 2) & (1U));

		//2. The data order (MSB-first or LSB-first).c
		SPCR->bit.DORD = SPIConfig->Data_Order;

		//3. Select the clock polarity
		SPCR->bit.CPOL = SPIConfig->Clock_Polarity;

		//4. Select the clock phase
		SPCR->bit.CPHA = SPIConfig->Clock_Phase;

		//5. Setup the SPI device mode Master/Slave.
		SPCR->bit.MSTR = SPIConfig->Device_Mode;

		//6. Select SPI mechanism polling/interrupt
		SPCR->bit.SPIE = SPIConfig->Mechanism;

		//7. Enable SPI
		SPCR->bit.SPE = TRUE;
	}
}


/************************************************************************************
 * Function Name	: MCAL_SPI_deInit
 * Description		: Functional responsible  reset all the SPI Registers
 * 					  Disable SPI interrupt from NVIC
 * Parameters (in)	: None
 * Return value		: None
 ************************************************************************************/
void MCAL_SPI_deInit(){
	SPCR->ALL_REG = 0x00;
	SPSR->bit.SPI2X = 0;
	SPDR = 0x00;
}


/************************************************************************************
 * Function Name	: MCAL_SPI_sendData
 * Description		: Functional responsible  to send data from SPI
 * Parameters (in)	: TxBuffer - Data to be send
 * Return value		: None
 ************************************************************************************/
void MCAL_SPI_sendData(uint8 TxBuffer){
	SPDR = TxBuffer;
	if(Global_SPI_Config.Mechanism == SPI_POLLING){
		while(SPSR->bit.SPIF != TRUE);
	}
}


/************************************************************************************
 * Function Name	: MCAL_SPI_sendData
 * Description		: Functional responsible  to send data from SPI
 * Parameters (in)	: None
 * Return value		: RxBuffer - Data that is received
 ************************************************************************************/
uint8 MCAL_SPI_receiveData(){
	uint8 RxBuffer=0;
	if(Global_SPI_Config.Mechanism == SPI_POLLING){
		while(SPSR->bit.SPIF != TRUE);
	}
	RxBuffer = SPDR;
	return RxBuffer;
}

/************************************************************************************
 * Function Name	: MCAL_SPI_sendReceiceData
 * Description		: Functional responsible  to send and receive data from SPI
 * Parameters (in)	: TxBuffer - data to be send
 * Return value		: RxBuffer - data that is received
 ************************************************************************************/
uint8 MCAL_SPI_sendReceiveData(uint8 TxBuffer){
	SPDR = TxBuffer;

	if(Global_SPI_Config.Mechanism == SPI_POLLING){
		while(SPSR->bit.SPIF != TRUE);
	}
	return SPDR;

}


/************************************************************************************
 * Function Name	: MCAL_SPI_GPIO_SET_PINS
 * Description		: Functional responsible to setup GPIO pin for SPI
 * Parameters (in)	: None
 * Return value		: None
 ************************************************************************************/
void MCAL_SPI_GPIO_SET_PINS(){
	GPIO_PinConfig_t Pin_Config;
	// (SS) PB4	(MOSI) PB5	(MISO) PB6	(SCK) PB7
	switch(Global_SPI_Config.Device_Mode){
	case SPI_MASTER:
		//Configure  (SS) PB4 Pin
		Pin_Config = (GPIO_PinConfig_t){GPIO_PIN_4,GPIO_OUTPUT_PP};
		MCAL_GPIO_init(GPIO_PORTB, &Pin_Config);
		MCAL_GPIO_writePin(GPIO_PORTB, GPIO_PIN_4, PIN_HIGH);

		//Configure  (MOSI) PB5 Pin
		Pin_Config = (GPIO_PinConfig_t){GPIO_PIN_5,GPIO_OUTPUT_PP};
		MCAL_GPIO_init(GPIO_PORTB, &Pin_Config);

		//Configure  (MISO) PB6 Pin
		Pin_Config = (GPIO_PinConfig_t){GPIO_PIN_6,GPIO_INPUT_FLOATING};
		MCAL_GPIO_init(GPIO_PORTB, &Pin_Config);

		//Configure  (SCK) PB7 Pin
		Pin_Config = (GPIO_PinConfig_t){GPIO_PIN_7,GPIO_OUTPUT_PP};
		MCAL_GPIO_init(GPIO_PORTB, &Pin_Config);
		break;
	case SPI_SLAVE:
		//Configure  (SS) PB4 Pin
		Pin_Config = (GPIO_PinConfig_t){GPIO_PIN_4,GPIO_INPUT_FLOATING};
		MCAL_GPIO_init(GPIO_PORTB, &Pin_Config);

		//Configure  (MOSI) PB5 Pin
		Pin_Config = (GPIO_PinConfig_t){GPIO_PIN_5,GPIO_INPUT_FLOATING};
		MCAL_GPIO_init(GPIO_PORTB, &Pin_Config);

		//Configure  (MISO) PB6 Pin
		Pin_Config = (GPIO_PinConfig_t){GPIO_PIN_6,GPIO_OUTPUT_PP};
		MCAL_GPIO_init(GPIO_PORTB, &Pin_Config);

		//Configure  (SCK) PB7 Pin
		Pin_Config = (GPIO_PinConfig_t){GPIO_PIN_7,GPIO_INPUT_FLOATING};
		MCAL_GPIO_init(GPIO_PORTB, &Pin_Config);
		break;
	default:
		break;
	}
}

/************************************************************************************
 * Function Name	: MCAL_SPI_setCallBackFunc
 * Description		: Functional responsible to set function that is called when IRQ occur
 * Parameters (in)	: callBackFunc - pointer to call back function
 * Return value		: None
 ************************************************************************************/
void MCAL_SPI_setCallBackFunc( void (*callBackFunc)() ){
	if(callBackFunc != NULL){
		GP_SPI_CallBackFunc = callBackFunc;
	}
}


/************************************************************************************
 * Function Name	: MCAL_SPI_enable
 * Description		: Functional responsible to enable SPI
 * Parameters (in)	: None
 * Return value		: None
 ************************************************************************************/
void MCAL_SPI_enable(){
		SPCR->bit.SPE = TRUE;
}


/************************************************************************************
 * Function Name	: MCAL_SPI_disable
 * Description		: Functional responsible to disable SPI
 * Parameters (in)	: None
 * Return value		: None
 ************************************************************************************/
void MCAL_SPI_disable(){
		SPCR->bit.SPE = FALSE;
}

/************************************************************************************
 * Function Name	: MCAL_SPI_waitBusyFlag
 * Description		: Functional responsible to check Collision Flag
 * Parameters (in)	: None
 * Return value		: None
 ************************************************************************************/
boolean MCAL_SPI_CheckCollisionFlag(){
	return (SPSR->bit.WCOL);
}


//============================================================
/*******************************************************************************
 *                            			ISR	  		                 			*
 *******************************************************************************/

ISR(SPI_STC_vect){
	if(GP_SPI_CallBackFunc != NULL){
		(*GP_SPI_CallBackFunc)();
	}
}


