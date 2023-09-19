/******************************************************************************
 * Module: SPI
 * File Name: SPI.h
 * Description: Header file for SPI driver for Stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/
#ifndef INC_SPI_H_
#define INC_SPI_H_
/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "Common_Macros.h"
#include "Platform_Types.h"

/*******************************************************************************
 *                             	SPI Data Types                             	*
 *******************************************************************************/
//@REF : SPI_DeviceMode_e
typedef enum{
	SPI_SLAVE,
	SPI_MASTER
}SPI_DeviceMode_e;

//@REF : SPI_PreScaler_e
typedef enum{
	SPI_PERSCALER_4,
	SPI_PERSCALER_16,
	SPI_PERSCALER_64,
	SPI_PERSCALER_128,
	SPI_PERSCALER_2,
	SPI_PERSCALER_8,
	SPI_PERSCALER_32,
}SPI_PreScaler_e;

//@REF : SPI_CLKPloarity_e
typedef enum{
	SPI_CLKP_IDLE_LOW,
	SPI_CLKP_IDLE_HIGH
}SPI_CLKPloarity_e;

//@REF : SPI_CLKPhase_e
typedef enum{
	SPI_CLKPh_FIRST_EDGE,
	SPI_CLKPh_SECOND_EDGE
}SPI_CLKPhase_e;

//@REF : SPI_DataOrder_e;
typedef enum{
	SPI_DATA_ORDER_MSB_FIRST,
	SPI_DATA_ORDER_LSB_FIRST
}SPI_DataOrder_e;

//@REF : SPI_DataSize_e;
typedef enum{
	SPI_POLLING,
	SPI_Interrupt
}SPI_Mechanism_e;


typedef struct{
	SPI_DeviceMode_e 	Device_Mode;		//Specifics SPI Device mode(Master/Slave) @REF : SPI_Mode_e
	SPI_PreScaler_e		SPI_Prescaler;		//Specifics SPI BaudRate prescaler  @REF : SPI_Mode_e
	SPI_DataOrder_e		Data_Order;			//Specifics SPI  data order (MSB-first or LSB-first) @REF SPI_DataOrder_e
	SPI_CLKPloarity_e	Clock_Polarity;		//Specifics SPI idle level of clock line
	SPI_CLKPhase_e		Clock_Phase;		//Specifics SPI data capture edge
	SPI_Mechanism_e     Mechanism;			//Specifics SPI mechanism
}SPI_Config_t;

/*******************************************************************************
 *                             	SPI Definition                             	*
 *******************************************************************************/


/*******************************************************************************
 *					APIs Supported by "MCAL SPI DRIVER" 	                   *
 *******************************************************************************/
void MCAL_SPI_init(SPI_Config_t *SPIConfig);
void MCAL_SPI_deInit();
void MCAL_SPI_sendData(uint8 TxBuffer);
uint8 MCAL_SPI_receiveData();
uint8 MCAL_SPI_sendReceiveData(uint8 TxBuffer);

void MCAL_SPI_GPIO_SET_PINS();
void MCAL_SPI_setCallBackFunc(void (*callBackFunc)());

void MCAL_SPI_enable();
void MCAL_SPI_disable();


#endif /* INC_SPI_H_ */
