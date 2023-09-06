 /******************************************************************************
 * Module: EXTI
 * File Name: EXTI.h
 * Description: Header file for EXTI driver for stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

#ifndef INC_EXTI_H_
#define INC_EXTI_H_
/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "Platform_Types.h"
#include "Common_Macros.h"


/*******************************************************************************
 *                             	EXTI Data Types                             	*
 *******************************************************************************/
typedef enum{
	INT_0,INT_1,INT_2
}EXTI_NUM_e;

//@REF enumeration:EXTI_EDGE_e
typedef enum{
	LOW_LEVEL,BOTH_EDGES,FAILING_EGDE,RAISING_EDGE
}EXTI_EDGE_e;

typedef struct{
	EXTI_NUM_e INT_Num;
	uint8 GPIO_Port;
	uint8 GPIO_Pin;
}EXTI_GPIOMaping;

typedef struct{
	EXTI_GPIOMaping EXTI_Pin;	//Specifies the external interrupt GPIO mapping
								//This Parameter must be based on @REF EXTI_Define
	EXTI_EDGE_e		Edge;		//Specifies the edge on the pin for interrupt
								//This Parameter must be based on @REF enumeration:EXTI_EDGE_e
}EXTI_Config_t;
/*******************************************************************************
 *                             	GPIO Definition                          		*
 *******************************************************************************/
//@REF EXTI_Define
#define INT0PD2	(EXTI_GPIOMaping){INT_0 , GPIO_PORTD , GPIO_PIN_2}
#define INT1PD3	(EXTI_GPIOMaping){INT_1 , GPIO_PORTD , GPIO_PIN_3}
#define INT2PB2	(EXTI_GPIOMaping){INT_2 , GPIO_PORTB , GPIO_PIN_2}

//================================================================================
/*******************************************************************************
 *					APIs Supported by "MCAL EXTI DRIVER"                       *
 *******************************************************************************/

void MCAL_EXTI_init(EXTI_Config_t* EXTIConfig);
void MCAL_EXTI_deInit();
void MCAL_EXTI_enable(EXTI_NUM_e EXTI_Number);
void MCAL_EXTI_disable(EXTI_NUM_e EXTI_Number);
void MCAL_EXTI_update(EXTI_Config_t* EXTIConfig);
void MCAL_EXTI_setCallBackFun(EXTI_NUM_e EXTI_Number,void (*callBackFunc)(void));


//=================================================================================
#endif /* INC_EXTI_H_ */
