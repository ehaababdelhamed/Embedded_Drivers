 /******************************************************************************
 * File Name: Atmega32_Registers
 * Description: Header file for Atmega32 Registers map
 * Author: Ehab Mohamed Abdelhamed
 *******************************************************************************/

#ifndef ATMEGA32_H_
#define ATMEGA32_H_
#include "Platform_Types.h"

//Status Register
#define SRGE 	(*(volatile uint8*)(0X5F))

//GPIO Registers
#define PORTD 	((volatile uint8*)(0X32))
#define DDRD 	((volatile uint8*)(0X31))
#define PIND 	((volatile uint8*)(0X30))

#define PORTC 	((volatile uint8*)(0X35))
#define DDRC 	((volatile uint8*)(0X34))
#define PINC 	((volatile uint8*)(0X33))

#define PORTB 	((volatile uint8*)(0X38))
#define DDRB 	((volatile uint8*)(0X37))
#define PINB 	((volatile uint8*)(0X36))

#define PORTA 	((volatile uint8*)(0X3B))
#define DDRA 	((volatile uint8*)(0X3A))
#define PINA 	((volatile uint8*)(0X39))

//External interrupt Register
#define MCUCR  	(*(volatile uint8*)(0X55))
#define MCUCSR 	(*(volatile uint8*)(0X54))
#define GICR  	(*(volatile uint8*)(0X5B))
#define GIFR 	(*(volatile uint8*)(0X5A))


#endif /* ATMEGA32_H_ */
