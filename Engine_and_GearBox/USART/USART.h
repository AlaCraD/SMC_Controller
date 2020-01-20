/*
 * USART.h
 *
 * Created: 08.04.2019 21:48:49
 *  Author: Michael
 */ 


#ifndef USART_H_
#define USART_H_

	#include "../EngGear.h"
	
	void USART_Init(uint16_t);
	void USART_Transmit(uint8_t);
	//void USART_Transmit(uint16_t);
	uint8_t USART_Receive(void);
	//uint16_t USART_Receive(void);

#endif /* USART_H_ */