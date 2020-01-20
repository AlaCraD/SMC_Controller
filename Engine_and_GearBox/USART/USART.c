/*
 * USART.c
 *
 * Created: 08.04.2019 21:49:01
 *  Author: Michael
 */ 

#include "USART.h"

void USART_Init(uint16_t ubrr){
	/*Set baud rate */
	UBRR0H = (uint8_t)(ubrr>>8);
	UBRR0L = (uint8_t) ubrr;
	/*Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

 void USART_Transmit(uint8_t data){
	// Wait for empty transmit buffer
	while (!(UCSR0A & (1<<UDRE0)))
	// Put data into buffer, sends the data
	UDR0 = data;	
}


/* void USART_Transmit(uint16_t data){
	// Wait for empty transmit buffer 
	while (!(UCSR0A & (1<<UDRE0)));
	// Copy 9th bit to TXB8 
	UCSR0B &= ~(1<<TXB80);
	if (data & 0x0100) UCSR0B |= (1<<TXB80);
	// Put data into buffer, sends the data 
	UDR0 = data;
}*/

uint8_t USART_Receive(void){
	/* Wait for data to be received */
	while (!(UCSR0A & (1<<RXC0)));
	/* Get and return received data from buffer */
	return UDR0;
}

/* uint16_t USART_Receive( void ){
	uint8_t status, resh, resl;
	// Wait for data to be received
	while (!(UCSR0A & (1<<RXCn)));
	// Get status and 9th bit, then data from buffer 
	status = UCSR0A;
	resh = UCSR0B;
	resl = UDR0;
	// If error, return -1 
	if (status & (1<<FE0)|(1<<DOR0)|(1<<UPE0))
		return -1;
	// Filter the 9th bit, then return 
	resh = (resh >> 1) & 0x01;
	return ((resh << 8) | resl);
}*/

