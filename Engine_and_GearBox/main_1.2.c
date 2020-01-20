/*
 * Engine_and_GearBox.c
 *
 * Created: 15.03.2019 22:59:59
 * Author : Michael
 */ 

#include <avr/io.h>
/* Description:    This program demonstrates how to make a CAN network using mikroElektronika
                   CANSPI boards and mikroC compiler.
   Target device:  Atmel ATMEGA16
   Oscillator:     8MHz crystal */
 
char Can_Init_Flags, Can_Send_Flags, Can_Rcv_Flags;                           // can flags
char Rx_Data_Len;                                                             // received data length in bytes
char RxTx_Data[8];                                                            // can rx/tx data buffer
char Msg_Rcvd;                                                                // reception flag
long Tx_ID, Rx_ID;                                                            // can rx and tx ID
char ErrorCount;
// CANSPI module connections
#define CanSpi_CS (PORTB|=(1<<0))                                                   // Chip select (CS) pin for CANSPI board
#define CanSpi_CS_Direction (DDRB|=(1<<0))                                         // Direction register for CS pin
#define CanSpi_Rst (PORTB|=(1<<2))                                                 // Reset pin for CANSPI board
#define CanSpi_Rst_Direction (DDRB|=(1<<2))                                         // Direction register for Reset pin
// End CANSPI module connections

void port_ini(void)
{
	
	//Включим ножку светодиода на выход
	
	DDRB |= 0b00100000;
	
}

/* Функция инициализация АЦП */
void ADC_Init(){
	ADCSRA |= (1 << ADEN) // Включаем АЦП
	|(1 << ADPS1)|(1 << ADPS0);    // устанавливаем предделитель преобразователя на 8
	ADMUX |= (0 << REFS1)|(1 << REFS0) //выставляем опорное напряжение, как внешний ИОН      MUX3, MUX2, MUX1 и MUX0.
	|(0 << MUX0)|(0 << MUX1)|(0 << MUX2)|(0 << MUX3); // снимать сигнал будем с  входа PC0	 0000-1000	ADC0-ADC8
}

int main(void){

	unsigned int u;

	ADC_Init();
	
	DDRB = (1 << 5); //пин 5 порта B как выход
	PORTB = 0x00; //Начальное состояние порта 0

	/*while(1){
		do{
			ADCSRA |= (1 << ADSC);    // Начинаем преобразование
		}
		while ((ADCSRA & (1 << ADIF)) == 0); // пока не будет выставлено флага об окончании преобразования
		
		u = (ADCL|ADCH << 8); // Считываем  полученное значение
		
		//Проверяем считанное значение
		if (u > 1000){
			PORTB |= (1 << 5);
		}
		else {
			PORTB &= ~(1 << 5);
		}
	}*/
	ADCSRA &= (0<<7);                                                              // Configure analog pins as digital I/O
	PORTB  = 0; DDRB = 255;                                                     // Initialize ports
	PORTD  = 0; DDRD = 255;
	PORTC  = 0; DDRC = 255;
	
	ErrorCount     = 0;                                                         // Error flag
	Can_Init_Flags = 0; Can_Send_Flags = 0; Can_Rcv_Flags = 0;                  // clear flags
	
	Can_Send_Flags = _CANSPI_TX_PRIORITY_0 &                                    // form value to be used
	_CANSPI_TX_XTD_FRAME &                                     // with CANSPIWrite
	_CANSPI_TX_NO_RTR_FRAME;
	
	Can_Init_Flags = _CANSPI_CONFIG_SAMPLE_THRICE &                             // form value to be used
	_CANSPI_CONFIG_PHSEG2_PRG_ON &                             // with CANSPIInit
	_CANSPI_CONFIG_XTD_MSG &
	_CANSPI_CONFIG_DBL_BUFFER_ON &
	_CANSPI_CONFIG_VALID_XTD_MSG;
	
	SPI1_Init();
	Spi_Rd_Ptr = SPI1_Read;                                                     // initialize SPI module
	CANSPIInitialize(1, 3, 3, 3, 1, Can_Init_Flags);                            // Initialize external CANSPI module
	CANSPISetOperationMode(_CANSPI_MODE_CONFIG, 0xFF);                          // set CONFIGURATION mode
	CANSPISetMask(_CANSPI_MASK_B1, -1, _CANSPI_CONFIG_XTD_MSG);                 // set all mask1 bits to ones
	CANSPISetMask(_CANSPI_MASK_B2, -1, _CANSPI_CONFIG_XTD_MSG);                 // set all mask2 bits to ones
	
	CANSPISetFilter(_CANSPI_FILTER_B2_F4, 0x12, _CANSPI_CONFIG_XTD_MSG);        // Node1 accepts messages with ID 0x12
	CANSPISetFilter(_CANSPI_FILTER_B1_F1, 0x13, _CANSPI_CONFIG_XTD_MSG);        // Node1 accepts messages with ID 0x13
	
	CANSPISetOperationMode(_CANSPI_MODE_NORMAL,0xFF);                           // set NORMAL mode
	RxTx_Data[0] = 0x40;                                                        // set initial data to be sent
	
	Tx_ID = 0x10;                                                               // set transmit ID for CAN message
	
	CANSPIWrite(Tx_ID, &RxTx_Data, 1, Can_Send_Flags);                          // Node1 sends initial message
	
	while (1)                                                                   // endless loop
	{
		Msg_Rcvd = CANSPIRead(&Rx_ID, RxTx_Data, &Rx_Data_Len, &Can_Rcv_Flags); // attempt receive message
		if (Msg_Rcvd) {                                                         // if message is received then check id
			
			if (Rx_ID == 0x12)                                                  // check ID
			PORTC = RxTx_Data[0];                                           // output data at PORTC
			else
			PORTD = RxTx_Data[0];                                           // output data at PORTD
			Delay_ms(50);                                                       // wait for a while between messages
			CANSPIWrite(Tx_ID, RxTx_Data, 1, Can_Send_Flags);                   // send one byte of data
			Tx_ID++;                                                            // switch to next message
			if (Tx_ID > 0x11) Tx_ID = 0x10;                                     // check overflow
			
			
		}
		else {                                                                  // an error occured, wait for a while
			
			ErrorCount++;                                                       // increment error indicator
			Delay_ms(10);                                                       // wait for 10ms
			if (ErrorCount > 10) {                                              // timeout expired - process errors
				ErrorCount = 0;                                                   // reset error counter
				Tx_ID++;                                                          // switch to another message
				if (Tx_ID > 0x11) Tx_ID = 0x10;                                   // check overflow
				CANSPIWrite(Tx_ID, RxTx_Data, 1, Can_Send_Flags);                 // send new message
			}
			
		}
	}
}