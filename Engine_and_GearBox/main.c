/*
 * Engine_and_GearBox.c
 *
 * Created: 15.03.2019 22:59:59
 * Author : Michael
 */ 

#include "EngGear.h"
#include <avr/interrupt.h>
#include "CAN/CAN.h"
	

/***************************** Переменные *****************************/

 uint8_t received_data[8];
 uint8_t Number_Data = 0, bit_action = 0;
 CANMessage mess_to_send, mess_to_get;

/***************************** ********** *****************************/


void port_ini(void)
{
	//Включим ножку светодиода на выход
	DDRB |= 0b00100000;
	
}

/* Функция инициализация АЦП */
void ADC_Init();

ISR(SPI_STC_vect)      // передача по SPI данных
{

}

int main(void){

	uint16_t u;

	ADC_Init();
	mcp_init();


	
	while(1){
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
	}
}