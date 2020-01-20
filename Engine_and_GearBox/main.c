/*
 * Engine_and_GearBox.c
 *
 * Created: 15.03.2019 22:59:59
 * Author : Michael
 */ 

#include "EngGear.h"
#include <avr/interrupt.h>
#include "CAN/CAN.h"
	

/***************************** ���������� *****************************/

 uint8_t received_data[8];
 uint8_t Number_Data = 0, bit_action = 0;
 CANMessage mess_to_send, mess_to_get;

/***************************** ********** *****************************/


void port_ini(void)
{
	//������� ����� ���������� �� �����
	DDRB |= 0b00100000;
	
}

/* ������� ������������� ��� */
void ADC_Init();

ISR(SPI_STC_vect)      // �������� �� SPI ������
{

}

int main(void){

	uint16_t u;

	ADC_Init();
	mcp_init();


	
	while(1){
		do{
			ADCSRA |= (1 << ADSC);    // �������� ��������������
		}
		while ((ADCSRA & (1 << ADIF)) == 0); // ���� �� ����� ���������� ����� �� ��������� ��������������
		
		u = (ADCL|ADCH << 8); // ���������  ���������� ��������
		
		//��������� ��������� ��������
		if (u > 1000){
			PORTB |= (1 << 5);
		}
		else {
			PORTB &= ~(1 << 5);
		}
	}
}