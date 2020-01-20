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
 
void port_ini(void)
{
	
	//������� ����� ���������� �� �����
	
	DDRB |= 0b00100000;
	
}

/* ������� ������������� ��� */
void ADC_Init(){
	ADCSRA |= (1 << ADEN) // �������� ���
	|(1 << ADPS1)|(1 << ADPS0);    // ������������� ������������ ��������������� �� 8
	ADMUX |= (0 << REFS1)|(1 << REFS0) //���������� ������� ����������, ��� ������� ���      MUX3, MUX2, MUX1 � MUX0.
	|(0 << MUX0)|(0 << MUX1)|(0 << MUX2)|(0 << MUX3); // ������� ������ ����� �  ����� PC0	 0000-1000	ADC0-ADC8
}

int main(void){

	unsigned int u;

	ADC_Init();
	
	DDRB = (1 << 5); //��� 5 ����� B ��� �����
	PORTB = 0x00; //��������� ��������� ����� 0

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