/*
 * PWM_ATmega328P.c
 *
 * Created: 18.03.2019 19:02:18
 *  Author: Michael
 */ 

#include "PWM_ATmega328P.h"

void pwm_init()
{
	SET_INPUT(PWM_OCR1A);
	SET_0(PWM_OCR1A);
	TCCR0A=(1<<COM0A1)|(1<<WGM10); //На выводе OC1A единица, когда OCR1A==TCNT1, восьмибитный ШИМ
	TCCR0B=(1<<CS12);		 //Делитель= /1
	TCNT1 = 0x00;
	ICR1 = 0xFF;
	OCR1A=0x00;			//Начальная яркость нулевая
}

void pwm_work(uint8_t occupancy)
{
	OCR0A=occupancy;
	
}

