/*
 * ADC_ATmega328P.c
 *
 * Created: 18.03.2019 19:10:33
 *  Author: Michael
 */ 
#include "ADC_ATmega328P.h"

void ADC_Init(){
	ADCSRA = (1 << ADEN)|(1 << ADIE)|(0 << ADLAR)								// Включаем АЦП
	|(1 << ADPS2)|(1 << ADPS1)|(1 << ADPS0);							// устанавливаем предделитель преобразователя на 8
	ADMUX |= (1 << REFS1)|(1 << REFS0)					// выставляем опорное напряжение, как внешний ИОН      MUX3, MUX2, MUX1 и MUX0.
	|(0 << MUX0)|(0 << MUX1)|(0 << MUX2)|(0 << MUX3);   // снимать сигнал будем с  входа PC0					0000-1000	ADC0-ADC8
}

uint16_t ADC_Procc(){
	do { ADCSRA |= (1 << ADSC); }						// Начинаем преобразование
		while ((ADCSRA & (1 << ADIF)) == 0);			// пока не будет выставлено флага об окончании преобразования
		
	return (ADCL|(ADCH << 8));
}