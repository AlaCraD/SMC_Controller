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