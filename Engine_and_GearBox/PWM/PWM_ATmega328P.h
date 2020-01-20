/*
 * PWM_ATmega328P.h
 *
 * Created: 18.03.2019 19:02:07
 *  Author: Michael
 */ 


#ifndef PWM_ATMEGA328P_H_
#define PWM_ATMEGA328P_H_
	
	#include "../EngGear.h"
		
		// 16 000 000 hz
	void pwm_init();
	void pwm_work(uint8_t);
	
#endif /* PWM_ATMEGA328P_H_ */