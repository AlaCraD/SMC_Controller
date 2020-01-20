/*
 * EngGear.h
 *
 * Created: 16.03.2019 15:30:56
 *  Author: Michael
 */ 


#ifndef ENGGEAR_H_
#define ENGGEAR_H_

	#define		F_CPU 16000000UL
	
	#include	<avr/io.h>
	#include	<inttypes.h>
	#include	<util/delay.h>
	#include	<stdbool.h>
	
//*********************************************************************
	
	#define	SET_0(x)		_XRS(x)
	#define	SET_1(x)		_XS(x)
	#define	TOGGLE(x)		_XT(x)
	#define	SET_OUTPUT(x)	_XSO(x)
	#define	SET_INPUT(x)	_XSI(x)
	#define	IS_SET(x)		_XR(x)

	#define	PORT(x)			_port2(x)
	#define	DDR(x)			_ddr2(x)
	#define	PIN(x)			_pin2(x)

	#define	_XRS(x,y)	PORT(x) &= ~(1<<y)
	#define	_XS(x,y)	PORT(x) |= (1<<y)
	#define	_XT(x,y)	PORT(x) ^= (1<<y)

	#define	_XSO(x,y)	DDR(x) |= (1<<y)
	#define	_XSI(x,y)	DDR(x) &= ~(1<<y)

	#define	_XR(x,y)	((PIN(x) & (1<<y)) != 0)

	#define	_port2(x)	PORT ## x
	#define	_ddr2(x)	DDR ## x
	#define	_pin2(x)	PIN ## x
	
//******************************************************
	
	#define P_SCK		B,5
	#define P_MISO		B,4
	#define P_MOSI		B,3
	#define P_CS		B,2
	#define	MCP_INT		D,3
	
	#define PWM_OCR1A	D,6
	#define RXD0		D,0
	#define TXD0		D,1


#endif /* ENGGEAR_H_ */