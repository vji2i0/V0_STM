/*
	Global variables
*/
#include "config.h"
#include "stm32f1xx_hal.h"

#ifndef GLOBALVARIABLES_H
#define GLOBALVARIABLES_H
extern struct vector location;
extern struct vector placement;

extern uint8_t state0;
extern unsigned char current_temperature;
extern unsigned char current_temperature_bed;

extern struct discret_vector translation_discret;
extern long translation_discret_length;
extern struct discret_vector progress;
extern long counter;
extern long kappa;

extern TIM_HandleTypeDef htim15;
extern ADC_HandleTypeDef hadc1;
/*
	state0 - control byte
	0b
	1 - 0 - we can't move, 1 - we can move
	2 - 0 - absolute, 1 - relative positioning
	3 - 0 - no CRS check, 1 - CRC check
	4
	5
	6
	7
	8
*/
#endif
