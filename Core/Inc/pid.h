/*
 * pid.h
 *
 *  Created on: Sep 16, 2024
 *      Author: jlabe
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"
#include "filter.h"

#define ANGLE_REFERENCE_VALUE 0
#define INTEGRAL_GAIN_MAX 1000
#define PID_MAX 1000

typedef struct
{
	uint32_t p_gain;
	uint32_t i_gain;
	uint32_t d_gain;
	int16_t last_error;
	int32_t error_sum;
	uint32_t timestamp;
	int16_t output;
}Pid_Handler;

extern Pid_Handler hpid;

bool pidInit(uint32_t p, uint32_t i, uint32_t d);

void pidApply();

#endif /* INC_PID_H_ */
