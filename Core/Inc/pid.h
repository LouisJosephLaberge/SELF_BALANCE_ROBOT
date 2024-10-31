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
#define INTEGRAL_GAIN_MAX 5000
#define PID_MAX 10

typedef struct
{
	float p_gain;
	float i_gain;
	float d_gain;
	float last_error;
	int32_t error_sum;
	uint32_t timestamp;
	float output;
	bool change;
}Pid_Handler;

extern Pid_Handler hpid;

bool pidInit(float p, float i, float d);

void pidApply();

#endif /* INC_PID_H_ */
