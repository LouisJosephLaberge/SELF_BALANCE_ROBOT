/*
 * pid.c
 *
 *  Created on: Sep 16, 2024
 *      Author: jlabe
 */

#include "pid.h"

static Pid_Handler hpid;

bool pidInit(uint32_t p, uint32_t i, uint32_t d)
{
	hpid.p_gain = p;
	hpid.i_gain = i;
	hpid.d_gain = d;

	return true;
}

void pidApply()
{
	uint32_t error = (uint32_t)GYRO_REFERENCE_VALUE - filterGetPitchAngle();
	uint32_t dt = HAL_GetTick() - hpid.timestamp;

	hpid.timestamp = HAL_GetTick();
	hpid.error_sum += error;

	if(hpid.error_sum >= INTEGRAL_GAIN_MAX)
	{
		hpid.error_sum = INTEGRAL_GAIN_MAX;
	}
	if(hpid.error_sum <= -INTEGRAL_GAIN_MAX)
	{
		hpid.error_sum = -INTEGRAL_GAIN_MAX;
	}

	hpid.output = hpid.p_gain * error
			+ hpid.i_gain * hpid.error_sum * dt
			+ hpid.d_gain * (error - hpid.last_error) /dt;

	if(hpid.output >= PID_MAX)
	{
		hpid.output = PID_MAX;
	}
	if(hpid.output <= -PID_MAX)
	{
		hpid.output = -PID_MAX;
	}
	hpid.last_error = error;
}
