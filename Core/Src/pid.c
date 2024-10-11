/*
 * pid.c
 *
 *  Created on: Sep 16, 2024
 *      Author: jlabe
 */

#include "pid.h"
#include "motor.h"

Pid_Handler hpid;

bool pidInit(uint32_t p, uint32_t i, uint32_t d)
{
	hpid.p_gain = p;
	hpid.i_gain = i;
	hpid.d_gain = d;

	return true;
}

void pidApply()
{
	int32_t error = filterGetRollAngle();
	uint32_t dt = HAL_GetTick() - hpid.timestamp;
	hpid.change = false;

	if(dt >= 25)
	{
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

		int16_t d = error - hpid.last_error;
		d = d / (int16_t)dt;

		hpid.output = hpid.p_gain * error
				+ hpid.i_gain * hpid.error_sum * dt
				+ hpid.d_gain * d;

		if(hpid.output >= PID_MAX)
		{
			hpid.output = PID_MAX;
		}
		if(hpid.output <= -PID_MAX)
		{
			hpid.output = -PID_MAX;
		}
		hpid.last_error = error;
		hpid.change = true;
	}
}
