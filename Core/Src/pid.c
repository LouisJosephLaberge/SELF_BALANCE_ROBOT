/*
 * pid.c
 *
 *  Created on: Sep 16, 2024
 *      Author: jlabe
 */

#include "pid.h"
#include "motor.h"
#include "adc.h"

Pid_Handler hpid;

bool pidInit(float p, float i, float d)
{
	hpid.p_gain = p;
	hpid.i_gain = i;
	hpid.d_gain = d;

	return true;
}

void pidApply()
{
	uint32_t dt = HAL_GetTick() - hpid.timestamp;
	hpid.change = false;

	if(dt >= RESPONSE_TIME_MS)
	{
		float error = filterGetRollAngle();
		if(handler_adc.convCompleted)
		{
			hpid.p_gain = (float)handler_adc.raw_values[0] / 4095.0f;
			hpid.i_gain = (float)handler_adc.raw_values[1] / 4095.0f;
			hpid.d_gain = (float)handler_adc.raw_values[2] / 4095.0f;

			handler_adc.convCompleted = false;
		}

		hpid.timestamp = HAL_GetTick();

		hpid.error_sum += error;

		if(hpid.error_sum >= INTEGRAL_GAIN_MAX) hpid.error_sum = INTEGRAL_GAIN_MAX;

		if(hpid.error_sum <= -INTEGRAL_GAIN_MAX) hpid.error_sum = -INTEGRAL_GAIN_MAX;

		hpid.output = (float)(hpid.p_gain * error
				+ (hpid.i_gain * hpid.error_sum * (float)dt / 1000)
				- hpid.d_gain * (error - hpid.last_error) * 1000 / (float)dt);

		if(hpid.output >= PID_MAX) hpid.output = PID_MAX;

		if(hpid.output <= -PID_MAX) hpid.output = -PID_MAX;

		hpid.last_error = error;
		hpid.change = true;
	}
}
