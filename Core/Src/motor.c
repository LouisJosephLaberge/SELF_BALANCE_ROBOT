/*
 * motor.c
 *
 *  Created on: Jul 30, 2024
 *      Author: jlabe
 */
#include "motor.h"

extern TIM_HandleTypeDef htim1;

bool motor_init()
{
	return HAL_OK == HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1|TIM_CHANNEL_2|TIM_CHANNEL_3|TIM_CHANNEL_4);
}

void motor_requestMovement(int8_t speed, uint8_t motor)
{
	uint8_t abs_speed = abs(speed);
	switch(motor)
	{
		case LEFT:
		{
			if(speed >= 0)
			{
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, abs_speed);
			}else
			{
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, abs_speed);
			}
			break;
		}
		case RIGHT:
		{
			if(speed >= 0)
			{
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, abs_speed);
			}else
			{
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, abs_speed);
			}
			break;
		}
	}
}
