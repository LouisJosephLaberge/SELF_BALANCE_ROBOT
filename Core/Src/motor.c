/*
 * motor.c
 *
 *  Created on: Jul 30, 2024
 *      Author: jlabe
 */
#include "motor.h"

extern TIM_HandleTypeDef htim2;
extern Pid_Handler hpid;

bool motorInit()
{
	return HAL_OK == HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1)
			&& HAL_OK == HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2)
			&& HAL_OK == HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3)
			&& HAL_OK == HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void motorTest()
{
	static uint8_t test_motor_idx = 0;

	switch(test_motor_idx)
	{
		case 0:
		{
			motorRequestMovementSpeed(35, LEFT);
			test_motor_idx++;
			break;
		}
		case 1:
		{
			motorRequestMovementSpeed(-35, LEFT);
			test_motor_idx++;
			break;
		}
		case 2:
		{
			motorRequestMovementSpeed(35, RIGHT);
			test_motor_idx++;
			break;
		}
		case 3:
		{
			motorRequestMovementSpeed(-35, RIGHT);
			test_motor_idx++;
			break;
		}
		case 4:
		{
			motorRequestMovementSpeed(35, BOTH);
			test_motor_idx++;
			break;
		}
		case 5:
		{
			motorRequestMovementSpeed(-35, BOTH);
			test_motor_idx = 0;
			break;
		}
	}
	HAL_Delay(2000);
}

void motorRequestMovementSpeed(int8_t speed, uint8_t motor)
{
	uint8_t abs_speed = abs(speed);
	uint32_t ccr_value = (abs_speed * htim2.Init.Period)/100;
	switch(motor)
	{
		case LEFT:
		{
			if(speed >= 0)
			{
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_value);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
			}else if(speed <= 0)
			{
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_value);
			}else
			{
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
			}
			break;
		}
		case RIGHT:
		{
			if(speed >= 0)
			{
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr_value);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			}else if(speed <= 0)
			{
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ccr_value);
			}else
			{
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			}
			break;
		}
		case BOTH:
			if(speed >= 0)
			{
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_value);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr_value);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			}else if(speed <= 0)
			{
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_value);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ccr_value);
			}else
			{
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			}
	}
}

void motorProcess()
{
	char msg[100];
	pidApply();
	float pid_perc = (hpid.output * (100 - MIN_PWM)) / PID_MAX;
	int8_t sign = (pid_perc < 0) ? -1 : 1;
	int8_t speed_pid_output = abs(pid_perc) < 0.001 ? (sign*MIN_PWM) : (int8_t)pid_perc + (sign*MIN_PWM);
	if(hpid.change)
	{
		sprintf(msg, "Angle : %d               speed output : %d              Time : %ld \n\r", (int16_t)(100*hpid.last_error), speed_pid_output, (uint32_t)HAL_GetTick());
		__disable_irq();
		if(HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY) != HAL_OK)
		{
			Error_Handler();
		}
		while(huart1.gState == HAL_UART_STATE_BUSY_TX);
		hpid.change = false;
		__enable_irq();
	}
	motorRequestMovementSpeed(speed_pid_output, BOTH);
}

