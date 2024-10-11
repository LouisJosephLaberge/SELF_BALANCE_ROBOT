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
			&& HAL_OK == HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
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
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_value);
			if(speed >= 0)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			}else if(speed <= 0)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			}else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			}
			break;
		}
		case RIGHT:
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_value);
			if(speed >= 0)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			}else if(speed <= 0)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			}else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			}
			break;
		}
		case BOTH:
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_value);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_value);
			if(speed >= 0)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			}else if(speed <= 0)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			}else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			}
	}
}

void motorProcess()
{
	char msg[100];
	pidApply();
	double pid_perc = (hpid.output * 100) / PID_MAX;
	int8_t speed_pid_output = (int8_t)(pid_perc);
	if(hpid.change)
	{
		sprintf(msg, "Angle : %d \n\r", hpid.last_error);
		if(HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY) != HAL_OK)
		{
			Error_Handler();
		}
	}
	motorRequestMovementSpeed(speed_pid_output, BOTH);
}
