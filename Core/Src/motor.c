/*
 * motor.c
 *
 *  Created on: Jul 30, 2024
 *      Author: jlabe
 */
#include "motor.h"

extern TIM_HandleTypeDef htim2;

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
			motorRequestMovement(30, LEFT);
			test_motor_idx++;
			break;
		}
		case 1:
		{
			motorRequestMovement(-30, LEFT);
			test_motor_idx++;
			break;
		}
		case 2:
		{
			motorRequestMovement(30, RIGHT);
			test_motor_idx++;
			break;
		}
		case 3:
		{
			motorRequestMovement(-30, RIGHT);
			test_motor_idx++;
			break;
		}
		case 4:
		{
			motorRequestMovement(30, BOTH);
			test_motor_idx++;
			break;
		}
		case 5:
		{
			motorRequestMovement(-30, BOTH);
			test_motor_idx = 0;
			break;
		}
	}
	HAL_Delay(2000);
}

void motorRequestMovement(int8_t speed, uint8_t motor)
{
	uint8_t abs_speed = abs(speed);
	uint32_t ccr_value = (abs_speed * htim2.Init.Period)/100;
	switch(motor)
	{
		case LEFT:
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_value);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_value);
			if(speed >= 0)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			}else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			}
			break;
		}
		case RIGHT:
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_value);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_value);
			if(speed >= 0)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			}else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			}
			break;
		}
		case BOTH:
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_value);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_value);
			if(speed >= 0)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			}else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			}
	}
}
