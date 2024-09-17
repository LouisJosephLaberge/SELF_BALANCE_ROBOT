/*
 * motor.h
 *
 *  Created on: Jul 30, 2024
 *      Author: jlabe
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include "pid.h"

enum Motor
{
	LEFT = 0,
	RIGHT = 1,
	BOTH = 2,

	MOTOR_COUNT
};

bool motorInit();

void motorTest();

void motorRequestMovement(int8_t speed, uint8_t motor);

void motorProcess();

#endif /* INC_MOTOR_H_ */
