/*
 * motor.h
 *
 *  Created on: Jul 30, 2024
 *      Author: jlabe
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

enum Motor
{
	LEFT = 0,
	RIGHT = 1,
	BOTH = 2,

	MOTOR_COUNT
};

bool motor_init();

void motor_requestMovement(int8_t speed, uint8_t motor);

#endif /* INC_MOTOR_H_ */
