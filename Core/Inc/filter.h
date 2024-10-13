/*
 * filter.h
 *
 *  Created on: Sep 16, 2024
 *      Author: jlabe
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

#include "main.h"
#include "mpu6050.h"

#define G 9.81
#define ALPHA (0.99)
#define GYRO_LP 5
#define ACC_HP  15
#define PI (3.141592654)

typedef struct
{
	float last_roll_angle;
	uint16_t timestamp;
}Filter_Handler;

float filterGetRollAngle();

void filterLpAcc(Acc_Handler* acc);

void filterHpGyro(Gyro_Handler* gyro);

#endif /* INC_FILTER_H_ */
