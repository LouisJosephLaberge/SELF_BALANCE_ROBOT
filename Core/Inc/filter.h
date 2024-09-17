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

typedef struct
{
	uint32_t last_pitch_angle;
	uint16_t timestamp;
}Filter_Handler;

int32_t filterGetPitchAngle();

void filterLpAcc(Acc_Handler* acc_raw, Acc_Handler* acc_filtered);

void filterHpGyro(Gyro_Handler* gyro_raw, Gyro_Handler* gyro_filtered);

#endif /* INC_FILTER_H_ */
