/*
 * filter.h
 *
 *  Created on: Sep 16, 2024
 *      Author: jlabe
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

#include "main.h"

typedef struct
{
	uint32_t last_pitch_angle;
	uint16_t timestamp;
}Filter_Handle;

uint32_t filterGetPitchAngle();

void filterLpAcc(Acc_Handle* acc_raw, Acc_Handle* acc_filtered);

void filterHpGyro(Gyro_Handle* gyro_raw, Gyro_Handle* gyro_filtered);

#endif /* INC_FILTER_H_ */
