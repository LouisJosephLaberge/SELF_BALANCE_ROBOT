/*
 * filter.c
 *
 *  Created on: Sep 16, 2024
 *      Author: jlabe
 */

#include "filter.h"

static Filter_Handle hfilter;

uint32_t filterGetPitchAngle()
{
	Acc_Handle acc_raw, acc_filtered;
	Gyro_Handle gyro_raw, gyro_filtered;

	uint32_t pitch_angle = 0;
	uint32_t dt = HAL_GetTick() - hfilter.timestamp;
	hfilter.timestamp = HAL_GetTick();

	mpu6050GetGyro(&gyro_raw);
	mpu6050GetAcc(&acc_raw);

	//Apply Low-Pass filter for the Accelerometer
	filterLpAcc(&acc_raw, &acc_filtered);

	//Apply High-Pass filter for the Gyroscope
	filterHpGyro(&gyro_raw, &gyro_filtered);

	//Combine both inputs in
	pitch_angle = 0.98 * (hfilter.last_pitch_angle + gyro_filtered.x*dt) + 0.02*acc_filtered.x;

	return pitch_angle;
}

void filterLpAcc(Acc_Handle* acc_raw, Acc_Handle* acc_filtered)
{
	acc_filtered->x = acc_raw->x;
	acc_filtered->y = acc_raw->y;
	acc_filtered->z = acc_raw->z;
}

void filterHpGyro(Gyro_Handle* gyro_raw, Gyro_Handle* gyro_filtered)
{
	gyro_filtered->x = gyro_raw->x;
	gyro_filtered->y = gyro_raw->y;
	gyro_filtered->z = gyro_raw->z;
}
