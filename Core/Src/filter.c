/*
 * filter.c
 *
 *  Created on: Sep 16, 2024
 *      Author: jlabe
 */

#include "filter.h"

static Filter_Handler hfilter;

float filterGetRollAngle()
{
	Acc_Handler acc;
	Gyro_Handler gyro;

	float roll_angle_total = 0, roll_gyro = 0, roll_acc = 0;
	float dt_ms = HAL_GetTick() - hfilter.timestamp;

	hfilter.timestamp = HAL_GetTick();

	mpu6050GetGyro(&gyro);
	mpu6050GetAcc(&acc);

	//Apply Low-Pass filter for the Accelerometer
	filterLpAcc(&acc);

	//Apply High-Pass filter for the Gyroscope
	filterHpGyro(&gyro);

	//Combine both inputs giving 97% weight to gyro according to literature
	roll_gyro = gyro.filtered_y*dt_ms/1000;
	roll_acc = (180*(atan(acc.filtered_x / sqrt(pow(acc.filtered_y, 2) + pow(acc.filtered_z,2)))))/PI;

	roll_angle_total = (ALPHA * (hfilter.last_roll_angle + roll_gyro) + (1-ALPHA)*roll_acc);
	hfilter.last_roll_angle = roll_angle_total;

	return roll_acc;
}

void filterLpAcc(Acc_Handler* acc)
{
	acc->filtered_x = (float)((acc->raw_x * G) / 8192);
	acc->filtered_y = (float)((acc->raw_y * G) / 8192);
	acc->filtered_z = (float)((acc->raw_z * G) / 8192);
}

void filterHpGyro(Gyro_Handler* gyro)
{
	gyro->filtered_x = (float)(gyro->raw_x / 65.5);
	gyro->filtered_y = (float)(gyro->raw_y / 65.5);
	gyro->filtered_z = (float)(gyro->raw_z / 65.5);
}
