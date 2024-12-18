/*
 * mpu6050.h
 *
 *  Created on: Apr 8, 2024
 *      Author: jlabe
 */

#include "main.h"

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define MPU6050_ADDR ((0b1101000 << 1) +0)

#define MPU6050_SMPRT_DIV_REG		25
#define MPU6050_CONFIG_REG 			26
#define MPU6050_GYRO_CONFIG_REG 	27
#define MPU6050_ACCEL_CONFIG_REG 	28
#define MPU6050_MOT_THR_REG 		29
#define MPU6050_PWR_MGMT_1_REG		0X6B
#define MPU6050_ACCEL_DATA_REG 		59
#define MPU6050_GYRO_DATA_REG 		67

#define MPU_6050_GYRO_CONFIG_FS_SEL 	3
#define MPU_6050_ACCEL_CONFIG_AFS_SEL 	3

#define MPU6050_AFS_SEL_2G	0
#define MPU6050_AFS_SEL_4G	1
#define MPU6050_AFS_SEL_8G	2
#define MPU6050_AFS_SEL_16G	3

#define MPU6050_FS_SEL_250HZ	0
#define MPU6050_FS_SEL_500HZ	1
#define MPU6050_FS_SEL_1000HZ	2
#define MPU6050_FS_SEL_2000HZ	3

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

typedef struct
{
	int16_t raw_x;
	int16_t raw_y;
	int16_t raw_z;
	float   filtered_x;
	float   filtered_y;
	float   filtered_z;
}Acc_Handler;

typedef struct
{
	int16_t raw_x;
	int16_t raw_y;
	int16_t raw_z;
	float   filtered_x;
	float   filtered_y;
	float   filtered_z;
}Gyro_Handler;

bool mpu6050Init(void);
void mpu6050GetAcc(Acc_Handler* acc_buff);
void mpu6050GetGyro(Gyro_Handler* gyro_buff);

#endif /* INC_MPU6050_H_ */
