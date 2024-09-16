/*
 * mpu6050.c
 *
 *  Created on: Apr 8, 2024
 *      Author: jlabe
 */

#include "mpu6050.h"

bool mpu6050Init(void)
{
	//Check if device is ready
	char msg[200];
	uint8_t reg_buff = 0;

	if(HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	//RESET MODULE
	reg_buff = 0x1<<7;
	if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, 1, &reg_buff, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	HAL_Delay(50);

	//Remove from SLEEP MODE
	if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, 1, 0, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	HAL_Delay(50);
	if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, 1, &reg_buff, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	HAL_Delay(50);

	//Accelerometer configuration
	if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REG, 1, &reg_buff, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	HAL_Delay(50);
	reg_buff |= (MPU6050_AFS_SEL_4G << MPU_6050_ACCEL_CONFIG_AFS_SEL);
	if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REG, 1, &reg_buff, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	HAL_Delay(50);
	if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REG, 1, &reg_buff, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	HAL_Delay(50);
	if(reg_buff != (MPU6050_AFS_SEL_4G << MPU_6050_ACCEL_CONFIG_AFS_SEL))
	{
		sprintf(msg,"MPU6050_ACCEL_CONFIG NOT SET CORRECTLY\n\r");
		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		return false;
	}
	HAL_Delay(50);

	//Gyroscope configuration

	reg_buff = 8;

	if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_CONFIG_REG, 1, &reg_buff, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	HAL_Delay(50);
	reg_buff = 0;
	if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_CONFIG_REG, 1, &reg_buff, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	HAL_Delay(50);
	if(reg_buff != 8)
	{
		sprintf(msg,"MPU6050_GYRO_CONFIG NOT SET CORRECTLY\n\r");
		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		return false;
	}
	HAL_Delay(50);
	return true;
}

void mpu6050GetAcc(Acc_Handle* acc_buff)
{
	uint8_t reg_buff[6] = {0,0,0,0,0,0};

	if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_DATA_REG, 1, reg_buff, 6, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}

	acc_buff->x = ((uint16_t) reg_buff[0] << 8) | (uint16_t) reg_buff[1];
	acc_buff->y = ((uint16_t) reg_buff[2] << 8) | (uint16_t) reg_buff[3];
	acc_buff->z = ((uint16_t) reg_buff[4] << 8) | (uint16_t) reg_buff[5];
}

void mpu6050GetGyro(Gyro_Handle* gyro_buff)
{
	uint8_t reg_buff[6] = {0,0,0,0,0,0};

	if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_DATA_REG, 1, reg_buff, 6, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}

	gyro_buff->x = ((uint16_t) reg_buff[0] << 8) | (uint16_t) reg_buff[1];
	gyro_buff->y = ((uint16_t) reg_buff[2] << 8) | (uint16_t) reg_buff[3];
	gyro_buff->z = ((uint16_t) reg_buff[4] << 8) | (uint16_t) reg_buff[5];
}
