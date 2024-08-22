/*
 * mpu6050.c
 *
 *  Created on: Apr 8, 2024
 *      Author: Utilisateur
 */

#include "mpu6050.h"

void MPU6050_Init(void)
{
	//Check if device is ready
	char msg[200];
	uint8_t reg_buff = 0;

	if(HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}
	//RESET MODULE
	reg_buff = 0x1<<7;
	if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, 1, &reg_buff, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_Delay(50);

	//Remove from SLEEP MODE
	if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, 1, 0, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_Delay(50);
	if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, 1, &reg_buff, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_Delay(50);

	//Accelerometer configuration
	if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REG, 1, &reg_buff, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_Delay(50);
	reg_buff |= (MPU6050_AFS_SEL_4G << MPU_6050_ACCEL_CONFIG_AFS_SEL);
	if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REG, 1, &reg_buff, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_Delay(50);
	if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REG, 1, &reg_buff, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_Delay(50);
	if(reg_buff != (MPU6050_AFS_SEL_4G << MPU_6050_ACCEL_CONFIG_AFS_SEL))
	{
		sprintf(msg,"MPU6050_ACCEL_CONFIG NOT SET CORRECTLY\n\r");
		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		Error_Handler();
	}
	HAL_Delay(50);

	//Gyroscope configuration

	reg_buff = 8;

	if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_CONFIG_REG, 1, &reg_buff, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_Delay(50);
	reg_buff = 0;
	if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_CONFIG_REG, 1, &reg_buff, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_Delay(50);
	if(reg_buff != 8)
	{
		sprintf(msg,"MPU6050_GYRO_CONFIG NOT SET CORRECTLY\n\r");
		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		Error_Handler();
	}
	HAL_Delay(50);
}

void MPU6050_GetAcc(uint16_t* acc_buff)
{
	uint8_t reg_buff[6] = {0,0,0,0,0,0};

	if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_DATA_REG, 1, reg_buff, 6, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}

	acc_buff[0] = ((uint16_t) reg_buff[0] << 8) | (uint16_t) reg_buff[1];
	acc_buff[1] = ((uint16_t) reg_buff[2] << 8) | (uint16_t) reg_buff[3];
	acc_buff[2] = ((uint16_t) reg_buff[4] << 8) | (uint16_t) reg_buff[5];
}

void MPU6050_GetGyro(uint16_t* gyro_buff)
{
	uint8_t reg_buff[6] = {0,0,0,0,0,0};

	if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_DATA_REG, 1, reg_buff, 6, HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}

	gyro_buff[0] = ((uint16_t) reg_buff[0] << 8) | (uint16_t) reg_buff[1];
	gyro_buff[1] = ((uint16_t) reg_buff[2] << 8) | (uint16_t) reg_buff[3];
	gyro_buff[2] = ((uint16_t) reg_buff[4] << 8) | (uint16_t) reg_buff[5];
}
