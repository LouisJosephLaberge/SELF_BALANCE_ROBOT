/*
 * adc.c
 *
 *  Created on: Oct 24, 2024
 *      Author: jlabe
 */

#include "adc.h"

extern ADC_HandleTypeDef hadc1;

Adc_Handler handler_adc;

bool adcInit()
{
	return HAL_OK == HAL_ADC_Start_DMA(&hadc1, handler_adc.raw_values, 3);
}

uint16_t adcGetValues()
{
	handler_adc.raw_values[0] = HAL_ADC_GetValue(&hadc1);
	return HAL_ADC_GetValue(&hadc1);
}
