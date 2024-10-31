/*
 * adc.h
 *
 *  Created on: Oct 24, 2024
 *      Author: jlabe
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"

typedef struct
{
	bool convCompleted;
	uint32_t raw_values[3];
}Adc_Handler;

bool adcInit();

extern Adc_Handler handler_adc;

#endif /* INC_ADC_H_ */
