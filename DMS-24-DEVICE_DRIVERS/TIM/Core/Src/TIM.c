/*
 * TIM.c
 *
 *  Created on: Dec 27, 2023
 *      Author: Ashton Dudley
 */


#include "TIM.h"
#include "stm32f4xx.h"

float TIM_ConvertValue(uint16_t inputValue);


/**
 * @brief Throttle Input Module
 * @retval None
 */
float TIM_ConvertValue(uint16_t  inputValue)
{
	uint16_t xarray[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
	float yarray[] = {0.0f, 2.5f, 7.5f, 12.5f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f, 75.0f, 100.0f};

	float x0 = 0.0f, x1 = 0.0f, y0 = 0.0f, y1 = 0.0f;

	int i;
	while (xarray[i] < inputValue) {
		i++;
	}
	x0 = xarray[i - 1];
	x1 = xarray[i];
	y0 = yarray[i - 1];
	y1 = yarray[i];

	float outputValue = y1 + (inputValue - x1) * ((y1 - y0) / (x1 - x0)); // Linear Approximation
	return outputValue;
}
