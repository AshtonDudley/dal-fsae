/*
 * TIM.h
 *
 *  Created on: Dec 27, 2023
 *  Author: Ashton Dudley
 */

#ifndef SRC_TIM_H_
#define SRC_TIM_H_

/*	INCLUDES	*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "PEDAL_PLAUSIBILITY.h"


/* 	DEFINES 	*/

#define ADC_BUFFER_LEN 			4096 	// must be a power of two, when divided by the total number of ADC channels//
#define ADC_CHANNEL_BUFFER_LEN 	2048 	// buffer length per ADC channel, must be divisible into ADC_BUFFER_LEN

#define APPS1_BUF_ADDR 	0;				// Which array element APPS1 data in the ADC array begins
#define BPS_BUF_ADDR 	1;				// Which array element BPS data in the ADC array begins

#define MAX_THROTTLE_PROFILE_NAME_LENGTH 8u


typedef struct adcBufferChannel_s {
	uint16_t adcAPPS1;
	uint16_t adcBPS;
}adcBufferChannel_t;



float TIM_GetScaledThrottle(float inputVal);

float TIM_GetExpoedValue(float rawIn, float expoVal);

void TIM_Init(ADC_HandleTypeDef *TIM_hadc1);

uint16_t TIM_ConvertValue(uint16_t inputValue);

uint16_t TIM_ConvertValueLinearApprox(uint16_t inputValue);

uint16_t TIM_Average(uint16_t adc_buffer[], uint16_t depth);

uint16_t TIM_DeInterleave(uint16_t unsortedBuf[], uint16_t startPoint, uint16_t depth);

void TIM_OutputDAC(uint16_t DAC_Output);

#endif /* SRC_TIM_H_ */
