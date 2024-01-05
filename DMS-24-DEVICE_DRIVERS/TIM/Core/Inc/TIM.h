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





#define ADC_BUFFER_LEN 4096 			// must be a power of two, when divided by the total number of ADC channels//
#define ADC_CHANNEL_BUFFER_LEN 2048 	//buffer length per ADC channel, must be divisible into ADC_BUFFER_LEN

#define MAX_THROTTLE_PROFILE_NAME_LENGTH 8u

typedef struct throttleProfileConfig_s {
    uint8_t thrExpo;
    uint8_t regenExpo;
    uint8_t crossover;
    uint8_t throttleProfile_Type;
    char profileName[MAX_THROTTLE_PROFILE_NAME_LENGTH + 1]; // Descriptive name for throttle profile
} throttleProfileConfig_t;

typedef struct adcBufferChannel_s {
	uint16_t adcThrottle_buf[ADC_CHANNEL_BUFFER_LEN];
	uint16_t adcThrottle;
	uint16_t adcBPS_buf[ADC_CHANNEL_BUFFER_LEN];
	uint16_t adcBPS;
}adcBufferChannel_t;



float TIM_GetScaledThrottle(float inputVal);

float TIM_GetExpoedValue(float rawIn, float expoVal);

void TIM_Init(ADC_HandleTypeDef *TIM_hadc1);

uint16_t TIM_ConvertValue(uint16_t inputValue);

uint16_t TIM_ConvertValueLinearApprox(uint16_t inputValue);

uint16_t TIM_Average(uint16_t adc_buffer[]);

void TIM_DeInterleave(adcBufferChannel_t *adcBuf, uint16_t unsortedBuf[]);

void TIM_OutputDAC(uint16_t DAC_Output);

#endif /* SRC_TIM_H_ */
