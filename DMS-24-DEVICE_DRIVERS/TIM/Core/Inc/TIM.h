/*
 * TIM.h
 *
 *  Created on: Dec 27, 2023
 *      Author: Ashton Dudley
 */

#ifndef SRC_TIM_H_
#define SRC_TIM_H_

#define MAX_THROTTLE_PROFILE_NAME_LENGTH 8u

typedef struct throttleProfileConfig_s {
    uint8_t thrExpo;
    uint8_t regenExpo;
    uint8_t crossover;
    uint8_t throttleProfile_Type;
    char profileName[MAX_THROTTLE_PROFILE_NAME_LENGTH + 1]; // Descriptive name for throttle profile
} throttleProfileConfig_t;


float TIM_GetScaledThrottle(float inputVal);

float TIM_GetExpoedValue(float rawIn, float expoVal);

void TIM_Init(ADC_HandleTypeDef *TIM_hadc1);

uint16_t TIM_ConvertValue(uint16_t inputValue);

uint16_t TIM_ConvertValueLinearApprox(uint16_t inputValue);

uint16_t TIM_Average(uint16_t adc_buffer[]);

void TIM_OutputDAC(uint16_t DAC_Output);

#endif /* SRC_TIM_H_ */
