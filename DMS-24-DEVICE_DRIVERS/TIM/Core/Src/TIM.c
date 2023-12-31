/*
 * TIM.c
 *
 *  Created on: Dec 27, 2023
 *      Author: Ashton Dudley
 */


#include "stm32f4xx_hal.h"
#include "TIM.h"

extern DAC_HandleTypeDef hdac;

#define ADC_BUFFER_LEN 128
uint16_t adc_buf[ADC_BUFFER_LEN];


throttleProfileConfig_t throttleProfileConfig = (throttleProfileConfig_t){
	.thrExpo = 50,
	.regenExpo = 50,
	.crossover = 30
};

/**
 * @brief Scales throttle input so regen is < 0 and throttle is > 0
 * @return Scaled throttle values with regen -1 to 0 and throttle is 0 to 1
 */
float TIM_GetScaledThrottle(float inputVal){
	float maxVal = 1.00f - (throttleProfileConfig.crossover / 100.00f);
	if (inputVal < 0.00f) {
		maxVal = throttleProfileConfig.crossover / 100.00f;
	}
	return inputVal / maxVal;
}


float TIM_GetExpoedValue(float rawIn, float expoVal){
	float expoF = expoVal / 100.00f;
	float power5 = rawIn * rawIn * rawIn * rawIn * rawIn;
	return power5 * expoF + rawIn  * (1.00f - expoF);
}


uint16_t TIM_ConvertValue(uint16_t inputValue)
{
	float inputPos = (inputValue / 255.00f) - (throttleProfileConfig.crossover / 100.00f); // Scale 0-1 and then shift down so regen is - and throttle is positive.
	float crossoverF = (throttleProfileConfig.crossover / 100.00f);
	float scaledThrottle = TIM_GetScaledThrottle(inputPos);
	float expoedThrottle = 0.00f;
	float throttleOut = 0.00f;

	if (scaledThrottle < 0.00f){
		expoedThrottle = TIM_GetExpoedValue(scaledThrottle, throttleProfileConfig.thrExpo);
		throttleOut = expoedThrottle * crossoverF + crossoverF;
	} else {
		expoedThrottle = TIM_GetExpoedValue(scaledThrottle, throttleProfileConfig.regenExpo);
		throttleOut = expoedThrottle * (1.00f - crossoverF) + crossoverF;
	}

	return throttleOut * 4096;
}


/**
 * @brief Throttle Input Module
 * @return Throttle value scaled to desired map
 */
uint16_t TIM_ConvertValueLinearApprox(uint16_t inputValue)
{
	float xarray[] = {0.0f, 23.3f, 46.6f, 69.9f, 93.2f, 116.5f, 139.8f, 163.1f, 186.4f, 209.7f, 233.0f};
	float yarray[] = {0.0f, 102.4f, 307.2f, 512.0f, 819.2f, 1228.8f, 1638.4f, 2048.0f, 2457.6f, 3072.0f, 4096.0f};


	float x0 = 0.0f, x1 = 0.0f, y0 = 0.0f, y1 = 0.0f;

	int i = 0;
	while (xarray[i] < inputValue) {
		i++;
	}
	x0 = xarray[i - 1];
	x1 = xarray[i];
	y0 = yarray[i - 1];
	y1 = yarray[i];

	uint16_t outputValue =  (y1 + (inputValue - x1) * ((y1 - y0) / (x1 - x0))); 	// Linear Approximation, On a scale of 1-100
	outputValue = outputValue / 30.3030f * 4096 / 3.3; 								// Convert Value from 1-100 scale to 1-4096
	return outputValue;
}

/**
  * @brief  Must be used to initialize ADC with DMA
  * @param ADC_HandleTypeDef
  * @retval None
  */
void TIM_Init(ADC_HandleTypeDef *TIM_hadc1){
	HAL_ADC_Start_DMA(TIM_hadc1, (uint32_t*)adc_buf, ADC_BUFFER_LEN);
}


/**
  * @brief  Used to average half of the buffer, and output a 0-3.3V signal to the
  * motor controller.
  *	Note, the signal is amplified to a 0-5V range using a hardware amp.
  * Motor Data sheet: https://wiki.neweagle.net/docs/Rinehart/PM100_User_Manual_3_2011.pdf
  * @todo Replace with a moving average algorithm, for large buffer sizes, an overflow may occur
  * @return averages first half the the input array
  */
uint16_t TIM_Average(uint16_t adc_buffer[]){
	uint32_t total = 0;
	for (int i = 0; i < (ADC_BUFFER_LEN / 2); i++) {
		total += adc_buffer[i];
	}
	uint16_t avg = total / (ADC_BUFFER_LEN / 2);
	return avg;
}


/**
  * @brief  Uses DAC to output a 0-3.3V signal to the motor controller.
  *	Note, the signal is amplified to a 0-5V range using a hardware amp.
  * Motor Data sheet: https://wiki.neweagle.net/docs/Rinehart/PM100_User_Manual_3_2011.pdf
  * @retval None
  */
void TIM_OutputDAC(uint16_t DAC_Output){
	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_Output);
}



/**
  * @brief  This function is executed when half the TIM buffer is full
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1){

	uint32_t outputVoltage = TIM_Average(adc_buf);
	uint32_t convertedVoltage = TIM_ConvertValueLinearApprox(outputVoltage);
	TIM_OutputDAC(convertedVoltage);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
}


/**
  * @brief  This function is executed when  TIM buffer is completely full
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

}

