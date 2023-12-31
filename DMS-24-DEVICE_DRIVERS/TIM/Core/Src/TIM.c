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

	return throttleOut * 2048;
}


/**
 * @brief Throttle Input Module
 * @return Throttle value scaled to desired map
 */
uint16_t TIM_ConvertValueLinearApprox(uint16_t inputValue)
{
	float xarray[] = {0.0f, 26.0f, 51.0f, 77.0f, 102.0f, 128.0f, 153.0f, 179.0f, 204.0f, 230.0f, 256.0f	};
	//uint16_t xarray[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
	float yarray[] = {0.0f, 2.5f, 7.5f, 12.5f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f, 75.0f, 100.0f};

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
	uint32_t convertedVoltage = TIM_ConvertValue(outputVoltage);
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

