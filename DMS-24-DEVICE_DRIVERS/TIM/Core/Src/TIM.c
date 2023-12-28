/*
 * TIM.c
 *
 *  Created on: Dec 27, 2023
 *      Author: Ashton Dudley
 */


#include "stm32f4xx_hal.h"
#include "TIM.h"


#define ADC_BUFFER_LEN 4096
uint16_t adc_buf[ADC_BUFFER_LEN];





/**
 * @brief Throttle Input Module
 * @return Throttle value scaled to desired map
 */
uint16_t TIM_ConvertValue(uint16_t  inputValue)
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

uint16_t TIM_Average(uint16_t adc_buffer[]){
	uint16_t total = 0;
	for (int i = 0; i < ADC_BUFFER_LEN / 2; i++) {
		total += adc_buffer[i];
	}
	return total / (ADC_BUFFER_LEN / 2);
}


/**
  * @brief  Used to average half of the buffer, and output a 0-3.3V signal to the
  * motor controller.
  *	Note, the signal is amplified to a 0-5V range using a hardware amp.
  * Motor Data sheet: https://wiki.neweagle.net/docs/Rinehart/PM100_User_Manual_3_2011.pdf
  * @return 0-3.3v Analog Voltage
  */
void TIM_OutputDAC(){

}



/**
  * @brief  This function is executed when half the TIM buffer is full
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
}


/**
  * @brief  This function is executed when  TIM buffer is completely full
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

}

