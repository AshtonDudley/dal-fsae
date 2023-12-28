/*
 * TIM.h
 *
 *  Created on: Dec 27, 2023
 *      Author: Ashton Dudley
 */

#ifndef SRC_TIM_H_
#define SRC_TIM_H_



void TIM_Init(ADC_HandleTypeDef *TIM_hadc1);

uint16_t TIM_ConvertValue(uint16_t inputValue);

uint16_t TIM_Average(uint16_t adc_buffer[]);

void TIM_OutputDAC(uint16_t DAC_Output);

#endif /* SRC_TIM_H_ */
