/*
 * TIM.h
 *
 *  Created on: Dec 27, 2023
 *      Author: Ashton Dudley
 */

#ifndef SRC_TIM_H_
#define SRC_TIM_H_



void TIM_Init(ADC_HandleTypeDef *TIM_hadc1);

float TIM_ConvertValue(uint16_t inputValue);


#endif /* SRC_TIM_H_ */
