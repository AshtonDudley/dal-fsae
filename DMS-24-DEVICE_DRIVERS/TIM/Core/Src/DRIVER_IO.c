/*
 * DRIVER_IO.c
 *
 *  Created on: Jan 5, 2024
 *      Author: Ashton Dudley
 */


#include "DRIVER_IO.h"


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0) {
	  // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_P);
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	} else {
		__NOP();
  }
}
