/*
 * DRIVER_IO.c
 *
 *  Created on: Jan 5, 2024
 *      Author: Ashton Dudley
 */

#include "DRIVER_IO.h"

#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "TIM.h"

#define CHANGE_THROTTLE_MAP_PIN GPIO_PIN_0
#define FORWARD_DIRECTION_PIN GPIO_PIN_1
#define REVERSE_DIRECTION_PIN GPIO_PIN_2

_Bool changeThrottleMapFlag = 0;
_Bool forwardDirFlag = 0;
_Bool reverseDirFlag = 0;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == CHANGE_THROTTLE_MAP_PIN) {
		changeThrottleMapFlag = 1;
	}
	else if (GPIO_Pin == FORWARD_DIRECTION_PIN){
		forwardDirFlag = 1;
	}
	else if (GPIO_Pin == REVERSE_DIRECTION_PIN){
		reverseDirFlag = 1;
	}



	else {
		__NOP();
  }
}
