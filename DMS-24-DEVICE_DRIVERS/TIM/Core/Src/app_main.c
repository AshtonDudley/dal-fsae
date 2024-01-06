/*
 * app_main.c
 *
 *  Created on: Jan 6, 2024
 *  Author: Ashton Dudley
 *  This file is used to initialize the application, and to avoid writing code
 *  inside main.c, as STM32cubeIDE likes to overwrite things.
 *
 */

#include "app_main.h"
#include "stm32f4xx_hal.h"
#include "TIM.h"
#include "DRIVER_IO.h"
#include "stdbool.h"

/* Handles */

extern DAC_HandleTypeDef hdac;
extern ADC_HandleTypeDef hadc1;


void AppConfig() {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	TIM_Init(&hadc1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);

	HAL_DAC_Start(&hdac,DAC1_CHANNEL_1);

}

<<<<<<< HEAD
#define END end
#define ENTRY entry
=======
>>>>>>> parent of b00526e (setup structure and function prototypes for state machine)

void AppMain() {

	uint32_t prevTime =0, curTime = 0;

<<<<<<< HEAD


	while (1) {
=======
	while (1){
>>>>>>> parent of b00526e (setup structure and function prototypes for state machine)
		//int voltage = TIM_ConvertValue(128);
		//TIM_OutputDAC(voltage);
		curTime = HAL_GetTick();

		if (curTime - prevTime >= 500) {
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
			prevTime = curTime;
		}
		extern uint8_t dataReadyFlag;
		if (dataReadyFlag == 1) {
			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
			TIM_ProcessData();
		}
	}
}
