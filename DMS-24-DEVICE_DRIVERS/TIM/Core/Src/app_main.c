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

#define EXIT_STATE end
#define ENTRY_STATE entry


/* Handles */

extern DAC_HandleTypeDef hdac;
extern ADC_HandleTypeDef hadc1;




/* This fucntion MUST be synced with the state_codes_t enum,
 * ie if entry_state = 0 then, entry =0.
 * to add a new state, add to the enum AND the
 */
int (*state[])(void) = {entry_state, idle_state, forward_state, reverse_state, end_state};


struct transition {
  state_codes_t src_state;
  ret_codes_t 	ret_code;
  state_codes_t dst_state;
};

struct transition state_transitions[] = {
  {entry,       ok,                 idle},
  {entry,       fail,               entry},
  {idle,        dir_forward,  		forward},
  {idle,        dir_reverse,  		reverse},
  {idle,        repeat,             idle},
  {forward,     ok,                 forward},
  {forward,     fail,               forward },
  {forward,     change_map,         forward},
  {forward,     vehicle_stopped,	idle},
  {forward,     adc_data_ready,     forward},
  {reverse,     ok,                 reverse},
  {reverse,     fail,               forward },
  {reverse,     vehicle_stopped,    idle},
};




void AppConfig() {

	TIM_Init(&hadc1);
	HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);

}

void AppMain() {

	uint32_t prevTime = 0, curTime = 0;

	state_codes_t cur_state = ENTRY_STATE;
	ret_codes_t rc;
	int (*state_fun)(void);

	while (1) {
		//int voltage = TIM_ConvertValue(128);
		//TIM_OutputDAC(voltage);
		curTime = HAL_GetTick();

		if (curTime - prevTime >= 500) {
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
			prevTime = curTime;
		}

	    state_fun = state[cur_state];
	    rc = state_fun();

	    cur_state = lookup_transitions(cur_state, rc);

	}
}

state_codes_t lookup_transitions(state_codes_t cur_state, ret_codes_t rc){
	return 0;
}

int entry_state(void){
	return 0;
}
int idle_state(void){
	return 0;
}
int forward_state(void){
	return 0;
}
int reverse_state(void){
	return 0;
}
int end_state(void){
	return 0;
}



