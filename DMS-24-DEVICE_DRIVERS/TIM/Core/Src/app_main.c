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

/* External Flags */
extern _Bool dataReadyFlag;
extern _Bool changeThrottleMapFlag;
extern _Bool forwardDirFlag;
extern _Bool reverseDirFlag;


/* This fucntion MUST be synced with the state_codes_t enum,
 * ie if entry_state = 0 then, entry =0.
 * to add a new state, add to the enum AND the
 */
int (*state[])(
		void) = {entry_state, idle_state, forward_state, reverse_state, end_state
};


struct transition {
	state_codes_t src_state;
	ret_codes_t ret_code;
	state_codes_t dst_state;
};

struct transition state_transitions[] = {
	{entry,       ok,                 idle},
	{entry,       fail,               entry},
	{idle,        dir_forward,  		forward},
	{idle,        dir_reverse,  		reverse},
	{idle,        repeat,             idle},
	{forward,     repeat,             forward},
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
	for (int i = 0; i < sizeof(state_transitions) / sizeof(state_transitions[0]); i++) {

		if (state_transitions[i].src_state == cur_state && state_transitions[i].ret_code == rc) {


			switch (state_transitions[i].src_state) {

			case entry:
				break;

			case idle:
				break;

			case forward:
				switch (rc) {
				case adc_data_ready:
					TIM_ProcessData();
					dataReadyFlag = 0;
					break;
				case ok:
					break;
				case fail:
					break;
				case change_map:
					TIM_ChangeThrottleMap();
					changeThrottleMapFlag = 0;
					break;
				case vehicle_stopped:
					break;
				default:
					break;
				}

			case reverse:
				break;

			default:
				break;

			}

			return state_transitions[i].dst_state; // Return the next state
		}
	}
	// Return an error code indicating that no matching transition was found
	return -1;
}





/* These transition functions are called at the start of their corresponding state,
 * They return a ret_codes_t to decide what to do next
 */

int entry_state(void){
	// TODO
	// Check if all systems are okay
	if (TIM_AppsAgreement() == PDP_OKAY){
		return ok;
	}
	else {
		return fail;
	}
}
int idle_state(void){
	// TODO
	// Check if car is moving -> return fail
	// Check if driver selects forwards -> Set Forward Throttle Map
	if (forwardDirFlag){
		return dir_forward;
	}
	else if (reverseDirFlag){
		return dir_reverse;
	}
	else {
		return repeat;
	}
	// Check if driver selects reverse 	-> Set Reverse Throttle Map
}
int forward_state(void){
	// Check CANbus -> CanBUS
	// Check if car is stopped -> STATE -> Idle, set idle throttle map
	// Check if any data is ready -> deinterleve and send motor data
	if (dataReadyFlag){
		return adc_data_ready;
	}
	// Check for driver inputs -> change thottle map
	else if (changeThrottleMapFlag){
		return change_map;
	}
	// All okay -> wait and do nothing
	else {
		return repeat;
	}
}
int reverse_state(void){
	if (dataReadyFlag){
		return adc_data_ready;
	}
	return 0;
}
int end_state(void){
	return 0;
}



