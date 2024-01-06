	/*
 * app_main.h
 *
 *  Created on: Jan 6, 2024
 *      Author: Ashton Dudley
 */

#ifndef INC_APP_MAIN_H_
#define INC_APP_MAIN_H_

/* state machine */

typedef enum {
	ok				= 0x00u,
	fail			= 0x01u,
	repeat			= 0x02u,
	dir_forward		= 0x03u,
	dir_reverse		= 0x04u,
	vehicle_stopped	= 0x05u,
	change_map		= 0x06u,
	adc_data_ready	= 0x07u
} ret_codes_t;

/* int (*state[])(void) and enum below must be in sync! */
typedef enum { entry, idle, forward, reverse, end }state_codes_t;


void AppConfig();
void AppMain();

state_codes_t lookup_transitions(state_codes_t cur_state, ret_codes_t rc);




/* called when entering state */
int entry_state(void);		// car is initialized
int idle_state(void);		// car is not moving
int forward_state(void);
int reverse_state(void);
int end_state(void);		// shutdown sequence






#endif /* INC_APP_MAIN_H_ */
