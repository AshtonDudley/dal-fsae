/*
 * PEDAL_PLAUSIBLITY.c
 *
 *  Created on: Dec 31, 2023
 *  Author: Ashton Dudley
 */

#include "PEDAL_PLAUSIBILITY.h"


volatile uint32_t PAG_fault = 0;
volatile bool AAC_fault = false;
volatile bool SPA_fault	= false;


/**
  * @brief  Pedal Agreement Check. Latches until throttle pedal is released
  * @retval 0 no fault
  * @retval 1 PAG_fault, both pedals currently active
  * @retval 2 PAG_fault, waiting for latch to reset
  */
uint32_t PDP_PedealAgreement(uint32_t apps, uint32_t fbps){ 		// PAG_fault active
	if (apps > APPS_PAG_THRESHOLD && fbps > FBPS_PAG_THRESHOLD){
		PAG_fault = 1;
		return PAG_fault;
	}

	else if (PAG_fault > 0 && apps < APPS_PAG_RESET_THRESHOLD){		// Check if latch can be reset
		PAG_fault = 0;
		return PAG_fault;
	}
	else if(PAG_fault > 0 && apps > APPS_PAG_THRESHOLD){ 			// Waiting for latch to reset fault
		PAG_fault = 2;
		return 2;
	}
	else{
		return PAG_fault;
	}
}
