/*
 * PEDAL_PLAUSIBLITY.c
 *
 *  Created on: Dec 31, 2023
 *  Author: Ashton Dudley
 */

#include "PEDAL_PLAUSIBILITY.h"


volatile PDP_StatusTypeDef PAG_fault = PDP_OKAY;
volatile PDP_StatusTypeDef AAC_fault = PDP_OKAY;
volatile PDP_StatusTypeDef SPA_fault = PDP_OKAY;


/**
  * @brief  Pedal Agreement Check. Latches until throttle pedal is released
  * @retval 0 no fault
  * @retval 1 PAG_fault, both pedals currently active
  * @retval 2 PAG_fault, waiting for latch to reset
  */
PDP_StatusTypeDef PDP_PedealAgreement(uint32_t apps, uint32_t fbps){ 		// PAG_fault active
	if (apps > APPS_PAG_THRESHOLD && fbps > FBPS_PAG_THRESHOLD){
		PAG_fault = PDP_ERROR;
		return PAG_fault;
	}

	else if (PAG_fault != PDP_OKAY && apps < APPS_PAG_RESET_THRESHOLD){		// Check if latch can be reset
		PAG_fault = 0;
		return PAG_fault;
	}
	else if(PAG_fault != PDP_OKAY && apps > APPS_PAG_THRESHOLD){ 			// Waiting for latch to reset fault
		PAG_fault = PDP_RESET_LATCH;
		return PAG_fault;
	}
	else{
		return PAG_fault;
	}
}


// TODO APPS AGREEMENT FAULT LOGIC

// TODO SIGNAL PLAUSIBILITY FAULT LOGIC




