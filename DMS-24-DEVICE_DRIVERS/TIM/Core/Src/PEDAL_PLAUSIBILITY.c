/*
 * PEDAL_PLAUSIBLITY.c
 *
 *  Created on: Dec 31, 2023
 *  Author: Ashton Dudley
 */

#include "PEDAL_PLAUSIBILITY.h"

/* 	true == fault 	*/
volatile bool PAG_fault = false;
volatile bool AAC_fault = false;
volatile bool SPA_fault	= flase;


/**
  * @brief  Pedal Agreement Check. Latches until throttle pedal is released
  * @retval true pedal agreement fault
  * @retval false no fault
  */
bool PDP_PedealAgreement(uint32_t apps, uint32_t fbps){
	if (apps > APPS_PAG_THRESHOLD && fbps > FBPS_PAG_THRESHOLD){
		PAG_fault = true;
		return PAG_fault;
	}
	else if (PAG_fault == true && apps < APPS_PAG_RESET_THRESHOLD){
		PAG_fault = false;
		return PAG_fault;
	}
	else{
		return PAG_fault;
	}
}
