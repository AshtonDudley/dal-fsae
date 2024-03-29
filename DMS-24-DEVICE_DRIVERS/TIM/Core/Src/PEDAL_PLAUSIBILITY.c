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
	if (apps > APPS_PAG_THRESHOLD && fbps > FBPS_PAG_THRESHOLD) {
		PAG_fault = PDP_ERROR;
		return PAG_fault;
	}

	else if (PAG_fault != PDP_OKAY && apps < APPS_PAG_RESET_THRESHOLD) {	// Check if latch can be reset
		PAG_fault = 0;
		return PAG_fault;
	} else if (PAG_fault != PDP_OKAY && apps > APPS_PAG_THRESHOLD) {		// Waiting for latch to reset fault
		PAG_fault = PDP_RESET_LATCH;
		return PAG_fault;
	} else {
		return PAG_fault;
	}
}



/**
  * @brief  APPS Agreement Check. Checks if both APPS sensors are within
  * %error threshold of each other.
  * @retval 0 no fault
  * @retval 1 AAC_fault, difference between pedal sensors > %threshold
  */
PDP_StatusTypeDef PDP_AppsAgreement(uint32_t apps1, uint32_t apps2) {
	float absDif = abs((int) apps1 - (int) apps2);
	float percentDifference = (absDif / ((apps1 + apps2) / 2)) * 100; // Calculating percent Difference

	if (percentDifference >= APPS_AAC_ERROR_THRESHOLD) {
		AAC_fault = PDP_ERROR;
		return AAC_fault;
	} else {
		AAC_fault = PDP_OKAY;
		return AAC_fault;
	}

}
// TODO SIGNAL PLAUSIBILITY FAULT LOGIC
PDP_StatusTypeDef PDP_ThresholdCheck(uint32_t sensor) {
	if (sensor <= SPA_MIN_THRESHOLD || sensor >= SPA_MAX_THRESHOLD) {
		SPA_fault = PDP_ERROR;
		return PDP_ERROR;
	} else {
		SPA_fault = PDP_OKAY;
		return PDP_OKAY;
	}

}



