/*
 * PEDAL_PLAUSIBILITY.h
 *
 *  Created on: Dec 31, 2023
 *      Author: Ashton Dudley
 */

#ifndef INC_PEDAL_PLAUSIBILITY_H_
#define INC_PEDAL_PLAUSIBILITY_H_

/* INCLUDES	*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "stdlib.h"


/**
  * @brief  HAL Status structures definition
  */
typedef enum
{
  PDP_OKAY 			= 0x00U,
  PDP_ERROR	    	= 0x01U,
  PDP_RESET_LATCH   = 0x02U,
  PDP_TIMEOUT 		= 0x03U
} PDP_StatusTypeDef;


/* DEFINES 	*/
#define APPS_PAG_THRESHOLD 			107 	// APPS1 Threshold 1.25V	(1.25V / 3.3V) * 256
#define FBPS_PAG_THRESHOLD 			19		// FBPS  Threshold 0.25V	(0.25V / 3.3V) * 256
#define APPS_PAG_RESET_THRESHOLD	19		// RESET Threshold 0.25V	(0.25V / 3.3V) * 256

#define APPS_AAC_ERROR_THRESHOLD	10		// AAC %error Threshold as whole number (0-100)

#define SPA_MIN_THRESHOLD			19		// 0.25V (Short to ground threshold)
#define SPA_MAX_THRESHOLD			255 	// 3.3V  (Short to 3.3V threshold)

PDP_StatusTypeDef PDP_PedealAgreement(uint32_t apps, uint32_t fbps);

PDP_StatusTypeDef PDP_AppsAgreement (uint32_t apps1, uint32_t apps2);

PDP_StatusTypeDef PDP_ThresholdCheck (uint32_t sensor);

#endif /* INC_PEDAL_PLAUSIBILITY_H_ */
