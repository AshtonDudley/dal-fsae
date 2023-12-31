/*
 * PEDAL_PLAUSIBILITY.h
 *
 *  Created on: Dec 31, 2023
 *      Author: Ashton Dudley
 */

#ifndef INC_PEDAL_PLAUSIBILITY_H_
#define INC_PEDAL_PLAUSIBILITY_H_

/* INCLUDES	*/
#include "stdbool.h"


/* DEFINES 	*/
#define APPS_PAG_THRESHOLD 			107 	// APPS1 Threshold 1.25V	(1.25V / 3.3V) * 256
#define FBPS_PAG_THRESHOLD 			19		// FBPS  Threshold 0.25V	(0.25V / 3.3V) * 256
#define APPS_PAG_RESET_THRESHOLD	19		// RESET Threshold 0.25V	(0.25V / 3.3V) * 256

#endif /* INC_PEDAL_PLAUSIBILITY_H_ */
