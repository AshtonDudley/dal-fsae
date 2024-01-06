/*
 * TIM.c
 *
 *  Created on: Dec 27, 2023
 *      Author: Ashton Dudley
	 */


#include "TIM.h"

extern DAC_HandleTypeDef hdac;

uint8_t dataReadyFlag = 0;

uint16_t adc_buf[ADC_BUFFER_LEN];	 							// Interlaced ADC data,  the buffer size can be increased to add a delay in ADC processing, can be useful if the main function is stuck.
uint16_t dac_buf[ADC_BUFFER_LEN];

static volatile uint16_t *inBufPtr;								// TODO These are currently unused
static volatile uint16_t *outBufPtr = &adc_buf[0];				// https://www.youtube.com/watch?v=zlGSxZGwj-E for how to use them

adcBufferChannel_t adcBufferChannel = (adcBufferChannel_t){};	// De-Interlaced ADC Data




thottleMap_t currentThottleMap = (thottleMap_t) {
	.xarray = {0.0f, 25.6f, 51.2f, 76.8f, 102.4f, 128.0f, 153.6f, 179.2f, 204.8f, 230.4f, 256.0f},			// NOTE: The last value on this array MUST be larger then the largest possible ADC input value
	.yarray = {0.0f, 102.4f, 307.2f, 512.0f, 819.2f, 1228.8f, 1638.4f, 2048.0f, 2457.6f, 3072.0f, 4096.0f}
};

/**
 * @brief Throttle Input Module
 * @return Throttle value scaled to desired map
 */
uint16_t TIM_ConvertValueLinearApprox(uint16_t inputValue, thottleMap_t *thottleMap)
{

	float x0 = 0.0f, x1 = 0.0f, y0 = 0.0f, y1 = 0.0f;

	int i = 0;
	while (thottleMap->xarray[i] < inputValue && i < 11) { // TODO: Improve the safety of this function
		i++;
	}
	x0 = thottleMap->xarray[i - 1];
	x1 = thottleMap->xarray[i];
	y0 = thottleMap->yarray[i - 1];
	y1 = thottleMap->yarray[i];

	uint16_t outputValue =  (y1 + (inputValue - x1) * ((y1 - y0) / (x1 - x0))); 	// Linear Approximation, On a scale of 1-100
	//outputValue = outputValue / 30.3030f * 4096 / 3.3; 								// Convert Value from 1-100 scale to 1-4096
	return outputValue;
}



/**
  * @brief  Used to average half of the sorted buffer, and output a 0-3.3V signal to the
  * motor controller.
  *	Note, the signal is amplified to a 0-5V range using a hardware amp.
  * Motor Data sheet: https://wiki.neweagle.net/docs/Rinehart/PM100_User_Manual_3_2011.pdf
  * @todo Replace with a moving average algorithm, for large buffer sizes, an overflow may occur
  * @return averages first half the the input arrays
  */
uint16_t TIM_Average(uint16_t adc_buffer[], uint16_t depth){	// TODO FIX THIS ASAP ADC_CHANNEL_BUFFER_LEN / 2
	uint32_t total = 0;
	for (int i = 0; i < (depth / 2); i++) {  	// TODO Change buffer since to channel size
		total += adc_buffer[i];									// TODO Change to moving average
	}
	uint16_t avg = total / (depth / 2);
	return avg;
}
/**
  * @brief  De-Interleves the DMA buffer, into two separate buffers
  * @note we should look at improving this function so its scalable for more channels
  * and to improve efficiency. We may also want to pass a pointer to the struct, rather then
  * using global variables
  * @retval None
  */
uint16_t TIM_DeInterleave(uint16_t unsortedBuf[], uint16_t startPoint, uint16_t depth) {
	uint16_t DeInterleavedBuf[ADC_CHANNEL_BUFFER_LEN];

	for (int i = 0, j = 0; i < (depth + startPoint); i++, j += 2) {
		DeInterleavedBuf[i] = unsortedBuf[j + startPoint];
	}
	return TIM_Average(DeInterleavedBuf, depth);
}



/**
  * @brief  Uses DAC to output a 0-3.3V signal to the motor controller.
  *	Note, the signal is amplified to a 0-5V range using a hardware amp.
  * Motor Data sheet: https://wiki.neweagle.net/docs/Rinehart/PM100_User_Manual_3_2011.pdf
  * @retval None
  */
void TIM_OutputDAC(uint16_t DAC_Output){
	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_Output);
}


/**
  * @brief  Must be used to initialize ADC with DMA
  * @param ADC_HandleTypeDef
  * @retval None
  */
void TIM_Init(ADC_HandleTypeDef *TIM_hadc1){
	HAL_ADC_Start_DMA(TIM_hadc1, (uint32_t*)adc_buf, ADC_BUFFER_LEN);
}

/**
  * @brief  This function is executed when half the TIM buffer is full
  * @retval None
  */

void TIM_ProcessData(){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);	// DEBUG LED TOGGLE FOR TIME PROFILE
	adcBufferChannel.adcAPPS1  =	TIM_DeInterleave(adc_buf, 0, 64); 	// The depth can be changed to control how many values we average
	adcBufferChannel.adcBPS    =	TIM_DeInterleave(adc_buf, 1, 64);	// TODO Change to a smaller buffer (128) which samples slower

	// Plausibility Checks
	PDP_StatusTypeDef PAG = PDP_PedealAgreement(adcBufferChannel.adcAPPS1, adcBufferChannel.adcBPS);
	switch (PAG){
		case PDP_OKAY:
			uint32_t motorControllerOutputVoltage = TIM_ConvertValueLinearApprox(adcBufferChannel.adcAPPS1, &currentThottleMap);
			TIM_OutputDAC(motorControllerOutputVoltage);
			break;
		case PDP_ERROR:			// TODO add driver notifications and CAN logging for fault cases
			TIM_OutputDAC(CUT_MOTOR_SIGNAL);
			break;
		case PDP_RESET_LATCH:	// TODO add driver notifications and CAN logging for fault cases
			break;
			TIM_OutputDAC(CUT_MOTOR_SIGNAL);
		default:
			TIM_OutputDAC(CUT_MOTOR_SIGNAL);
			break;
	}
	// TEST CODE FOR AAC

	PDP_StatusTypeDef AAG = PDP_AppsAgreement(adcBufferChannel.adcAPPS1, adcBufferChannel.adcBPS);
	switch (AAG){
		case PDP_OKAY:
			// HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			break;
		case PDP_ERROR:			// TODO add driver notifications and CAN logging for fault cases
			// HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			break;
		default:
			break;
	}

	// END TEST CODE FOR AAC

	dataReadyFlag = 0;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);	// DEBUG LED TOGGLE FOR TIME PROFILE
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1){
	inBufPtr  = &adc_buf[0];
	outBufPtr = &dac_buf[0];
	dataReadyFlag = 1;
	//TIM_DeInterleave(&adcBufferChannel, adc_buf);

	// Average the first half of the buffer


	// HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);	// Flashing this LED lets us monitor the state
}															// of the buffer using the oscilloscope


/**
  * @brief  This function is executed when  TIM buffer is completely full
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1){
	inBufPtr  = &adc_buf[ADC_BUFFER_LEN / 2];
	outBufPtr = &dac_buf[ADC_BUFFER_LEN / 2];
	dataReadyFlag = 1;
	// HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
}

