/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CURRENT_SENSE_Pin GPIO_PIN_0
#define CURRENT_SENSE_GPIO_Port GPIOC
#define MC_REGEN_Pin GPIO_PIN_1
#define MC_REGEN_GPIO_Port GPIOC
#define RTDS_LED_Pin GPIO_PIN_2
#define RTDS_LED_GPIO_Port GPIOC
#define RTDS_Pin GPIO_PIN_3
#define RTDS_GPIO_Port GPIOC
#define APPS1_Pin GPIO_PIN_0
#define APPS1_GPIO_Port GPIOA
#define APPS2_Pin GPIO_PIN_1
#define APPS2_GPIO_Port GPIOA
#define FBPS_Pin GPIO_PIN_2
#define FBPS_GPIO_Port GPIOA
#define RBPS_Pin GPIO_PIN_3
#define RBPS_GPIO_Port GPIOA
#define MC_ACCELL_Pin GPIO_PIN_4
#define MC_ACCELL_GPIO_Port GPIOA
#define AAC_FAULT_Pin GPIO_PIN_6
#define AAC_FAULT_GPIO_Port GPIOA
#define AAC_FAULT_EXTI_IRQn EXTI9_5_IRQn
#define PAG_FAULT_Pin GPIO_PIN_7
#define PAG_FAULT_GPIO_Port GPIOA
#define PAG_FAULT_EXTI_IRQn EXTI9_5_IRQn
#define SPA_FAULT_Pin GPIO_PIN_4
#define SPA_FAULT_GPIO_Port GPIOC
#define FRWD_DRIVE_SNGL_Pin GPIO_PIN_5
#define FRWD_DRIVE_SNGL_GPIO_Port GPIOC
#define VCU_FAULT_Pin GPIO_PIN_1
#define VCU_FAULT_GPIO_Port GPIOB
#define VCU_DEBUG_LED_Pin GPIO_PIN_13
#define VCU_DEBUG_LED_GPIO_Port GPIOB
#define RVRS_DRIVE_SNGL_Pin GPIO_PIN_6
#define RVRS_DRIVE_SNGL_GPIO_Port GPIOC
#define FRWD_SWITCH_Pin GPIO_PIN_7
#define FRWD_SWITCH_GPIO_Port GPIOC
#define RVRS_SWITCH_Pin GPIO_PIN_8
#define RVRS_SWITCH_GPIO_Port GPIOC
#define BREAKLIGHT_PWM_Pin GPIO_PIN_8
#define BREAKLIGHT_PWM_GPIO_Port GPIOA
#define MOTOR_PUMP_PWM_Pin GPIO_PIN_9
#define MOTOR_PUMP_PWM_GPIO_Port GPIOA
#define MOTOR_FAN_PWM_Pin GPIO_PIN_10
#define MOTOR_FAN_PWM_GPIO_Port GPIOA
#define BATT_FAN_PWM_Pin GPIO_PIN_11
#define BATT_FAN_PWM_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
