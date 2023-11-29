/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
unsigned int analog_value;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;

WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN2_Init(void);
static void MX_WWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  const int APPS_1_in = GPIO_PIN_0; // accelerator pedel 1
  const int APPS_1_port = GPIOA;
  const int APPS_2_in = GPIO_PIN_1; // accelerator pedel 2
  const int APPS_2_port = GPIOA;
  const int BPSF_in = GPIO_PIN_2; // break pressure signal front
  const int BPSF_port = GPIOA;
  const int BPSB_in = GPIO_PIN_3; // break pressure signal back
  const int BPSN_port = GPIOA;
  //const int TQS_in = ; // torque system in (for faults)
  const int FWRD_in = GPIO_PIN_7; // forward drive trigger from dash
  const int FWRD_port = GPIOC;
  const int RVRS_in = GPIO_PIN_8; // reverse drive trigger from dash
  const int RVRS_port = GPIOC;
  const int RTDButton_in = GPIO_PIN_3; // ready to drive button from dash
  const int RTDButton_port = GPIOC;

  // define outputs
  const int FWRD_out = GPIO_PIN_5; // forward motor signal out
  const int FWRD_out_port = GPIOC;
  const int RVRS_out = GPIO_PIN_6; // reverse motor signal out
  const int RVS_out_port = GPIOC;
  //const int RTDMotorControl_out = ; // motor controller ready to drive, initializes motor controller
  //const int ACCEL_out = ; // outputs for torque output
  //const int REGEN_out = ;


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_CAN2_Init();
  MX_WWDG_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	#define MAXTHROTTLEVOLT 4.5
    #define MINTHROTTLEVOLT 0.5
    #define MAXBREAKVOLT 4.5
    #define MINBREAKVOLT 0.5
    #define MINREGENVOLT 0
    #define MAXREGENVOLT 0.49

    HAL_ADC_Start(hadc3);

    if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK) {
    	analog_value = HAL_ADC_GetValue(&hadc3);
    }

    HAL_ADC_Stop(&hadc3);
    HAL_Delay(50);

	// converts analog input to voltage
    float serial_to_volt(int serial, float MAX_VOLT, int ANALOG_IN) {
      float volt = serial * MAX_VOLT/ANALOG_IN;
      return volt;
    }

    float convert_range(float old_value, float old_max, float old_min, float new_max, float new_min) {
      return (((old_value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min);
    }

    // check if a value is within a range
    int in_range(float VALUE, float MAX_VALUE, float MIN_VALUE) {
      if ((VALUE > MIN_VALUE) && (VALUE < MAX_VALUE)) {
        return 1;
      } else {
        return 0;
      }
    }

    // ready to drive sound function
    void readyToDriveSound(const int pin_port, const int pin_out) {
      // turn on and off noise for 1 second (1000 ms)
      HAL_GPIO_WritePin(pin_port, pin_out, 1);
      HAL_Delay(1000);
      HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState)
      HAL_GPIO_WritePin(pin_port, pin_out, 0);
      HAL_Delay(1000);
    }



    // gets analog reading from inputs
    int APPS_1_Serial_in = HAL_GPIO_ReadPin(ADC3_IN0, APPS_1_in);
    int APPS_2_Serial_in = HAL_GPIO_ReadPin(ADC3_IN1, APPS_2_in);
    int BPSF_Serial_in = HAL_GPIO_ReadPin(ADC3_IN2, BPSF_in);
    int BPSB_Serial_in = HAL_GPIO_ReadPin(ADC3_IN3, BPSB_in);

    // gets digital reading from inputs
    // int TQS_Digital_in = HAL_GPIO_ReadPin(, TQS_in);
    int FWRD_Digital_in = HAL_GPIO_ReadPin(FWRD_port, FWRD_in);
    int RVRS_Digital_in = HAL_GPIO_ReadPin(RVRS_port, RVRS_in);
    int RTDButton_Digital_in = HAL_GPIO_ReadPin(RTDButton_port, RTDButton_in);

    // convert serial input to volts scale
    float APPS_1_VOLTS_in = serial_to_volt(APPS_1_Serial_in, 5.0, 1023.0);
    float APPS_2_VOLTS_in = serial_to_volt(APPS_2_Serial_in, 5.0, 1023.0);
    float BPSF_VOLTS_in = serial_to_volt(BPSF_Serial_in, 5.0, 1023.0);
    float BPSB_VOLTS_in = serial_to_volt(BPSB_Serial_in, 5.0, 1023.0);


    // start up mode

    // check break is applied within operating range
    if (in_range(BPSF_VOLTS_in, MAXBREAKVOLT, MINBREAKVOLT) == 1 && in_range(BPSB_VOLTS_in, MAXBREAKVOLT, MINBREAKVOLT) == 1) {
    	// check ready to drive button is pushed
    	if (RTDButton_Digital_in == HIGH) {
    		// TRIGGER READY TO DRIVE MODE
    		HAL_GPIO_WritePin(RTDMotorControl_port, RTDMotorControl_out, 1); // initialize motor controller
    	}
    }

    // ready to drive mode

    // Torque system fault check (not aloud to drive when there is a fault)
    if ((TQS_Digital_in == 1)) {

    	// if (either?) break is applied torque production must stop: APPS/Break Pedal Probability check
    	bool breaksEngaged = false;
    	if (in_range(BPSF_VOLTS_in, MAXBREAKVOLT, MINBREAKVOLT) == 1 || in_range(BPSB_VOLTS_in, MAXBREAKVOLT, MINBREAKVOLT) == 1) {
    		breaksEngaged = true; // this is applied as a constraint while trying to supply power to the motors
    	}

    	// check that the throttle signal maintain 5% offset from each other: APPS implausibility check
    	float throttle_tolterance = MAXTHROTTLEVOLT*0.05; // value which is 5% of max powering voltage
    	bool throttlesMatch = false;
    	if ((APPS_1_VOLTS_in <= (APPS_2_VOLTS_in + throttle_tolterance)) && (APPS_1_VOLTS_in >= (APPS_2_VOLTS_in - throttle_tolterance))) {
    		throttlesMatch = true;
    	} else {
    		throttlesMatch = false;
    	}

    	// check current throttle signals are within operating range
    	if (in_range(APPS_2_VOLTS_in, MAXTHROTTLEVOLT, MINTHROTTLEVOLT)== 1 && in_range(APPS_1_VOLTS_in, MAXTHROTTLEVOLT, MINTHROTTLEVOLT) == 1) { // throttle signals within range to drive motors (might be able to change to checking just one if they are inside throttle tollerance)
    		// check if forward or reverse is active, that throttles match, and if breaks are engaged
    		if ((FWRD_Digital_in == 1)) {
    			HAL_GPIO_WritePin(FWRD_out_port, FWRD_out, 1); // tell motor controller to drive forward
    		} else if ((RVRS_Digital_in == 1)) {
    			HAL_GPIO_WritePin(RVRS_out_port, RVRS_out, 1); // tell motor controller to drive in reverse
    		}
    		if (throttlesMatch == true && breaksEngaged == false) {
    			HAL_GPIO_WritePin(ACCEL_out_port, ACCEL_out, 1); // signal motors to supply torque
    		}
    	} else if (in_range(APPS_2_VOLTS_in, MAXREGENVOLT, MINREGENVOLT) && in_range(APPS_1_VOLTS_in, MAXREGENVOLT, MINREGENVOLT)) {
    		HAL_GPIO_WritePin(REGEN_out_port, REGEN_out, 1); // signal motors to supply regen torque output? not sure about this one
    	}
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG_Init(void)
{

  /* USER CODE BEGIN WWDG_Init 0 */

  /* USER CODE END WWDG_Init 0 */

  /* USER CODE BEGIN WWDG_Init 1 */

  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
  hwwdg.Init.Window = 64;
  hwwdg.Init.Counter = 64;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG_Init 2 */

  /* USER CODE END WWDG_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RTDS_LED_Pin|FRWD_DRIVE_SNGL_Pin|RVRS_DRIVE_SNGL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VCU_FAULT_GPIO_Port, VCU_FAULT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 RTDS_Pin
                           FRWD_SWITCH_Pin RVRS_SWITCH_Pin PC9 PC10
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|RTDS_Pin
                          |FRWD_SWITCH_Pin|RVRS_SWITCH_Pin|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RTDS_LED_Pin FRWD_DRIVE_SNGL_Pin RVRS_DRIVE_SNGL_Pin */
  GPIO_InitStruct.Pin = RTDS_LED_Pin|FRWD_DRIVE_SNGL_Pin|RVRS_DRIVE_SNGL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : AAC_FAULT_Pin PAG_FAULT_Pin */
  GPIO_InitStruct.Pin = AAC_FAULT_Pin|PAG_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPA_FAULT_Pin */
  GPIO_InitStruct.Pin = SPA_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPA_FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VCU_FAULT_Pin */
  GPIO_InitStruct.Pin = VCU_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VCU_FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VCU_DEBUG_LED_Pin */
  GPIO_InitStruct.Pin = VCU_DEBUG_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VCU_DEBUG_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
