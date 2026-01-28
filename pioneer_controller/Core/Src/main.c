/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>   // M_PI
#include <string.h> // memcpy

#include "firmware/fmw_inc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
FMW_Result message_handler(FMW_Message *msg)
  __attribute__((warn_unused_result, nonnull));
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MOTOR_COUNT 2
#define ENCODER_COUNT 2
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

#define UART_MESSANGER_HANDLE (&huart3)

// TODO(lb): fill with sensible default
static union {
  FMW_Encoder values[ENCODER_COUNT];
  struct {
    FMW_Encoder right;
    FMW_Encoder left;
  };
} encoders = {
  .right = {
    .timer = &htim3,
    .ticks_per_revolution = 19150,
    .wheel_circumference = (200.f * M_PI) / 1000.f,
  },
  .left = {
    .timer = &htim2,
    .ticks_per_revolution = 19150,
    .wheel_circumference = (200.f * M_PI) / 1000.f,
  },
};

// TODO(lb): fill with sensible default
FMW_Odometry odometry = {
  .baseline = 0.3,
};

// TODO(lb): fill with sensible default
FMW_PidController pid_left = {
  .ks = {
    .proportional = 0.1f,
    .integral     = 0.1f,
    .derivative   = 0.1f,
  },
};

// TODO(lb): fill with sensible default
FMW_PidController pid_right = {
  .ks = {
    .proportional = 0.1f,
    .integral     = 0.1f,
    .derivative   = 0.1f,
  },
};

// TODO(lb): fill with sensible default
FMW_PidController pid_cross = {
  .ks = {
    .proportional = 0.1f,
    .integral     = 0.1f,
    .derivative   = 0.1f,
  },
};

static union {
  FMW_Motor values[MOTOR_COUNT];
  struct {
    FMW_Motor right;
    FMW_Motor left;
  };
} motors = {
  .right = {
    .sleep_gpio_port = sleep1_GPIO_Port,
    .sleep_pin = sleep1_Pin,
    .dir_gpio_port = dir1_GPIO_Port,
    .dir_pin = dir1_Pin,
    .pwm_timer = &htim4,
    .pwm_channel = TIM_CHANNEL_4,
  },
  .left = {
    .sleep_gpio_port = sleep2_GPIO_Port,
    .sleep_pin = sleep2_Pin,
    .dir_gpio_port = dir2_GPIO_Port,
    .dir_pin = dir2_Pin,
    .pwm_timer = &htim4,
    .pwm_channel = TIM_CHANNEL_3,
  },
};

FMW_Led pled = {
  .adc = &hadc1,
  .timer = &htim1,
  .timer_channel = TIM_CHANNEL_1,
  .voltage_red = 11.5f,
  .voltage_orange = 12.5f,
  .voltage_hysteresis = 0.05f,
  .state = FMW_LedState_Red,
};

FMW_Buzzer buzzer = {
  .timer = &htim1,
  .timer_channel = TIM_CHANNEL_2,
};

int32_t pid_max = 0;
int32_t pid_min = 0;

static uint32_t led_update_period = 200;

static volatile int32_t ticks_left  = 0;
static volatile int32_t ticks_right = 0;
static volatile float previous_tx_millis;
static volatile uint8_t tx_done_flag = 1;
/* volatile MessageStatusCode otto_status = MessageStatusCode_Waiting4Config; */

static volatile FMW_Message run_msg = {0};

static volatile uint32_t time_aux_press = 0;
static volatile uint32_t time_aux2_press = 0;
static volatile uint32_t time_last_motors = 0;
static volatile FMW_State fmw_state = FMW_State_Init;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
      Error_Handler();
    }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
      Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
      Error_Handler();
    }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 47999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
      Error_Handler();
    }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3840;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
      Error_Handler();
    }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
      Error_Handler();
    }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
      Error_Handler();
    }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SLED_GPIO_Port, SLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LDIR_Pin|RDIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SLED_Pin */
  GPIO_InitStruct.Pin = SLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : aux_Pin */
  GPIO_InitStruct.Pin = aux_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(aux_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LDIR_Pin RDIR_Pin */
  GPIO_InitStruct.Pin = LDIR_Pin|RDIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : motors_btn_Pin aux2_Pin */
  GPIO_InitStruct.Pin = motors_btn_Pin|aux2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void start(void) {
  for (FMW_Message msg = {0}; fmw_state == FMW_State_Init; ) {
    FMW_Result res = fmw_message_receive_uart(UART_MESSANGER_HANDLE, 180 * 1000, &msg);
    if (res != FMW_Result_Ok) {
      FMW_RESULT_LOG_UART(UART_MESSANGER_HANDLE, res);
    } else {
      res = message_handler(&msg);
      if (res != FMW_Result_Ok) { FMW_RESULT_LOG_UART(UART_MESSANGER_HANDLE, res); }
    }
  }

  fmw_encoder_init(encoders.values, ENCODER_COUNT);
  fmw_motor_init(motors.values, MOTOR_COUNT);
  fmw_led_init(&pled);

  // Right and left motors have the same parameters
  pid_max = (int32_t)htim4.Instance->ARR;
  pid_min = -pid_max;
  assert(pid_max > pid_min);

  // Enables TIM6 interrupt (used for PID control)
  HAL_StatusTypeDef timer_status = HAL_TIM_Base_Start_IT(&htim6);
  assert(timer_status == HAL_OK);

  // Enables UART RX interrupt
  HAL_UART_Receive_DMA(UART_MESSANGER_HANDLE, (uint8_t*)&run_msg, sizeof run_msg);

  for (uint32_t time_last_led_update = 0;;) {
    uint32_t time_now = HAL_GetTick();
    if (time_now - time_last_led_update >= led_update_period) {
      time_last_led_update = time_now;
      fmw_led_update(&pled);
    }
  }
}

FMW_Result message_handler(FMW_Message *msg) {
  // NOTE(lb): the `msg->header.crc != -1` checks are just because i haven't
  //           implemented CRC into the program that sends these messages.
  //           i also don't know if the code to calculate CRC is correct.
  if (msg->header.crc != -1) {
    uint32_t crc_received = msg->header.crc;
    msg->header.crc = 0;
    uint32_t crc_computed = HAL_CRC_Calculate(&hcrc, (uint32_t*)msg, sizeof *msg);
    if (!(crc_computed == crc_received)) { return FMW_Result_Error_UART_Crc; }
  }

  switch (fmw_state) {
  case FMW_State_Init: {
    switch (msg->header.type) {
    case FMW_MessageType_Run: {
      fmw_state = FMW_State_Running;
    } break;
    case FMW_MessageType_Config_Robot: {
      if (!(msg->config_robot.baseline > 0.f)) {
        return FMW_Result_Error_MessageHandler_Init_NonPositiveBaseline;
      }
      if (!(msg->config_robot.wheel_circumference_left > 0.f &&
            msg->config_robot.wheel_circumference_right > 0.f)) {
        return FMW_Result_Error_MessageHandler_Init_NonPositiveWheelCircumference;
      }
      if (!(msg->config_robot.ticks_per_revolution_left > 0 &&
            msg->config_robot.ticks_per_revolution_right > 0)) {
        return FMW_Result_Error_MessageHandler_Init_NonPositiveTicksPerRevolution;
      }

      odometry.baseline                         = msg->config_robot.baseline;
      encoders.left.wheel_circumference         = msg->config_robot.wheel_circumference_left;
      encoders.left.ticks_per_revolution        = msg->config_robot.ticks_per_revolution_left;
      encoders.right.wheel_circumference        = msg->config_robot.wheel_circumference_right;
      encoders.right.ticks_per_revolution       = msg->config_robot.ticks_per_revolution_right;
    } break;
    case FMW_MessageType_Config_PID: {
      memcpy(&pid_left.ks,  &msg->config_pid.left,  sizeof pid_left.ks);
      memcpy(&pid_right.ks, &msg->config_pid.right, sizeof pid_right.ks);
      memcpy(&pid_cross.ks, &msg->config_pid.cross, sizeof pid_cross.ks);
    } break;
    case FMW_MessageType_Config_LED: {
      if (!(msg->config_led.update_period > 0)) {
        return FMW_Result_Error_MessageHandler_Init_NonPositiveLEDUpdatePeriod;
      }

      pled.voltage_red = msg->config_led.voltage_red;
      pled.voltage_orange = msg->config_led.voltage_orange;
      pled.voltage_hysteresis = msg->config_led.voltage_hysteresis;
      led_update_period = msg->config_led.update_period;
    } break;
    case FMW_MessageType_Status: // NOTE(lb): allow status messages in init mode?
    case FMW_MessageType_Velocity: {
      return FMW_Result_Error_Command_NotAvailable;
    } break;
    default: {
      return FMW_Result_Error_Command_NotRecognized;
    } break;
    }
  } break;
  case FMW_State_Running: {
    switch (msg->header.type) {
    case FMW_MessageType_Status: { // TODO(lb): this should be `GetStatus` or something like that.
      int32_t current_ticks_left = ticks_left + fmw_encoder_count_get(&encoders.left);
      int32_t current_ticks_right = ticks_right + fmw_encoder_count_get(&encoders.right);
      (void)current_ticks_left;
      (void)current_ticks_right;
      // TODO(lb): add the rest.
    } break;
    case FMW_MessageType_Velocity: {
      fmw_odometry_setpoint_from_velocities(&odometry, msg->velocity.linear, msg->velocity.angular);
      pid_left.setpoint  = odometry.setpoint_left;
      pid_right.setpoint = odometry.setpoint_right;
      pid_cross.setpoint = odometry.setpoint_left - odometry.setpoint_right;
    } break;
    case FMW_MessageType_Run:
    case FMW_MessageType_Config_Robot:
    case FMW_MessageType_Config_PID:
    case FMW_MessageType_Config_LED: {
      return FMW_Result_Error_Command_NotAvailable;
    } break;
    default: {
      return FMW_Result_Error_Command_NotRecognized;
    } break;
    }
  } break;
  }

  return FMW_Result_Ok;
}

// TIMER 100Hz PID control
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // NOTE(lb): metrics taken for transmission
  ticks_left  += fmw_encoder_count_get(&encoders.left);
  ticks_right += fmw_encoder_count_get(&encoders.right);

  { // PID control
    fmw_encoder_update(&encoders.left);
    float velocity_left = fmw_encoder_get_linear_velocity(&encoders.left);
    int dutycycle_left = fmw_pid_update(&pid_left, velocity_left);

    fmw_encoder_update(&encoders.right);
    float velocity_right = fmw_encoder_get_linear_velocity(&encoders.right);
    int dutycycle_right = fmw_pid_update(&pid_right, velocity_right);
    int dutycycle_cross = fmw_pid_update(&pid_cross, velocity_left - velocity_right);

    dutycycle_left  += dutycycle_cross;
    dutycycle_right -= dutycycle_cross;

    fmw_motor_set_speed(&motors.left, dutycycle_left);
    fmw_motor_set_speed(&motors.right, dutycycle_right);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == UART_MESSANGER_HANDLE) {
    // NOTE(lb): i don't think another interrupt of this kind can occur
    //           while i'm still handling the previous one so casting away
    //           volatile should be fine.
    FMW_Result res = message_handler((FMW_Message*)&run_msg);
    if (res != FMW_Result_Ok) { FMW_RESULT_LOG_UART(huart, res); }

    // NOTE(lb): listen for the next message.
    HAL_UART_Receive_DMA(huart, (uint8_t*)&run_msg, sizeof run_msg);

#if 0
    /*
     * Manage new transmission
     */

    int32_t left_ticks_tx = left_ticks + encoder_count_get(&encoders.left);
    int32_t right_ticks_tx = right_ticks + encoder_count_get(&encoders.right);

    status_msg.left_ticks = left_ticks_tx;
    status_msg.right_ticks = right_ticks_tx;

    left_ticks = 0;
    right_ticks = 0;

    float current_tx_millis = HAL_GetTick();
    status_msg.delta_millis = current_tx_millis - previous_tx_millis;
    previous_tx_millis = current_tx_millis;

    status_msg.status = otto_status;

    uint32_t crc_tx = HAL_CRC_Calculate(&hcrc, (uint32_t*) &status_msg, 12);

    status_msg.crc = crc_tx;

    if (tx_done_flag) {
      HAL_UART_Transmit_DMA(&huart6, (uint8_t*) &status_msg, sizeof(status_msg));
      tx_done_flag = 0;
    }
#endif
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
  tx_done_flag = 1;
}

uint8_t uart_err = 0;
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
  uart_err += 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  uint32_t time_now = HAL_GetTick();

  switch (GPIO_Pin) {
  case aux_Pin: {
    if (time_now - time_aux_press > FMW_DEBOUNCE_DELAY) {
      time_aux_press = time_now;
      HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
      HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
      HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
      /* char msg[] = "AUX1 button pressed\r\n"; */
      /* HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY); */
    }
  } break;
  case aux2_Pin: {
    if (time_now - time_aux2_press > FMW_DEBOUNCE_DELAY) {
      time_aux2_press = time_now;
      /* char msg[] = "AUX2 button pressed\r\n"; */
      /* HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY); */
    }
  } break;
  case motors_btn_Pin: {
    if (time_now - time_last_motors > FMW_DEBOUNCE_DELAY) {
      time_last_motors = time_now;
      /* char msg[] = "Motors button pressed\r\n"; */
      /* HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY); */
      if(motors.left.active && motors.right.active) {
        fmw_motor_disable(&motors.left);
        fmw_motor_disable(&motors.right);
        HAL_GPIO_WritePin(SLED_GPIO_Port, SLED_Pin, GPIO_PIN_RESET);
        fmw_buzzer_set(&buzzer, 1, false);
        /* char msg[] = "Motors OFF\r\n"; */
        /* HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY); */
      } else {
        fmw_motor_enable(&motors.left);
        fmw_motor_enable(&motors.right);
        HAL_GPIO_WritePin(SLED_GPIO_Port, SLED_Pin, GPIO_PIN_SET);
        fmw_buzzer_set(&buzzer, 1, false);
        /* char msg[] = "Motors ON\r\n"; */
        /* HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY); */
      }
    }
  } break;
  case fault1_Pin:
  case fault2_Pin: {
    fmw_motor_brake(&motors.left);
    fmw_motor_brake(&motors.right);
    // stop TIM6 interrupt (used for PID control)
    HAL_TIM_Base_Stop_IT(&htim6);
    /* otto_status = MessageStatusCode_Fault_HBridge; */
  } break;
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     tex: printf("Wrong parameduty_cycleters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
