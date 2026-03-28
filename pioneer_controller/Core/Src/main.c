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
void message_handler(FMW_Message *msg, CRC_HandleTypeDef *hcrc) __attribute__((nonnull));

static __attribute__((always_inline)) inline Vec2Float setpoint_from_velocities(float baseline, float linear, float angular) {
  Vec2Float res = {
    .left = linear - (baseline * angular) / 2.f,
    .right = linear + (baseline * angular) / 2.f,
  };
  return res;
}

void emergency_mode_begin(void);
void emergency_mode_end(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_HANDLE_PTR_USED_FOR_MESSAGE_EXCHANGE_PROTOCOL (&huart3)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

// TODO(lb): fill with sensible default
static union {
  FMW_Encoder values[2];
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
  .ks.fields = {
    .proportional = 0.1f,
    .integral     = 0.1f,
    .derivative   = 0.1f,
  },
};

// TODO(lb): fill with sensible default
FMW_PidController pid_right = {
  .ks.fields = {
    .proportional = 0.1f,
    .integral     = 0.1f,
    .derivative   = 0.1f,
  },
};

// TODO(lb): fill with sensible default
FMW_PidController pid_cross = {
  .ks.fields = {
    .proportional = 0.1f,
    .integral     = 0.1f,
    .derivative   = 0.1f,
  },
};

static union {
  FMW_Motor values[2];
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM7_Init(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_CRC_Init();
  MX_TIM7_Init();
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 10799;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
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
  FMW_InitInfo fmw_info = {
    .motors = motors.values,
    .motors_count = ARRLENGTH(motors.values),
    .encoders = encoders.values,
    .encoders_count = ARRLENGTH(encoders.values),
    .emergency = {
      .timer = &htim7,
      .on_begin = emergency_mode_begin,
      .on_end = emergency_mode_end,
      .wait_at_most_ms_before_emergency = 2000,
    },
    .message_exchange = {
      .huart = UART_HANDLE_PTR_USED_FOR_MESSAGE_EXCHANGE_PROTOCOL,
      .hcrc = &hcrc,
      .handler = message_handler,
    },
  };
  FMW_ASSERT(fmw_init(&fmw_info) == FMW_Result_Ok);

  for (;;) {
    switch (fmw_mode_current()) {
    case FMW_Mode_Config: {
    } break;
    case FMW_Mode_Run: {
      static uint32_t time_last_led_update = 0;
      uint32_t time_now = HAL_GetTick();
      if (time_now - time_last_led_update >= led_update_period) {
        time_last_led_update = time_now;
        FMW_ASSERT(fmw_led_update(&pled) == FMW_Result_Ok);
      }
    } break;
    }
  }
}

Mat3Float mat3float_multiply(Mat3Float lhs, Mat3Float rhs) {
  Mat3Float output = {0};
  output.values[0][0] = (lhs.values[0][0] * rhs.values[0][0]) + (lhs.values[0][1] * rhs.values[1][0]) + (lhs.values[0][2] * rhs.values[2][0]);
  output.values[0][1] = (lhs.values[0][0] * rhs.values[0][1]) + (lhs.values[0][1] * rhs.values[1][1]) + (lhs.values[0][2] * rhs.values[2][1]);
  output.values[0][2] = (lhs.values[0][0] * rhs.values[0][2]) + (lhs.values[0][1] * rhs.values[1][2]) + (lhs.values[0][2] * rhs.values[2][2]);
  output.values[1][0] = (lhs.values[1][0] * rhs.values[0][0]) + (lhs.values[1][1] * rhs.values[1][0]) + (lhs.values[1][2] * rhs.values[2][0]);
  output.values[1][1] = (lhs.values[1][0] * rhs.values[0][1]) + (lhs.values[1][1] * rhs.values[1][1]) + (lhs.values[1][2] * rhs.values[2][1]);
  output.values[1][2] = (lhs.values[1][0] * rhs.values[0][2]) + (lhs.values[1][1] * rhs.values[1][2]) + (lhs.values[1][2] * rhs.values[2][2]);
  output.values[2][0] = (lhs.values[2][0] * rhs.values[0][0]) + (lhs.values[2][1] * rhs.values[1][0]) + (lhs.values[2][2] * rhs.values[2][0]);
  output.values[2][1] = (lhs.values[2][0] * rhs.values[0][1]) + (lhs.values[2][1] * rhs.values[1][1]) + (lhs.values[2][2] * rhs.values[2][1]);
  output.values[2][2] = (lhs.values[2][0] * rhs.values[0][2]) + (lhs.values[2][1] * rhs.values[1][2]) + (lhs.values[2][2] * rhs.values[2][2]);
  return output;
}

void emergency_mode_begin(void) {
  HAL_GPIO_TogglePin(GPIOB, LD1_Pin | LD2_Pin | LD3_Pin);
  fmw_encoder_count_reset(&encoders.left);
  fmw_encoder_count_reset(&encoders.right);
}

void emergency_mode_end(void) {
  HAL_GPIO_TogglePin(GPIOB, LD1_Pin | LD2_Pin | LD3_Pin);
}

void message_handler(FMW_Message *msg, CRC_HandleTypeDef *hcrc) {
  // NOTE(lb): the `msg->header.crc != -1` checks are just because i haven't
  //           implemented CRC into the program that sends these messages.
  //           i also don't know if the code to calculate CRC is correct (probably isn't).

  FMW_Result result = FMW_Result_Ok;
  if (msg->header.crc != -1) {
    uint32_t crc_received = msg->header.crc;
    msg->header.crc = 0;
    uint32_t crc_computed = HAL_CRC_Calculate(hcrc, (uint32_t*)msg, sizeof *msg);
    if (crc_computed != crc_received) {
      result = FMW_Result_Error_UART_Crc;
      goto msg_contains_error;
    }
  }

  switch (fmw_mode_current()) {
  case FMW_Mode_Config: {
    switch (msg->header.type) {
    case FMW_MessageType_ModeChange_Run: {
      result = fmw_encoders_init(); if (result != FMW_Result_Ok) { goto msg_contains_error; }
      result = fmw_motors_init();   if (result != FMW_Result_Ok) { goto msg_contains_error; }
      result = fmw_led_init(&pled); if (result != FMW_Result_Ok) { goto msg_contains_error; }

      // Right and left motors have the same parameters
      pid_max = (int32_t)htim4.Instance->ARR;
      pid_min = -pid_max;
      FMW_ASSERT(pid_max > pid_min);

      // Enables TIM6 interrupt (used for PID control)
      HAL_StatusTypeDef timer_status = HAL_TIM_Base_Start_IT(&htim6);
      FMW_ASSERT(timer_status == HAL_OK);

      FMW_ASSERT(fmw_mode_transition(FMW_Mode_Run) == FMW_Result_Ok);
    } break;
    case FMW_MessageType_Config_Robot: {
      if (!(msg->config_robot.baseline > 0.f)) {
        result = FMW_Result_Error_MessageHandler_Config_NonPositiveBaseline;
        goto msg_contains_error;
      }
      if (!(msg->config_robot.wheel_circumference_left > 0.f &&
            msg->config_robot.wheel_circumference_right > 0.f)) {
        result = FMW_Result_Error_MessageHandler_Config_NonPositiveWheelCircumference;
        goto msg_contains_error;
      }
      if (!(msg->config_robot.ticks_per_revolution_left > 0 &&
            msg->config_robot.ticks_per_revolution_right > 0)) {
        result = FMW_Result_Error_MessageHandler_Config_NonPositiveTicksPerRevolution;
        goto msg_contains_error;
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
        result = FMW_Result_Error_MessageHandler_Config_NonPositiveLEDUpdatePeriod;
        goto msg_contains_error;
      }

      pled.voltage_red = msg->config_led.voltage_red;
      pled.voltage_orange = msg->config_led.voltage_orange;
      pled.voltage_hysteresis = msg->config_led.voltage_hysteresis;
      led_update_period = msg->config_led.update_period;
    } break;
    case FMW_MessageType_Emergency_Begin: {
      fmw_emergency_begin();
    } break;
    case FMW_MessageType_Emergency_End:
    case FMW_MessageType_ModeChange_Config:
    case FMW_MessageType_Run_GetStatus:
    case FMW_MessageType_Run_SetVelocity: {
      result = FMW_Result_Error_Command_NotAvailable;
      goto msg_contains_error;
    } break;
    default: {
      result = FMW_Result_Error_Command_NotRecognized;
      goto msg_contains_error;
    } break;
    }
  } break;
  case FMW_Mode_Run: {
    switch (msg->header.type) {
    case FMW_MessageType_Run_SetVelocity: {
      Vec2Float setpoint = setpoint_from_velocities(odometry.baseline, msg->run_set_velocity.linear, msg->run_set_velocity.angular);
      pid_left.setpoint  = setpoint.left;
      pid_right.setpoint = setpoint.right;
      pid_cross.setpoint = setpoint.left - setpoint.right;
    } // fallthrough
    case FMW_MessageType_Run_GetStatus: {
      // NOTE(lb): SetVelocity continues here as well because the previous case doesn't end with a `break`.
      //           order matters.

      int32_t ticks_measured_left = 0;
      int32_t ticks_measured_right = 0;
      result = fmw_encoder_count_get(&encoders.left, &ticks_measured_left);   if (result != FMW_Result_Ok) { goto msg_contains_error; }
      result = fmw_encoder_count_get(&encoders.right, &ticks_measured_right); if (result != FMW_Result_Ok) { goto msg_contains_error; }

      int32_t current_ticks_left = ticks_left + ticks_measured_left;
      int32_t current_ticks_right = ticks_right + ticks_measured_right;
      ticks_left = ticks_right = 0;

      static float time_millis_previous = 0.f;
      float time_millis_current = HAL_GetTick();
      float time_millis_delta = time_millis_current - time_millis_previous;
      time_millis_previous = time_millis_current;

      FMW_Message msg = {0};
      msg.header.type = FMW_MessageType_Response;
      msg.response.result = FMW_Result_Ok;
      msg.response.delta_millis = time_millis_delta;
      msg.response.ticks_left = current_ticks_left;
      msg.response.ticks_right = current_ticks_right;
      msg.response.position_x = odometry.position.x;
      msg.response.position_y = odometry.position.y;
      msg.response.orientation_x = odometry.orientation.x;
      msg.response.orientation_y = odometry.orientation.y;
      msg.response.velocity_linear = (odometry.velocity_linear.left + odometry.velocity_linear.right) / 2.f;
      msg.response.velocity_angular = odometry.velocity_angular;
      fmw_uart_message_send(&msg);
      return;
      // NOTE(lb): GetStatus&SetVelocity have to respond with the same special message format.
      //           so they don't continue down to `msg_contains_error`.
    } break;
    case FMW_MessageType_ModeChange_Config: {
      fmw_motors_stop();
      result = fmw_encoder_count_reset(&encoders.left);  if (result != FMW_Result_Ok) { goto msg_contains_error; }
      result = fmw_encoder_count_reset(&encoders.right); if (result != FMW_Result_Ok) { goto msg_contains_error; }

      result = fmw_encoders_deinit(); if (result != FMW_Result_Ok) { goto msg_contains_error; }
      result = fmw_motors_deinit();   if (result != FMW_Result_Ok) { goto msg_contains_error; }
      result = fmw_led_deinit(&pled); if (result != FMW_Result_Ok) { goto msg_contains_error; }

      HAL_StatusTypeDef timer_status = HAL_TIM_Base_Stop_IT(&htim6);
      FMW_ASSERT(timer_status == HAL_OK);

      FMW_ASSERT(fmw_mode_transition(FMW_Mode_Config) == FMW_Result_Ok);
    } break;
    case FMW_MessageType_Emergency_Begin: {
      fmw_emergency_begin();
    } break;
    case FMW_MessageType_Emergency_End:
    case FMW_MessageType_ModeChange_Run:
    case FMW_MessageType_Config_Robot:
    case FMW_MessageType_Config_PID:
    case FMW_MessageType_Config_LED: {
      result = FMW_Result_Error_Command_NotAvailable;
      goto msg_contains_error;
    } break;
    default: {
      result = FMW_Result_Error_Command_NotRecognized;
      goto msg_contains_error;
    } break;
    }
  } break;
  case FMW_Mode_Emergency: {
    switch (msg->header.type) {
    case FMW_MessageType_Emergency_End: {
      fmw_emergency_end();
    } break;
    default: {
      result = FMW_Result_Error_Command_NotAvailable;
    } break;
    }
  } break;
  }

  // NOTE(lb): control flow naturally converges here.
  //           the symbol is used to jump here directly in case of error.
 msg_contains_error:;
  FMW_Message msg_response = {0};
  msg_response.header.type = FMW_MessageType_Response;
  msg_response.response.result = result;
  fmw_uart_message_send(&msg_response);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim7) { // TIMER 1Hz no message exchange check
    fmw_emergency_timer_update();
  } else if (htim == &htim6) { // TIMER 100Hz PID control
    fmw_encoders_update();
    ticks_left += encoders.left.ticks;
    ticks_right += encoders.right.ticks;

    float meters_traveled_left = FMW_METERS_FROM_TICKS(encoders.left.ticks, encoders.left.wheel_circumference, encoders.left.ticks_per_revolution);
    float meters_traveled_right = FMW_METERS_FROM_TICKS(encoders.right.ticks, encoders.right.wheel_circumference, encoders.right.ticks_per_revolution);

    fmw_odometry_pose_update(&odometry, meters_traveled_left, meters_traveled_right);

    float velocity_left = 0.f;
    float velocity_right = 0.f;
    fmw_encoder_get_linear_velocity(&encoders.left, meters_traveled_left, &velocity_left);
    fmw_encoder_get_linear_velocity(&encoders.right, meters_traveled_right, &velocity_right);

    odometry.velocity_linear.left = velocity_left;
    odometry.velocity_linear.right = velocity_right;
    odometry.velocity_angular = (velocity_right - velocity_left) / odometry.baseline;

    int32_t dutycycle_left = fmw_pid_update(&pid_left, velocity_left);
    int32_t dutycycle_right = fmw_pid_update(&pid_right, velocity_right);
    int32_t dutycycle_cross = fmw_pid_update(&pid_cross, velocity_left - velocity_right);

    dutycycle_left  += dutycycle_cross;
    dutycycle_right -= dutycycle_cross;

    fmw_motor_set_speed(&motors.left, dutycycle_left);
    fmw_motor_set_speed(&motors.right, dutycycle_right);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == UART_HANDLE_PTR_USED_FOR_MESSAGE_EXCHANGE_PROTOCOL) {
    fmw_uart_message_dispatch();
    return;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == UART_HANDLE_PTR_USED_FOR_MESSAGE_EXCHANGE_PROTOCOL) {
    fmw_uart_error();
    return;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  uint32_t time_now = HAL_GetTick();

  switch (GPIO_Pin) {
  case USER_Btn_Pin: {
    static uint32_t time_btn_press_start = 0;
    static uint32_t time_last_emergency = 0;
    if (time_now - time_last_emergency <= FMW_DEBOUNCE_DELAY) { return; }
    time_last_emergency = time_now;
    GPIO_PinState btn_state = HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin);
    if (btn_state == GPIO_PIN_SET) {
      time_btn_press_start = time_now;
    } else {
      uint32_t btn_held_duration = time_now - time_btn_press_start;
      if (btn_held_duration >= FMW_EMERGENCY_MODE_EXIT_BUTTON_HOLD_DURATION_MS) {
        fmw_emergency_end();
      } else  {
        fmw_emergency_begin();
      }
    }
  } break;
  case motors_btn_Pin: {
    static uint32_t time_last_motors = 0;
    if (time_now - time_last_motors > FMW_DEBOUNCE_DELAY) {
      time_last_motors = time_now;
      if (motors.left.active && motors.right.active) {
        fmw_motors_deinit();
        HAL_GPIO_WritePin(SLED_GPIO_Port, SLED_Pin, GPIO_PIN_RESET);
        fmw_buzzers_set(&buzzer, 1, false);
      } else {
        FMW_ASSERT(!motors.left.active);
        FMW_ASSERT(!motors.right.active);
        fmw_motors_init();
        HAL_GPIO_WritePin(SLED_GPIO_Port, SLED_Pin, GPIO_PIN_SET);
        fmw_buzzers_set(&buzzer, 1, false);
      }
    }
  } break;
  case fault1_Pin:
  case fault2_Pin: {
    fmw_motors_stop();
    FMW_Message response = {0};
    response.header.type = FMW_MessageType_Response;
    response.response.result = FMW_Result_Error_FaultPinTriggered;

    fmw_uart_message_send(&response);

    HAL_TIM_Base_Stop_IT(&htim6);
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
