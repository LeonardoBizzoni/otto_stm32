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
#include "crc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h> // memcpy

#include "control/encoder.h"
#include "control/odometry.h"
#include "control/motor_controller.h"
#include "control/pid.h"

#include "communication/otto_messages.h"

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

/* USER CODE BEGIN PV */

static union {
  Encoder values[2];
  struct {
    Encoder right;
    Encoder left;
  };
} encoders = {
  .right = {
    .timer = &htim5,
  },
  .left = {
    .timer = &htim2,
  },
};

Odometry odom = {0};

Pid pid_left  = {0};
Pid pid_right = {0};
Pid pid_cross = {0};

int32_t pid_max = 0;
int32_t pid_min = 0;

static union {
  MotorController values[2];
  struct {
    MotorController right;
    MotorController left;
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

volatile int32_t left_ticks;
volatile int32_t right_ticks;
volatile float previous_tx_millis;
volatile uint8_t tx_done_flag = 1;
volatile MessageStatusCode otto_status = MessageStatusCode_Waiting4Config;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();
  MX_CRC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  // wait for config or run
  OttoMessage msg = {0};
  // TODO(lb): how do i send stuff to USART6?
  HAL_StatusTypeDef uart_packet_status =
    HAL_UART_Receive(&huart6, (uint8_t*)&msg, sizeof msg.header, 60 * 1000);
  otto_report_unless(uart_packet_status == HAL_OK,
                     OttoMessageError_UART_ReceiveTimeoutElapsed);
  otto_report_unless(msg.header.type < OttoMessageType_COUNT,
                     OttoMessageError_Command_NotRecognized);
  otto_report_unless(msg.header.type == OttoMessageType_Run ||
                     msg.header.type == OttoMessageType_Config,
                     OttoMessageError_Command_NotAvailable);

  uint32_t crc_computed;
  uint32_t *msg_body = (uint32_t*)((uint8_t*)&msg + sizeof msg.header.crc);
  switch (msg.header.type) {
  case OttoMessageType_Run: {
    crc_computed = HAL_CRC_Calculate(&hcrc, msg_body, sizeof msg.header.type);
  } break;
  case OttoMessageType_Config: {
    // NOTE(lb): i don't know if the rest of the message is buffered
    //           and so i can immediately read it in or if this entire
    //           idea is trash.
    HAL_StatusTypeDef config_uart_packet_status =
      HAL_UART_Receive(&huart6, ((uint8_t*)&msg + sizeof msg.header),
                       sizeof msg.config, 60 * 1000);
    otto_report_unless(config_uart_packet_status == HAL_OK,
                       OttoMessageError_UART_ReceiveTimeoutElapsed);
    crc_computed = HAL_CRC_Calculate(&hcrc, msg_body,
                                     sizeof msg.config + sizeof msg.header.type);
  } break;
  }
  otto_report_unless(crc_computed == msg.header.crc, OttoMessageError_UART_Crc);

#if 0
  // ======================================================================
  // NOTE(lb): all of this should be transformed in compile time constants
  odom.baseline = config_msg.baseline;
  encoders.left.wheel_circumference = config_msg.left_wheel_circumference;
  encoders.left.ticks_per_revolution = config_msg.ticks_per_revolution;
  encoders.right.wheel_circumference = config_msg.right_wheel_circumference;
  encoders.right.ticks_per_revolution = config_msg.ticks_per_revolution;

  // NOTE(lb): maybe even this but i'm not sure. And at this point
  //           i'm not even sure that there is a need for a config message.
  memcpy(&pid_left.ks,  &config_msg.pid_ks_left,  sizeof pid_left.ks);
  memcpy(&pid_right.ks, &config_msg.pid_ks_right, sizeof pid_right.ks);
  memcpy(&pid_cross.ks, &config_msg.pid_ks_cross, sizeof pid_cross.ks);
  // ======================================================================
#endif

  encoder_init(encoders.values);
  motorcontroller_init(motors.values);

  //right and left motors have the same parameters
  pid_max = (int32_t)htim4.Instance->ARR;
  pid_min = -pid_max;

  motorcontroller_brake(&motors.left);
  motorcontroller_brake(&motors.right);

  //Enables TIM6 interrupt (used for PID control)
  HAL_TIM_Base_Start_IT(&htim6);

#if 0
  //Enables UART RX interrupt
  HAL_UART_Receive_DMA(&huart6, (uint8_t*) &vel_msg, 12);
#endif

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
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 2);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USART6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

/* USER CODE BEGIN 4 */
void otto_report_handler(OttoMessageError error_code) {
  // TODO(lb): stop motors, send error message via UART,
  //           go back to some sort of "initialization" state
  for (;;) {}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  //TIMER 100Hz PID control

  //accumulate ticks for transmission
  left_ticks += encoder_count_get(&encoders.left);
  right_ticks += encoder_count_get(&encoders.right);

  //PID control
  encoder_update(&encoders.left);
  float left_velocity = encoder_linear_velocity(&encoders.left);
  int left_dutycycle = pid_update(&pid_left, left_velocity);

  encoder_update(&encoders.right);
  float right_velocity = encoder_linear_velocity(&encoders.right);
  int right_dutycycle = pid_update(&pid_right, right_velocity);

  float difference = left_velocity - right_velocity;
  int cross_dutycycle = pid_update(&pid_cross, difference);

  left_dutycycle += cross_dutycycle;
  right_dutycycle -= cross_dutycycle;

  motorcontroller_speed_set(&motors.left, left_dutycycle);
  motorcontroller_speed_set(&motors.right, right_dutycycle);
}

#if 0
uint8_t porcoddio = 0; // NOTE(lb): LOL
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
  /*
   * Manage received message
   */

  uint32_t crc_rx = HAL_CRC_Calculate(&hcrc, (uint32_t*) &vel_msg, 8);

  float linear_velocity;
  float angular_velocity;

  if (crc_rx == vel_msg.crc) {
    linear_velocity = vel_msg.linear_velocity;
    angular_velocity = vel_msg.angular_velocity;
    otto_status = MessageStatusCode_Running;
  } else {
    linear_velocity = 0;
    angular_velocity = 0;
    otto_status = MessageStatusCode_Error_Velocity;
  }

  odometry_setpoint_from_cmdvel(&odom, linear_velocity, angular_velocity);
  float left_setpoint = odom.velocity.left;
  float right_setpoint = odom.velocity.right;

  pid_left.setpoint = left_setpoint;
  pid_right.setpoint = right_setpoint;

  float cross_setpoint = left_setpoint - right_setpoint;
  pid_cross.setpoint = cross_setpoint;

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

  HAL_UART_Receive_DMA(&huart6, (uint8_t*) &vel_msg, sizeof(vel_msg));
}
#endif


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
  tx_done_flag = 1;
}

uint8_t uart_err = 0;
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
  uart_err += 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  //Blue user button on the NUCLEO board
  switch (GPIO_Pin) {
  case user_button_Pin: {
    //TODO ci può servire il bottone blu?
  } break;
  case fault1_Pin:
  case fault2_Pin: {
    motorcontroller_brake(&motors.left);
    motorcontroller_brake(&motors.right);
    //stop TIM6 interrupt (used for PID control)
    HAL_TIM_Base_Stop_IT(&htim6);
    otto_status = MessageStatusCode_Fault_HBridge;
  } break;
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameduty_cycleters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
