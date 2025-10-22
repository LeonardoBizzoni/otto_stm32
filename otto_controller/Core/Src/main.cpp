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

#include "control/encoder.h"
#include "control/odometry.h"
#include "control/motor_controller.h"
#include "control/pid.h"

#include "communication/otto_messages.h"

// NOTE(lb): couldn't get it to link in the final executable
#include "./control/motor_controller.c"
#include "./control/encoder.c"

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

//Odometry
static Encoder encoders[2] = {{0}, {0}};
Encoder *encoder_right = &encoders[0];
Encoder *encoder_left = &encoders[1];

Odometry odom = {0};

//PID
Pid left_pid;
Pid right_pid;
Pid cross_pid;

//MotorController
static MotorController motors[2] = {
  {
    // Right motor
    .sleep_gpio_port = sleep1_GPIO_Port,
    .sleep_pin = sleep1_Pin,
    .dir_gpio_port = dir1_GPIO_Port,
    .dir_pin = dir1_Pin,
    .pwm_timer = &htim4,
    .pwm_channel = TIM_CHANNEL_4,
  },
  {
    // Left motor
    .sleep_gpio_port = sleep2_GPIO_Port,
    .sleep_pin = sleep2_Pin,
    .dir_gpio_port = dir2_GPIO_Port,
    .dir_pin = dir2_Pin,
    .pwm_timer = &htim4,
    .pwm_channel = TIM_CHANNEL_3,
  },
};
MotorController *motor_right = &motors[0];
MotorController *motor_left = &motors[1];

//Communication
ConfigMessage config_msg;
VelocityMessage vel_msg;
StatusMessage status_msg;

volatile int32_t left_ticks;
volatile int32_t right_ticks;
volatile float previous_tx_millis;
volatile uint8_t tx_done_flag = 1;
volatile uint16_t otto_status = 0;

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

  //wait for config
  HAL_StatusTypeDef config_status = HAL_UART_Receive(&huart6, (uint8_t*) &config_msg, sizeof(config_msg), 60*1000);
  uint32_t config_crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) &config_msg, sizeof(config_msg) - 4);
  if (config_crc != config_msg.crc || config_status != HAL_OK){
    status_msg.status = 2;
    status_msg.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) &status_msg, sizeof(status_msg) - 4);
    while(1){
      HAL_UART_Transmit(&huart6, (uint8_t*) &status_msg, sizeof(status_msg), 1000);
    }
  }

  encoder_left->timer = &htim2;
  encoder_left->wheel_circumference = config_msg.left_wheel_circumference;
  encoder_left->ticks_per_revolution = config_msg.ticks_per_revolution;

  encoder_right->timer = &htim5;
  encoder_right->wheel_circumference = config_msg.right_wheel_circumference;
  encoder_right->ticks_per_revolution = config_msg.ticks_per_revolution;

  odom.baseline = config_msg.baseline;

  encoder_init(encoders);
  motorcontroller_init(motors);

  //right and left motors have the same parameters
  uint32_t max_dutycycle = *(&htim4.Instance->ARR);
  int pid_min = 0;
  int pid_max = 0;
  pid_min = -(int) max_dutycycle;
  pid_max = (int) max_dutycycle;

  left_pid.Config(config_msg.kp_left, config_msg.ki_left, config_msg.kd_left, pid_min, pid_max);
  right_pid.Config(config_msg.kp_right, config_msg.ki_right, config_msg.kd_right, pid_min, pid_max);
  cross_pid.Config(config_msg.kp_cross, config_msg.ki_cross, config_msg.kd_cross, pid_min, pid_max);

  motorcontroller_brake(motor_left);
  motorcontroller_brake(motor_right);

  //Enables TIM6 interrupt (used for PID control)
  HAL_TIM_Base_Start_IT(&htim6);

  //Enables UART RX interrupt
  HAL_UART_Receive_DMA(&huart6, (uint8_t*) &vel_msg, 12);

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  //TIMER 100Hz PID control

  //accumulate ticks for transmission
  left_ticks += encoder_count_get(encoder_left);
  right_ticks += encoder_count_get(encoder_right);

  //PID control
  encoder_update(encoder_left);
  float left_velocity = encoder_linear_velocity(encoder_left);
  int left_dutycycle = left_pid.Update(left_velocity);

  encoder_update(encoder_right);
  float right_velocity = encoder_linear_velocity(encoder_right);
  int right_dutycycle = right_pid.Update(right_velocity);

  float difference = left_velocity - right_velocity;
  int cross_dutycycle = cross_pid.Update(difference);

  left_dutycycle += cross_dutycycle;
  right_dutycycle -= cross_dutycycle;

  motorcontroller_speed_set(motor_left, left_dutycycle);
  motorcontroller_speed_set(motor_right, right_dutycycle);
}

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
    otto_status = 1;
  } else {
    linear_velocity = 0;
    angular_velocity = 0;
    otto_status = 3;
  }

  odometry_setpoint_from_cmdvel(&odom, linear_velocity, angular_velocity);
  float left_setpoint = odom.velocity.left;
  float right_setpoint = odom.velocity.right;

  left_pid.Set(left_setpoint);
  right_pid.Set(right_setpoint);

  float cross_setpoint = left_setpoint - right_setpoint;
  cross_pid.Set(cross_setpoint);

  /*
   * Manage new transmission
   */

  int32_t left_ticks_tx = left_ticks + encoder_count_get(encoder_left);
  int32_t right_ticks_tx = right_ticks + encoder_count_get(encoder_right);

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

  if (tx_done_flag == 1) {
    HAL_UART_Transmit_DMA(&huart6, (uint8_t*) &status_msg, sizeof(status_msg));
    tx_done_flag = 0;
  }

  HAL_UART_Receive_DMA(&huart6, (uint8_t*) &vel_msg, sizeof(vel_msg));
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
  tx_done_flag = 1;
}

uint8_t uart_err = 0;
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
  uart_err += 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  //Blue user button on the NUCLEO board
  if (GPIO_Pin == user_button_Pin) {
    //TODO ci può servire il bottone blu?
  } else if (GPIO_Pin == fault1_Pin) {
    motorcontroller_brake(motor_left);
    motorcontroller_brake(motor_right);
    //stop TIM6 interrupt (used for PID control)
    HAL_TIM_Base_Stop_IT(&htim6);
    otto_status = 4;
  } else if (GPIO_Pin == fault2_Pin) {
    motorcontroller_brake(motor_left);
    motorcontroller_brake(motor_right);
    //stop TIM6 interrupt (used for PID control)
    HAL_TIM_Base_Stop_IT(&htim6);
    otto_status = 4;
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
