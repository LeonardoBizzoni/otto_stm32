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

#include "p3dx/p3dx_inc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void p3dx_report_handler(P3DX_Error error_code, const char *filename, int32_t filename_length, int32_t line);
void p3dx_message_handle(P3DX_State state, UART_HandleTypeDef *huart, int32_t wait_ms);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static union {
  P3DX_Encoder values[2];
  struct {
    P3DX_Encoder right;
    P3DX_Encoder left;
  };
} encoders = {
  .right = {
    .timer = &htim5,
  },
  .left = {
    .timer = &htim2,
  },
};

P3DX_Odometry odometry = {0};

P3DX_Pid pid_left  = {0};
P3DX_Pid pid_right = {0};
P3DX_Pid pid_cross = {0};

int32_t pid_max = 0;
int32_t pid_min = 0;

static union {
  P3DX_Motor values[2];
  struct {
    P3DX_Motor right;
    P3DX_Motor left;
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
/* volatile MessageStatusCode otto_status = MessageStatusCode_Waiting4Config; */
volatile P3DX_State p3dx_state = P3DX_State_Init;

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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();
  MX_CRC_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  do {
    p3dx_message_handle(P3DX_State_Init, &huart3, 180 * 1000);
  } while(p3dx_state != P3DX_State_Running);

  p3dx_encoder_init(encoders.values);
  p3dx_motor_init(motors.values);

  //right and left motors have the same parameters
  pid_max = (int32_t)htim4.Instance->ARR;
  pid_min = -pid_max;

  p3dx_motor_brake(&motors.left);
  p3dx_motor_brake(&motors.right);

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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART6;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
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
void p3dx_message_handle(P3DX_State state, UART_HandleTypeDef *huart, int32_t wait_ms) {
  P3DX_Message msg = {0};
  HAL_StatusTypeDef uart_packet_status = HAL_UART_Receive(huart, (uint8_t*)&msg, sizeof msg.header, wait_ms);
  p3dx_report_unless(uart_packet_status == HAL_OK, P3DX_Error_UART_ReceiveTimeoutElapsed);
  p3dx_report_unless(msg.header.type < P3DX_MessageType_COUNT, P3DX_Error_Command_NotRecognized);

  uint32_t crc_received = msg.header.crc;
  msg.header.crc = 0;
  switch (state) {
  case P3DX_State_Init: {
    switch (msg.header.type) {
    case P3DX_MessageType_Run: {
      uint32_t crc_computed = HAL_CRC_Calculate(&hcrc, (uint32_t*)&msg, sizeof msg.header);
      p3dx_report_unless(crc_computed == crc_received, P3DX_Error_UART_Crc);
      p3dx_state = P3DX_State_Running;
    } break;
    case P3DX_MessageType_Config: {
      uart_packet_status = HAL_UART_Receive(huart, ((uint8_t*)&msg + sizeof msg.header), sizeof msg.config, 1);
      p3dx_report_unless(uart_packet_status == HAL_OK, P3DX_Error_UART_ReceiveTimeoutElapsed);
      uint32_t crc_computed = HAL_CRC_Calculate(&hcrc, (uint32_t*)&msg, sizeof msg.header.type + sizeof msg.config);
      p3dx_report_unless(crc_computed == crc_received, P3DX_Error_UART_Crc);

      odometry.baseline = msg.config.baseline;
      encoders.left.wheel_circumference = msg.config.left_wheel_circumference;
      encoders.left.ticks_per_revolution = msg.config.ticks_per_revolution;
      encoders.right.wheel_circumference = msg.config.right_wheel_circumference;
      encoders.right.ticks_per_revolution = msg.config.ticks_per_revolution;
      memcpy(&pid_left.ks,  &msg.config.pid_ks_left,  sizeof pid_left.ks);
      memcpy(&pid_right.ks, &msg.config.pid_ks_right, sizeof pid_right.ks);
      memcpy(&pid_cross.ks, &msg.config.pid_ks_cross, sizeof pid_cross.ks);
    } break;
    }
  } break;
  }
}

void p3dx_report_handler(P3DX_Error error_code, const char *filename, int32_t filename_length, int32_t line) {
  // TODO(lb): send error message via UART,
  //           go back to some sort of "initialization" state
  p3dx_motor_brake(&motors.left);
  p3dx_motor_brake(&motors.right);
  p3dx_state = P3DX_State_Error;
  for (;;) {}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  //TIMER 100Hz PID control

  //accumulate ticks for transmission
  left_ticks += p3dx_encoder_count_get(&encoders.left);
  right_ticks += p3dx_encoder_count_get(&encoders.right);

  //PID control
  p3dx_encoder_update(&encoders.left);
  float left_velocity = p3dx_encoder_get_linear_velocity(&encoders.left);
  int left_dutycycle = p3dx_pid_update(&pid_left, left_velocity);

  p3dx_encoder_update(&encoders.right);
  float right_velocity = p3dx_encoder_get_linear_velocity(&encoders.right);
  int right_dutycycle = p3dx_pid_update(&pid_right, right_velocity);

  float difference = left_velocity - right_velocity;
  int cross_dutycycle = p3dx_pid_update(&pid_cross, difference);

  left_dutycycle += cross_dutycycle;
  right_dutycycle -= cross_dutycycle;

  p3dx_motor_set_speed(&motors.left, left_dutycycle);
  p3dx_motor_set_speed(&motors.right, right_dutycycle);
}

#if 0
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
    p3dx_motor_brake(&motors.left);
    p3dx_motor_brake(&motors.right);
    //stop TIM6 interrupt (used for PID control)
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
