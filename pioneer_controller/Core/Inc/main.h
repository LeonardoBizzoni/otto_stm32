/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
  false = 0,
  true = 1,
} bool;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern int32_t pid_max;
extern int32_t pid_min;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ABS(A) ((A) >= 0 ? (A) : (-(A)))
#define MAX(A, B) ((A) >= (B) ? (A) : (B))
#define MIN(A, B) ((A) <= (B) ? (A) : (B))
#define CLAMP_TOP(A, B) MIN((A), (B))
#define CLAMP_BOT(A, B) MAX((A), (B))
#define CLAMP(V, Min, Max) CLAMP_BOT(CLAMP_TOP(V, Max), Min)

#define ARRLENGTH(Arr) (sizeof((Arr)) / sizeof(*(Arr)))

void start(void) __attribute__((noreturn));

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define USER_Btn_EXTI_IRQn EXTI15_10_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define SLED_Pin GPIO_PIN_0
#define SLED_GPIO_Port GPIOC
#define REA_Pin GPIO_PIN_0
#define REA_GPIO_Port GPIOA
#define REB_Pin GPIO_PIN_1
#define REB_GPIO_Port GPIOA
#define ADC_VBATT_Pin GPIO_PIN_2
#define ADC_VBATT_GPIO_Port GPIOA
#define aux_Pin GPIO_PIN_3
#define aux_GPIO_Port GPIOA
#define aux_EXTI_IRQn EXTI3_IRQn
#define LEA_Pin GPIO_PIN_6
#define LEA_GPIO_Port GPIOA
#define LEB_Pin GPIO_PIN_7
#define LEB_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define LDIR_Pin GPIO_PIN_13
#define LDIR_GPIO_Port GPIOF
#define RDIR_Pin GPIO_PIN_14
#define RDIR_GPIO_Port GPIOF
#define PLED_Pin GPIO_PIN_9
#define PLED_GPIO_Port GPIOE
#define buzzer_Pin GPIO_PIN_11
#define buzzer_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define RPWM_Pin GPIO_PIN_14
#define RPWM_GPIO_Port GPIOD
#define LPWM_Pin GPIO_PIN_15
#define LPWM_GPIO_Port GPIOD
#define motors_btn_Pin GPIO_PIN_4
#define motors_btn_GPIO_Port GPIOG
#define motors_btn_EXTI_IRQn EXTI4_IRQn
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define aux2_Pin GPIO_PIN_10
#define aux2_GPIO_Port GPIOG
#define aux2_EXTI_IRQn EXTI15_10_IRQn
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define user_button_Pin GPIO_PIN_13
#define user_button_GPIO_Port GPIOC
#define user_button_EXTI_IRQn EXTI15_10_IRQn
#define current2_Pin GPIO_PIN_0
#define current2_GPIO_Port GPIOC
#define encoder_dx1_Pin GPIO_PIN_0
#define encoder_dx1_GPIO_Port GPIOA
#define encoder_dx2_Pin GPIO_PIN_1
#define encoder_dx2_GPIO_Port GPIOA
#define current1_Pin GPIO_PIN_3
#define current1_GPIO_Port GPIOA
#define encoder_sx1_Pin GPIO_PIN_5
#define encoder_sx1_GPIO_Port GPIOA
#define fault2_Pin GPIO_PIN_6
#define fault2_GPIO_Port GPIOA
#define dir2_Pin GPIO_PIN_12
#define dir2_GPIO_Port GPIOF
#define dir1_Pin GPIO_PIN_13
#define dir1_GPIO_Port GPIOF
#define sleep2_Pin GPIO_PIN_14
#define sleep2_GPIO_Port GPIOF
#define sleep1_Pin GPIO_PIN_15
#define sleep1_GPIO_Port GPIOF
#define fault1_Pin GPIO_PIN_9
#define fault1_GPIO_Port GPIOE
#define pwm2_Pin GPIO_PIN_14
#define pwm2_GPIO_Port GPIOD
#define pwm1_Pin GPIO_PIN_15
#define pwm1_GPIO_Port GPIOD
#define encoder_sx2_Pin GPIO_PIN_3
#define encoder_sx2_GPIO_Port GPIOB

#define FMW_MOTOR_COUNT 2
#define FMW_ENCODER_COUNT 2

#define FLOAT_MAX          3.40282347e+38f
#define FLOAT_MIN_POSITIVE 1.17549435e-38f

#define INT_MAX  2147483647
#define INT_MIN -2147483648

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
