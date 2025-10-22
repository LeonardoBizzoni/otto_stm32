#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "main.h"

// TODO(lb): reorder after C++ removal to avoid padding
typedef struct MotorController {
  GPIO_TypeDef *sleep_gpio_port;
  uint16_t sleep_pin;
  GPIO_TypeDef *dir_gpio_port;
  uint16_t dir_pin;
  TIM_HandleTypeDef *pwm_timer;
  uint32_t pwm_channel;
  int32_t max_dutycycle;
} MotorController;

typedef uint8_t MotorDirection;
enum {
  MotorDirection_Backward = 0,
  MotorDirection_Forward,
};

void motorcontroller_init(MotorController *motors);
void motorcontroller_speed_set(MotorController *motor, int32_t duty_cycle);
void motorcontroller_brake(MotorController *motor);
void motorcontroller_coast(MotorController * motor);

#endif
