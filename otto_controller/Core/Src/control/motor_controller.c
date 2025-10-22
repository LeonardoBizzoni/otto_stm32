#include "control/motor_controller.h"

// NOTE(lb): this assumes `motors` initialized and with 2 elements
void motorcontroller_init(MotorController *motors) {
  // TODO(lb): is `assert` a thing?
  // assert(motors);
  HAL_TIM_PWM_Start(motors[0].pwm_timer, motors[0].pwm_channel);
  motors[0].max_dutycycle = motors[0].pwm_timer->Instance->ARR;

  HAL_TIM_PWM_Start(motors[1].pwm_timer, motors[1].pwm_channel);
  motors[1].max_dutycycle = motors[1].pwm_timer->Instance->ARR;
}

// TODO(lb): just pass the direction yourself and work with abs values
void motorcontroller_speed_set(MotorController *motor, int32_t duty_cycle) {
  HAL_GPIO_WritePin(motor->dir_gpio_port, motor->dir_pin,
                    (GPIO_PinState) (duty_cycle >= 0
                                     ? MotorDirection_Forward
                                     : MotorDirection_Backward));
  duty_cycle = CLAMP_TOP(ABS(duty_cycle), motor->max_dutycycle);
  __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, duty_cycle);
  HAL_GPIO_WritePin(motor->sleep_gpio_port, motor->sleep_pin, GPIO_PIN_SET);
}

void motorcontroller_brake(MotorController *motor) {
  HAL_GPIO_WritePin(motor->sleep_gpio_port, motor->sleep_pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, 0);
}

void motorcontroller_coast(MotorController * motor) {
  HAL_GPIO_WritePin(motor->sleep_gpio_port, motor->sleep_pin, GPIO_PIN_RESET);
}
