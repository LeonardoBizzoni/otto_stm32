#include "control/encoder.h"
#include "control/odometry.h"
#include "control/motor_controller.h"
#include "control/pid.h"

// NOTE(lb): this assumes `motors` initialized and with 2 elements
void motorcontroller_init(MotorController *motors) {
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

// NOTE(lb): this assumes `encoders` initialized and with 2 elements
void encoder_init(Encoder *encoders) {
  HAL_TIM_Encoder_Start(encoders[0].timer, TIM_CHANNEL_ALL);
  encoder_count_reset(&encoders[0]);
  encoders[0].previous_millis = 0;
  encoders[0].current_millis = HAL_GetTick();

  HAL_TIM_Encoder_Start(encoders[1].timer, TIM_CHANNEL_ALL);
  encoder_count_reset(&encoders[1]);
  encoders[1].previous_millis = 0;
  encoders[1].current_millis = HAL_GetTick();
}

void encoder_count_reset(Encoder *encoder) {
  //set counter to half its maximum value
  __HAL_TIM_SET_COUNTER(encoder->timer, (encoder->timer->Init.Period / 2));
}

int encoder_count_get(Encoder *encoder) {
  int count = (int)__HAL_TIM_GET_COUNTER(encoder->timer) -
              (encoder->timer->Init.Period / 2);
  return count;
}

void encoder_update(Encoder *encoder) {
  encoder->previous_millis = encoder->current_millis;
  encoder->current_millis = HAL_GetTick();
  encoder->ticks = encoder_count_get(encoder);
  encoder_count_reset(encoder);
}

float encoder_linear_velocity(Encoder *encoder) {
  float meters = meters_from_ticks(encoder->ticks,
                                   encoder->wheel_circumference,
                                   encoder->ticks_per_revolution);
  float deltatime = encoder->current_millis - encoder->previous_millis;
  float linear_velocity = deltatime ? (meters / (deltatime / 1000.f)) : 0.f;
  return linear_velocity;
}

void odometry_setpoint_from_cmdvel(Odometry *odom, float linear_vel,
                                   float angular_vel) {
  odom->setpoint.left = linear_vel - (odom->baseline * angular_vel) / 2;
  odom->setpoint.right = linear_vel + (odom->baseline * angular_vel) / 2;
}

int32_t pid_update(Pid *pid, float measure) {
  pid->error = pid->setpoint - measure;

  //proportional term
  float output = pid->error * pid->kp;

  //integral term without windup
  pid->error_sum += pid->error;
  output += pid->error_sum * pid->ki;

  //derivative term
  output += (pid->error - pid->previous_error) * pid->kd;
  pid->previous_error = pid->error;

  //anti windup
  pid->error_sum -= pid->error;
  int32_t integer_output = CLAMP(((int32_t)output), pid->min, pid->max);
  return integer_output;
}


float meters_from_ticks(float encoder_ticks,
                        float wheel_circumference,
                        float ticks_per_revolution) {
  float meters = (encoder_ticks * wheel_circumference) / ticks_per_revolution;
  return meters;
}
