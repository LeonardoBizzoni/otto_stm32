#include "main.h"
#include "firmware/fmw_inc.h"

// ============================================================
// Motor controller
void fmw_motor_init(FMW_Motor motors[FMW_MOTOR_COUNT]) {
  for (int32_t i = 0; i < FMW_MOTOR_COUNT; ++ i) {
    HAL_TIM_PWM_Start(motors[i].pwm_timer, motors[i].pwm_channel);
    motors[i].max_dutycycle = motors[i].pwm_timer->Instance->ARR;
    fmw_motor_brake(&motors[i]);
  }
}

void fmw_motor_set_speed(FMW_Motor *motor, int32_t duty_cycle) {
  HAL_GPIO_WritePin(motor->dir_gpio_port, motor->dir_pin,
                    (duty_cycle >= 0
                     ? FMW_MotorDirection_Forward
                     : FMW_MotorDirection_Backward));
  duty_cycle = CLAMP_TOP(ABS(duty_cycle), motor->max_dutycycle);
  __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, duty_cycle);
  HAL_GPIO_WritePin(motor->sleep_gpio_port, motor->sleep_pin, GPIO_PIN_SET);
}

inline void fmw_motor_brake(FMW_Motor *motor) {
  HAL_GPIO_WritePin(motor->sleep_gpio_port, motor->sleep_pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, 0);
}

inline void fmw_motor_coast(FMW_Motor *motor) {
  HAL_GPIO_WritePin(motor->sleep_gpio_port, motor->sleep_pin, GPIO_PIN_RESET);
}

// ============================================================
// Encoder
void fmw_encoder_init(FMW_Encoder encoders[FMW_ENCODER_COUNT]) {
  for (int32_t i = 0; i < FMW_ENCODER_COUNT; ++i) {
    HAL_TIM_Encoder_Start(encoders[i].timer, TIM_CHANNEL_ALL);
    fmw_encoder_count_reset(&encoders[i]);
    encoders[i].current_millis = HAL_GetTick();
    encoders[i].previous_millis = 0;
  }
}

void fmw_encoder_update(FMW_Encoder *encoder) {
  encoder->previous_millis = encoder->current_millis;
  encoder->current_millis = HAL_GetTick();
  encoder->ticks = fmw_encoder_count_get(encoder);
  fmw_encoder_count_reset(encoder);
}

float fmw_encoder_get_linear_velocity(const FMW_Encoder *encoder) {
  float meters = METERS_FROM_TICKS(encoder->ticks,
                                   encoder->wheel_circumference,
                                   encoder->ticks_per_revolution);
  float deltatime = encoder->current_millis - encoder->previous_millis;
  float linear_velocity = deltatime ? (meters / (deltatime / 1000.f)) : 0.f;
  return linear_velocity;
}

inline void fmw_encoder_count_reset(FMW_Encoder *encoder) {
  __HAL_TIM_SET_COUNTER(encoder->timer, (encoder->timer->Init.Period / 2));
}

inline int32_t fmw_encoder_count_get(const FMW_Encoder *encoder) {
  return (int32_t)__HAL_TIM_GET_COUNTER(encoder->timer) - (encoder->timer->Init.Period / 2);
}

// ============================================================
// Odometry
void odometry_setpoint_from_cmdvel(FMW_Odometry *odom, float linear_vel, float angular_vel) {
  odom->setpoint_left = linear_vel - (odom->baseline * angular_vel) / 2;
  odom->setpoint_right = linear_vel + (odom->baseline * angular_vel) / 2;
}

// ============================================================
// PID
int32_t fmw_pid_update(FMW_PidController *pid, float measure) {
  pid->error = pid->setpoint - measure;

  float output = pid->error * pid->ks.proportional;

  //integral term without windup
  pid->error_sum += pid->error;
  output += pid->error_sum * pid->ks.integral;

  output += (pid->error - pid->previous_error) * pid->ks.derivative;
  pid->previous_error = pid->error;

  // anti windup
  pid->error_sum -= pid->error;
  int32_t integer_output = CLAMP(((int32_t)output), pid_min, pid_max);
  return integer_output;
}
