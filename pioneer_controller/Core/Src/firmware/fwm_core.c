#include "main.h"
#include "firmware/fmw_inc.h"

// ============================================================
// Motor controller
void fmw_motor_init(FMW_Motor motors[FMW_MOTOR_COUNT]) {
  for (int32_t i = 0; i < FMW_MOTOR_COUNT; ++ i) {
    HAL_TIM_PWM_Start(motors[i].pwm_timer, motors[i].pwm_channel);
    motors[i].max_dutycycle = motors[i].pwm_timer->Instance->ARR;
    motors[i].active = true;
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

void fmw_motor_brake(FMW_Motor *motor) {
  HAL_GPIO_WritePin(motor->sleep_gpio_port, motor->sleep_pin, FMW_MotorDirection_Backward);
  __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, 0);
}

void fmw_motor_enable(FMW_Motor *motor) {
  HAL_TIM_PWM_Start(motor->pwm_timer, motor->pwm_channel);
  motor->active = true;
}

void fmw_motor_disable(FMW_Motor *motor) {
  HAL_TIM_PWM_Stop(motor->pwm_timer, motor->pwm_channel);
  motor->active = false;
}

// ============================================================
// Encoder
void fmw_encoder_init(FMW_Encoder encoders[FMW_ENCODER_COUNT]) {
  for (int32_t i = 0; i < FMW_ENCODER_COUNT; ++i) {
    HAL_TIM_Encoder_Start(encoders[i].timer, TIM_CHANNEL_ALL);
    fmw_encoder_count_reset(&encoders[i]);
    encoders[i].previous_millis = 0;
    encoders[i].current_millis = HAL_GetTick();
  }
}

void fmw_encoder_update(FMW_Encoder *encoder) {
  encoder->previous_millis = encoder->current_millis;
  encoder->current_millis = HAL_GetTick();
  encoder->ticks = fmw_encoder_count_get(encoder);
  fmw_encoder_count_reset(encoder);
}

float fmw_encoder_get_linear_velocity(const FMW_Encoder *encoder) {
  float meters = FMW_METERS_FROM_TICKS(encoder->ticks,
                                       encoder->wheel_circumference,
                                       encoder->ticks_per_revolution);
  float deltatime = encoder->current_millis - encoder->previous_millis;
  float linear_velocity = deltatime > 0.f ? (meters / (deltatime / 1000.f)) : 0.f;
  return linear_velocity;
}

void fmw_encoder_count_reset(FMW_Encoder *encoder) {
  __HAL_TIM_SET_COUNTER(encoder->timer, (encoder->timer->Init.Period / 2));
}

int32_t fmw_encoder_count_get(const FMW_Encoder *encoder) {
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

// ============================================================
// LEDs
void fmw_led_init(FMW_Led *led) {
  HAL_TIM_PWM_Start(led->timer, led->timer_channel);
  __HAL_TIM_SET_COMPARE(led->timer, led->timer_channel, 0);
}

void fmw_led_update(FMW_Led *led) {
  HAL_ADC_Start(led->adc);
  HAL_ADC_PollForConversion(led->adc, HAL_MAX_DELAY);
  uint32_t adc_val = HAL_ADC_GetValue(led->adc);
  float v_adc = ((float)adc_val / FMW_ADC_RESOLUTION) * FMW_V_REF;
  float vin = v_adc * FMW_VIN_SCALE_FACTOR;

  uint32_t duty = 0;
  uint32_t arr = led->timer->Instance->ARR;

  float pled_red_on     = led->voltage_red - led->voltage_hysteresis;
  float pled_red_off    = led->voltage_red + led->voltage_hysteresis;
  float pled_orange_on  = led->voltage_orange - led->voltage_hysteresis;
  float pled_orange_off = led->voltage_orange + led->voltage_hysteresis;

  switch (led->state) {
  case FMW_LedState_Red: {
    if (vin > pled_red_off) {
      led->state = FMW_LedState_Orange;
    }
  } break;
  case FMW_LedState_Orange: {
    if (vin <= pled_red_on) {
      led->state = FMW_LedState_Red;
    } else if (vin >= pled_orange_off) {
      led->state = FMW_LedState_Green;
    }
  } break;
  case FMW_LedState_Green: {
    if (vin < pled_orange_on) {
      led->state = FMW_LedState_Orange;
    }
  } break;
  }

  switch (led->state) {
  case FMW_LedState_Red: {
    duty = 0;
  } break;
  case FMW_LedState_Orange: {
    duty = arr / 2;
  } break;
  case FMW_LedState_Green: {
    duty = arr;
  } break;
  }

  __HAL_TIM_SET_COMPARE(led->timer, led->timer_channel, duty);
}
