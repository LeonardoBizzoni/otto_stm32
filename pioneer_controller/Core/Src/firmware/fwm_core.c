#include "main.h"
#include "firmware/fmw_inc.h"

#include <string.h>

FMW_Result fmw_message_receive_uart(UART_HandleTypeDef *huart, int32_t wait_ms, FMW_Message *msg) {
  assert(wait_ms >= 0);
  memset(msg, 0, sizeof *msg);

  HAL_StatusTypeDef uart_packet_status = HAL_UART_Receive(huart, (uint8_t*)msg, sizeof(*msg), wait_ms);
  if (!(uart_packet_status == HAL_OK)) { return FMW_Result_Error_UART_ReceiveTimeoutElapsed; }
  if (!(msg->header.type > FMW_MessageType_None && msg->header.type < FMW_MessageType_COUNT)) {
    return FMW_Result_Error_Command_NotRecognized;
  }
  return FMW_Result_Ok;
}

int32_t fmw_result_format(char buffer[], size_t buffer_size,
                          const char *filename, int16_t filename_length,
                          int32_t line, FMW_Result result) {
  static const char* const variant_strings[] = {
    #define X(Variant) #Variant,
    FMW_RESULT_VARIANTS(X)
    #undef X
  };
  return snprintf(buffer, buffer_size,
                  "[%.*s:%ld] failed with result: %s\r\n",
                  filename_length, filename, line,
                  variant_strings[result]);
}

void fmw_result_log_uart(UART_HandleTypeDef *huart, FMW_Result result,
                         const char *filename, int16_t filename_length,
                         int32_t line) {
  char buff[512] = {0};
  int32_t length = fmw_result_format(buff, sizeof buff, filename, filename_length, line, result);
  HAL_UART_Transmit(huart, (uint8_t*)buff, length, HAL_MAX_DELAY);
}

// ============================================================
// Motor controller
void fmw_motor_init(FMW_Motor motors[], int32_t count) {
  for (int32_t i = 0; i < count; ++ i) {
    assert(motors[i].sleep_gpio_port);
    assert(motors[i].dir_gpio_port);
    assert(motors[i].pwm_timer);
    assert(motors[i].pwm_channel == TIM_CHANNEL_1 || motors[i].pwm_channel == TIM_CHANNEL_2 ||
           motors[i].pwm_channel == TIM_CHANNEL_3 || motors[i].pwm_channel == TIM_CHANNEL_4 ||
           motors[i].pwm_channel == TIM_CHANNEL_5 || motors[i].pwm_channel == TIM_CHANNEL_6 ||
           motors[i].pwm_channel == TIM_CHANNEL_ALL);
    assert(motors[i].sleep_pin == GPIO_PIN_1 || motors[i].sleep_pin == GPIO_PIN_2 ||
           motors[i].sleep_pin == GPIO_PIN_3 || motors[i].sleep_pin == GPIO_PIN_4 ||
           motors[i].sleep_pin == GPIO_PIN_5 || motors[i].sleep_pin == GPIO_PIN_6 ||
           motors[i].sleep_pin == GPIO_PIN_7 || motors[i].sleep_pin == GPIO_PIN_8 ||
           motors[i].sleep_pin == GPIO_PIN_9 || motors[i].sleep_pin == GPIO_PIN_10 ||
           motors[i].sleep_pin == GPIO_PIN_11 || motors[i].sleep_pin == GPIO_PIN_12 ||
           motors[i].sleep_pin == GPIO_PIN_13 || motors[i].sleep_pin == GPIO_PIN_14 ||
           motors[i].sleep_pin == GPIO_PIN_15 || motors[i].sleep_pin == GPIO_PIN_All);
    assert(motors[i].dir_pin == GPIO_PIN_1 || motors[i].dir_pin == GPIO_PIN_2 ||
           motors[i].dir_pin == GPIO_PIN_3 || motors[i].dir_pin == GPIO_PIN_4 ||
           motors[i].dir_pin == GPIO_PIN_5 || motors[i].dir_pin == GPIO_PIN_6 ||
           motors[i].dir_pin == GPIO_PIN_7 || motors[i].dir_pin == GPIO_PIN_8 ||
           motors[i].dir_pin == GPIO_PIN_9 || motors[i].dir_pin == GPIO_PIN_10 ||
           motors[i].dir_pin == GPIO_PIN_11 || motors[i].dir_pin == GPIO_PIN_12 ||
           motors[i].dir_pin == GPIO_PIN_13 || motors[i].dir_pin == GPIO_PIN_14 ||
           motors[i].dir_pin == GPIO_PIN_15 || motors[i].dir_pin == GPIO_PIN_All);
    assert(motors[i].dir_pin != motors[i].sleep_pin);

    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(motors[i].pwm_timer, motors[i].pwm_channel);
    assert(status == HAL_OK);
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
void fmw_encoder_init(FMW_Encoder encoders[], int32_t count) {
  for (int32_t i = 0; i < count; ++i) {
    assert(encoders[i].timer != NULL);
    assert(encoders[i].ticks_per_revolution > 0);
    assert(encoders[i].wheel_circumference > 0.f);

    encoders[i].previous_millis = 0;
    encoders[i].current_millis = 0;
    encoders[i].ticks = 0;

    HAL_StatusTypeDef status = HAL_TIM_Encoder_Start(encoders[i].timer, TIM_CHANNEL_ALL);
    assert(status == HAL_OK);
    fmw_encoder_count_reset(&encoders[i]);
    encoders[i].current_millis = HAL_GetTick();
  }
}

FMW_Result fmw_encoder_update(FMW_Encoder *encoder) {
  encoder->previous_millis = encoder->current_millis;
  encoder->current_millis = HAL_GetTick();
  encoder->ticks = fmw_encoder_count_get(encoder);
  fmw_encoder_count_reset(encoder);
  if (!(__builtin_expect(encoder->current_millis >= encoder->previous_millis, true))) {
    return FMW_Result_Error_Encoder_GetTick;
  } else {
    return FMW_Result_Ok;
  }
}

float fmw_encoder_get_linear_velocity(const FMW_Encoder *encoder) {
  float deltatime = encoder->current_millis - encoder->previous_millis;
  if (!(__builtin_expect(deltatime > 0.f, true))) {
    return -1.f;
  } else {
    float meters = FMW_METERS_FROM_TICKS(encoder->ticks,
                                         encoder->wheel_circumference,
                                         encoder->ticks_per_revolution);
    assert(meters >= 0.f);
    float linear_velocity = meters / (deltatime / 1000.f);
    assert(linear_velocity >= 0.f);
    return linear_velocity;
  }
}

void fmw_encoder_count_reset(FMW_Encoder *encoder) {
  assert(encoder->timer);
  __HAL_TIM_SET_COUNTER(encoder->timer, (encoder->timer->Init.Period / 2));
}

int32_t fmw_encoder_count_get(const FMW_Encoder *encoder) {
  assert(encoder->timer);
  return (int32_t)__HAL_TIM_GET_COUNTER(encoder->timer) - (encoder->timer->Init.Period / 2);
}

// ============================================================
// Odometry
void fmw_odometry_setpoint_from_velocities(FMW_Odometry *odometry, float linear, float angular) {
  odometry->setpoint_left = linear - (odometry->baseline * angular) / 2;
  odometry->setpoint_right = linear + (odometry->baseline * angular) / 2;
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
  assert(led->timer);
  assert(led->adc);
  assert(led->timer_channel == TIM_CHANNEL_1 || led->timer_channel == TIM_CHANNEL_2 ||
         led->timer_channel == TIM_CHANNEL_3 || led->timer_channel == TIM_CHANNEL_4 ||
         led->timer_channel == TIM_CHANNEL_5 || led->timer_channel == TIM_CHANNEL_6 ||
         led->timer_channel == TIM_CHANNEL_ALL);
  assert(led->voltage_red > 0.f);
  assert(led->voltage_orange > 0.f);
  assert(led->voltage_hysteresis >= 0.f);
  assert(led->state < FMW_LedState_COUNT);

  HAL_StatusTypeDef status = HAL_TIM_PWM_Start(led->timer, led->timer_channel);
  assert(status == HAL_OK);
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
