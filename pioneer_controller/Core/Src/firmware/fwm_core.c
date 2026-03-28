#include "main.h"
#include "firmware/fmw_inc.h"

#include <string.h>
#include <math.h>

static struct {
  void (*callback_message_handler)(FMW_Message *msg, CRC_HandleTypeDef *hcrc);
  void (*callback_emergency_begin)(void);
  void (*callback_emergency_end)(void);

  TIM_HandleTypeDef *emergency_timer;
  uint32_t emergency_mode_grace_period_ms;
  volatile uint32_t emergency_last_message_received_time;

  FMW_Motor *motors;
  FMW_Encoder *encoders;
  int32_t motors_count;
  int32_t encoders_count;
  bool motors_active;
  bool motors_active_before_emergency;

  UART_HandleTypeDef *huart;
  CRC_HandleTypeDef *hcrc;

  volatile FMW_Message uart_buffer;
  volatile FMW_Mode mode_current;
  volatile FMW_Mode mode_previous;
} fmw_state = {
  .emergency_mode_grace_period_ms = 1000,
  .mode_current  = FMW_Mode_Config,
  .mode_previous = FMW_Mode_None,
};

// ============================================================
// Firmware initialization
FMW_Result fmw_init(const FMW_InitInfo *info) {
  if ((info->message_exchange.huart == NULL) || (info->message_exchange.hcrc == NULL) ||
      (info->message_exchange.handler == NULL) || (info->emergency.on_begin == NULL) ||
      (info->emergency.on_end == NULL) ||
      (info->motors_count < 0) || (info->motors_count > 0 && info->motors == NULL) ||
      (info->encoders_count < 0) || (info->encoders_count > 0 && info->encoders == NULL)) {
    return FMW_Result_Error_InvalidArguments;
  }

  for (int32_t i = 0; i < info->motors_count; ++i) {
    if ((info->motors[i].sleep_gpio_port == NULL) || (info->motors[i].dir_gpio_port == NULL) ||
        (info->motors[i].pwm_timer == NULL) || (info->motors[i].dir_pin == info->motors[i].sleep_pin) ||
        (info->motors[i].pwm_channel != TIM_CHANNEL_1 && info->motors[i].pwm_channel != TIM_CHANNEL_2 &&
         info->motors[i].pwm_channel != TIM_CHANNEL_3 && info->motors[i].pwm_channel != TIM_CHANNEL_4 &&
         info->motors[i].pwm_channel != TIM_CHANNEL_5 && info->motors[i].pwm_channel != TIM_CHANNEL_6 &&
         info->motors[i].pwm_channel != TIM_CHANNEL_ALL) ||
        (info->motors[i].sleep_pin != GPIO_PIN_1  && info->motors[i].sleep_pin != GPIO_PIN_2  &&
         info->motors[i].sleep_pin != GPIO_PIN_3  && info->motors[i].sleep_pin != GPIO_PIN_4  &&
         info->motors[i].sleep_pin != GPIO_PIN_5  && info->motors[i].sleep_pin != GPIO_PIN_6  &&
         info->motors[i].sleep_pin != GPIO_PIN_7  && info->motors[i].sleep_pin != GPIO_PIN_8  &&
         info->motors[i].sleep_pin != GPIO_PIN_9  && info->motors[i].sleep_pin != GPIO_PIN_10 &&
         info->motors[i].sleep_pin != GPIO_PIN_11 && info->motors[i].sleep_pin != GPIO_PIN_12 &&
         info->motors[i].sleep_pin != GPIO_PIN_13 && info->motors[i].sleep_pin != GPIO_PIN_14 &&
         info->motors[i].sleep_pin != GPIO_PIN_15 && info->motors[i].sleep_pin != GPIO_PIN_All) ||
        (info->motors[i].dir_pin != GPIO_PIN_1  && info->motors[i].dir_pin != GPIO_PIN_2  &&
         info->motors[i].dir_pin != GPIO_PIN_3  && info->motors[i].dir_pin != GPIO_PIN_4  &&
         info->motors[i].dir_pin != GPIO_PIN_5  && info->motors[i].dir_pin != GPIO_PIN_6  &&
         info->motors[i].dir_pin != GPIO_PIN_7  && info->motors[i].dir_pin != GPIO_PIN_8  &&
         info->motors[i].dir_pin != GPIO_PIN_9  && info->motors[i].dir_pin != GPIO_PIN_10 &&
         info->motors[i].dir_pin != GPIO_PIN_11 && info->motors[i].dir_pin != GPIO_PIN_12 &&
         info->motors[i].dir_pin != GPIO_PIN_13 && info->motors[i].dir_pin != GPIO_PIN_14 &&
         info->motors[i].dir_pin != GPIO_PIN_15 && info->motors[i].dir_pin != GPIO_PIN_All)) {
      return FMW_Result_Error_InvalidArguments;
    }
  }

  for (int32_t i = 0; i < info->encoders_count; ++i) {
    if ((info->encoders[i].timer == NULL) || (info->encoders[i].ticks_per_revolution <= 0) ||
        (info->encoders[i].wheel_circumference <= 0.f)) {
      return FMW_Result_Error_InvalidArguments;
    }
  }

  fmw_state.motors = info->motors;
  fmw_state.motors_count = info->motors_count;
  fmw_state.encoders = info->encoders;
  fmw_state.encoders_count = info->encoders_count;
  fmw_state.huart = info->message_exchange.huart;
  fmw_state.hcrc = info->message_exchange.hcrc;
  fmw_state.emergency_mode_grace_period_ms = info->emergency.wait_at_most_ms_before_emergency;
  fmw_state.callback_message_handler = info->message_exchange.handler;
  fmw_state.callback_emergency_begin = info->emergency.on_begin;
  fmw_state.callback_emergency_end = info->emergency.on_end;
  fmw_state.emergency_timer = info->emergency.timer;

  HAL_StatusTypeDef timer_init_res = HAL_TIM_Base_Start_IT(info->emergency.timer);
  if (timer_init_res != HAL_OK) { return FMW_Result_Error_Hal; }

  HAL_StatusTypeDef dma_init_res = HAL_UART_Receive_DMA(fmw_state.huart, (uint8_t*)&fmw_state.uart_buffer, sizeof fmw_state.uart_buffer);
  if (dma_init_res != HAL_OK) { return FMW_Result_Error_Hal; }

  return FMW_Result_Ok;
}

void fmw_uart_message_dispatch(void) {
  fmw_state.emergency_last_message_received_time = HAL_GetTick();
  fmw_state.callback_message_handler((FMW_Message*)&fmw_state.uart_buffer, fmw_state.hcrc);
  // NOTE(lb): listen for the next message "recursively".
  HAL_UART_Receive_DMA(fmw_state.huart, (uint8_t*)&fmw_state.uart_buffer, sizeof fmw_state.uart_buffer);
}

void fmw_uart_error(void) {
  // NOTE(lb): i don't know how to determine if the error that cause the jump here
  //           was during a receive or a send of a message over UART, so i'm just
  //           going to stop the motors and abort the receive just in case.
  fmw_motors_stop();
  HAL_UART_AbortReceive(fmw_state.huart);

  FMW_Message response = {0};
  response.response.result = fmw_result_from_uart_error();
  fmw_uart_message_send(&response);
  HAL_UART_Receive_DMA(fmw_state.huart, (uint8_t*)&fmw_state.uart_buffer, sizeof fmw_state.uart_buffer);
}

void fmw_emergency_begin(void) {
  if (fmw_state.mode_current == FMW_Mode_Emergency) { return; }
  if (fmw_state.motors_count > 0) {
    fmw_state.motors_active_before_emergency = fmw_state.motors[0].active;
    if (fmw_state.motors[0].active) {
      fmw_motors_stop();
      fmw_motors_deinit();
    }
  }

  fmw_state.mode_previous = fmw_state.mode_current;
  fmw_state.mode_current = FMW_Mode_Emergency;

  if (fmw_state.callback_emergency_begin) { fmw_state.callback_emergency_begin(); }
}

void fmw_emergency_end(void) {
  if (fmw_state.mode_previous == FMW_Mode_None) { return; }
  fmw_state.mode_current = fmw_state.mode_previous;
  fmw_state.mode_previous = FMW_Mode_None;
  if (fmw_state.motors_count > 0 && fmw_state.motors_active_before_emergency) {
    fmw_motors_init();
  }
  if (fmw_state.callback_emergency_end) { fmw_state.callback_emergency_end(); }
}

void fmw_emergency_timer_update(void) {
  if (fmw_state.mode_current != FMW_Mode_Run) { return; }
  uint32_t time_now = HAL_GetTick();
  if (time_now - fmw_state.emergency_last_message_received_time > fmw_state.emergency_mode_grace_period_ms) {
    fmw_emergency_begin();
  }
}

FMW_Mode fmw_mode_current(void) {
  return fmw_state.mode_current;
}

FMW_Result fmw_mode_transition(FMW_Mode mode) {
  if (mode <= FMW_Mode_None || mode >= FMW_Mode_COUNT) { return FMW_Result_Error_InvalidArguments; }
  fmw_state.mode_previous = fmw_state.mode_current;
  fmw_state.mode_current = mode;
  return FMW_Result_Ok;
}

// ============================================================
// Misc
void fmw_uart_message_send(FMW_Message *msg) {
  msg->header.crc = HAL_CRC_Calculate(fmw_state.hcrc, (uint32_t*)msg, sizeof *msg);
  HAL_StatusTypeDef res = HAL_UART_Transmit(fmw_state.huart, (uint8_t*)msg, sizeof *msg, fmw_state.emergency_mode_grace_period_ms);
  if (res != HAL_OK) { fmw_emergency_begin(); }
}

FMW_Result fmw_result_from_uart_error(void) {
  switch (fmw_state.huart->ErrorCode) {
  case HAL_UART_ERROR_PE: {
    return FMW_Result_Error_UART_Parity;
  } break;
  case HAL_UART_ERROR_FE: {
    return FMW_Result_Error_UART_Frame;
  } break;
  case HAL_UART_ERROR_NE: {
    return FMW_Result_Error_UART_Noise;
  } break;
  case HAL_UART_ERROR_ORE: {
    return FMW_Result_Error_UART_Overrun;
  } break;
  case HAL_UART_ERROR_RTO: {
    return FMW_Result_Error_UART_ReceiveTimeoutElapsed;
  } break;
  default: {
    return FMW_Result_Ok;
  } break;
  }
}

// ============================================================
// Motor controller
FMW_Result fmw_motors_init(void) {
  for (int32_t i = 0; i < fmw_state.motors_count; ++i) {
    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(fmw_state.motors[i].pwm_timer, fmw_state.motors[i].pwm_channel);
    if (status != HAL_OK) { return FMW_Result_Error_Hal; }
    fmw_state.motors[i].max_dutycycle = fmw_state.motors[i].pwm_timer->Instance->ARR;
    fmw_state.motors[i].active = true;
  }
  fmw_motors_stop();
  return FMW_Result_Ok;
}

FMW_Result fmw_motors_deinit(void) {
  for (int32_t i = 0; i < fmw_state.motors_count; ++i) {
    HAL_StatusTypeDef status = HAL_TIM_PWM_Stop(fmw_state.motors[i].pwm_timer, fmw_state.motors[i].pwm_channel);
    if (status != HAL_OK) { return FMW_Result_Error_Hal; }
    fmw_state.motors[i].active = false;
  }
  return FMW_Result_Ok;
}

void fmw_motors_stop(void) {
  for (int32_t i = 0; i < fmw_state.motors_count; ++i) {
    HAL_GPIO_WritePin(fmw_state.motors[i].sleep_gpio_port, fmw_state.motors[i].sleep_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(fmw_state.motors[i].pwm_timer, fmw_state.motors[i].pwm_channel, 0);
  }
}

FMW_Result fmw_motor_set_speed(FMW_Motor *motor, int32_t duty_cycle) {
  if (motor < fmw_state.motors || motor >= (fmw_state.motors + fmw_state.motors_count)) {
    return FMW_Result_Error_InvalidArguments;
  }

  HAL_GPIO_WritePin(motor->dir_gpio_port, motor->dir_pin, (duty_cycle >= 0 ? FMW_MotorDirection_Forward : FMW_MotorDirection_Backward));
  duty_cycle = CLAMP_TOP(ABS(duty_cycle), motor->max_dutycycle);
  __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, duty_cycle);
  HAL_GPIO_WritePin(motor->sleep_gpio_port, motor->sleep_pin, GPIO_PIN_SET);

  return FMW_Result_Ok;
}

// ============================================================
// Encoder
FMW_Result fmw_encoders_init(void) {
  for (int32_t i = 0; i < fmw_state.encoders_count; ++i) {
    fmw_state.encoders[i].previous_millis = 0;
    fmw_state.encoders[i].current_millis = 0;
    fmw_state.encoders[i].ticks = 0;

    HAL_StatusTypeDef status = HAL_TIM_Encoder_Start(fmw_state.encoders[i].timer, TIM_CHANNEL_ALL);
    if (status != HAL_OK) { return FMW_Result_Error_Hal; }
    FMW_Result res = fmw_encoder_count_reset(&fmw_state.encoders[i]);
    if (res != FMW_Result_Ok) { return res; }
    fmw_state.encoders[i].current_millis = HAL_GetTick();
  }
  return FMW_Result_Ok;
}

FMW_Result fmw_encoders_deinit(void) {
  for (int32_t i = 0; i < fmw_state.encoders_count; ++i) {
    HAL_StatusTypeDef status = HAL_TIM_Encoder_Stop(fmw_state.encoders[i].timer, TIM_CHANNEL_ALL);
    if (status != HAL_OK) { return FMW_Result_Error_Hal; }
  }
  return FMW_Result_Ok;
}

FMW_Result fmw_encoders_update(void) {
  for (int32_t i = 0; i < fmw_state.encoders_count; ++i) {
    fmw_state.encoders[i].previous_millis = fmw_state.encoders[i].current_millis;
    fmw_state.encoders[i].current_millis = HAL_GetTick();
    FMW_Result res = fmw_encoder_count_get(&fmw_state.encoders[i], &fmw_state.encoders[i].ticks);
    if (res != FMW_Result_Ok) { return res; }
    res = fmw_encoder_count_reset(&fmw_state.encoders[i]);
    if (res != FMW_Result_Ok) { return res; }
  }
  return FMW_Result_Ok;
}

FMW_Result fmw_encoder_get_linear_velocity(const FMW_Encoder *encoder, float meters_traveled, float *linear_velocity) {
  if (encoder < fmw_state.encoders || encoder >= (fmw_state.encoders + fmw_state.encoders_count)) {
    *linear_velocity = 0.f;
    return FMW_Result_Error_InvalidArguments;
  }
  float deltatime = encoder->current_millis - encoder->previous_millis;
  if (deltatime == 0.f) {
    *linear_velocity = 0.f;
  } else {
    *linear_velocity = meters_traveled / (deltatime / 1000.f);
  }
  return FMW_Result_Ok;
}

FMW_Result fmw_encoder_count_reset(FMW_Encoder *encoder) {
  if (encoder < fmw_state.encoders || encoder >= (fmw_state.encoders + fmw_state.encoders_count)) {
    return FMW_Result_Error_InvalidArguments;
  }
  __HAL_TIM_SET_COUNTER(encoder->timer, (encoder->timer->Init.Period / 2));
  return FMW_Result_Ok;
}

FMW_Result fmw_encoder_count_get(const FMW_Encoder *encoder, int32_t *ticks) {
  if (encoder < fmw_state.encoders || encoder >= (fmw_state.encoders + fmw_state.encoders_count)) {
    return FMW_Result_Error_InvalidArguments;
  }
  *ticks = (int32_t)__HAL_TIM_GET_COUNTER(encoder->timer) - (encoder->timer->Init.Period / 2);
  return FMW_Result_Ok;
}

// ============================================================
// Odometry
void fmw_odometry_pose_update(FMW_Odometry *odometry, float meters_traveled_left, float meters_traveled_right) {
  static Mat3Float robot_pose  = {{{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}}};
  static const float tolerance = 0.005f;

  Mat3Float robot_robotranslation = {0};
  Mat3Float robot_from_cir = {0};
  Mat3Float cir_rotation = {0};
  Mat3Float cir_from_robot = {0};

  // calcolo dell'angolo di rotazione
  float theta = -(meters_traveled_left - meters_traveled_right) / odometry->baseline;

  // soglia di tolleranza -> controllo la direzione del robot se va dritto
  if (ABS(theta) < tolerance) {
    /* traslazione lungo x, calcolo della matrice di rototraslazione per traiettoria dritta */
    robot_robotranslation.values[0][0] = 1; robot_robotranslation.values[0][1] = 0; robot_robotranslation.values[0][2] = (meters_traveled_left + meters_traveled_right) / 2;
    robot_robotranslation.values[1][0] = 0; robot_robotranslation.values[1][1] = 1; robot_robotranslation.values[1][2] = 0;
    robot_robotranslation.values[2][0] = 0; robot_robotranslation.values[2][1] = 0; robot_robotranslation.values[2][2] = 1;
  } else {
    // traiettoria lungo una curva
    // distanza del centro del robot dal CIR
    float d_cir = (meters_traveled_right / theta) - (odometry->baseline / 2.f);
    // matrice cir_from_robot di traslazione al CIR
    cir_from_robot.values[0][0] = 1; cir_from_robot.values[0][1] = 0; cir_from_robot.values[0][2] = 0;
    cir_from_robot.values[1][0] = 0; cir_from_robot.values[1][1] = 1; cir_from_robot.values[1][2] = -d_cir;
    cir_from_robot.values[2][0] = 0; cir_from_robot.values[2][1] = 0; cir_from_robot.values[2][2] = 1;
    // matrice r di rotazione attorno al CIR
    cir_rotation.values[0][0] = cosf(theta); cir_rotation.values[0][1] = -sinf(theta); cir_rotation.values[0][2] = 0;
    cir_rotation.values[1][0] = sinf(theta); cir_rotation.values[1][1] = cosf(theta);  cir_rotation.values[1][2] = 0;
    cir_rotation.values[2][0] = 0;           cir_rotation.values[2][1] = 0;            cir_rotation.values[2][2] = 1;
    // matrice robot_from_cir di ri-traslazione dal CIR
    robot_from_cir.values[0][0] = 1; robot_from_cir.values[0][1] = 0; robot_from_cir.values[0][2] = 0;
    robot_from_cir.values[1][0] = 0; robot_from_cir.values[1][1] = 1; robot_from_cir.values[1][2] = d_cir;
    robot_from_cir.values[2][0] = 0; robot_from_cir.values[2][1] = 0; robot_from_cir.values[2][2] = 1;
    // calcolo matrice di ROTOTRASLAZIONE corrente
    robot_robotranslation = mat3float_multiply(robot_from_cir, mat3float_multiply(cir_rotation, cir_from_robot));
  }

  // calcolo (nuova) ROBOT_POSE
  robot_pose = mat3float_multiply(robot_pose, robot_robotranslation);
  odometry->position.x = robot_pose.values[0][2];
  odometry->position.y = robot_pose.values[1][2];
  odometry->orientation.x = robot_pose.values[0][0];
  odometry->orientation.y = robot_pose.values[0][1];
}

// ============================================================
// PID
int32_t fmw_pid_update(FMW_PidController *pid, float velocity) {
  pid->error = pid->setpoint - velocity;
  float output = pid->error * pid->ks.fields.proportional;

  // integral term without windup
  pid->error_sum += pid->error;
  output += pid->error_sum * pid->ks.fields.integral;

  output += (pid->error - pid->previous_error) * pid->ks.fields.derivative;
  pid->previous_error = pid->error;

  // anti windup
  pid->error_sum -= pid->error;
  int32_t integer_output = CLAMP(((int32_t)output), pid_min, pid_max);
  return integer_output;
}

// ============================================================
// LEDs
FMW_Result fmw_led_init(FMW_Led *led) {
  if ((led->timer == NULL) || (led->adc == NULL) ||
      (led->voltage_red <= 0.f) || (led->voltage_orange <= 0.f) || (led->voltage_hysteresis < 0.f) ||
      (led->timer_channel != TIM_CHANNEL_1 && led->timer_channel != TIM_CHANNEL_2 &&
       led->timer_channel != TIM_CHANNEL_3 && led->timer_channel != TIM_CHANNEL_4 &&
       led->timer_channel != TIM_CHANNEL_5 && led->timer_channel != TIM_CHANNEL_6 &&
       led->timer_channel != TIM_CHANNEL_ALL) || (led->state >= FMW_LedState_COUNT)) {
    return FMW_Result_Error_InvalidArguments;
  }

  HAL_StatusTypeDef status = HAL_TIM_PWM_Start(led->timer, led->timer_channel);
  if (status != HAL_OK) { return FMW_Result_Error_Hal; }
  __HAL_TIM_SET_COMPARE(led->timer, led->timer_channel, 0);
  return FMW_Result_Ok;
}

FMW_Result fmw_led_deinit(FMW_Led *led) {
  if ((led->timer == NULL) || (led->adc == NULL) ||
      (led->voltage_red <= 0.f) || (led->voltage_orange <= 0.f) || (led->voltage_hysteresis < 0.f) ||
      (led->timer_channel != TIM_CHANNEL_1 && led->timer_channel != TIM_CHANNEL_2 &&
       led->timer_channel != TIM_CHANNEL_3 && led->timer_channel != TIM_CHANNEL_4 &&
       led->timer_channel != TIM_CHANNEL_5 && led->timer_channel != TIM_CHANNEL_6 &&
       led->timer_channel != TIM_CHANNEL_ALL) || (led->state >= FMW_LedState_COUNT)) {
    return FMW_Result_Error_InvalidArguments;
  }
  HAL_StatusTypeDef status = HAL_TIM_PWM_Stop(led->timer, led->timer_channel);
  if (status == HAL_OK) { return FMW_Result_Error_Hal; }
  return FMW_Result_Ok;
}

FMW_Result fmw_led_update(FMW_Led *led) {
  if ((led->timer == NULL) || (led->adc == NULL) ||
      (led->voltage_red <= 0.f) || (led->voltage_orange <= 0.f) || (led->voltage_hysteresis < 0.f) ||
      (led->timer_channel != TIM_CHANNEL_1 && led->timer_channel != TIM_CHANNEL_2 &&
       led->timer_channel != TIM_CHANNEL_3 && led->timer_channel != TIM_CHANNEL_4 &&
       led->timer_channel != TIM_CHANNEL_5 && led->timer_channel != TIM_CHANNEL_6 &&
       led->timer_channel != TIM_CHANNEL_ALL) || (led->state >= FMW_LedState_COUNT)) {
    return FMW_Result_Error_InvalidArguments;
  }
  HAL_StatusTypeDef adc_start_res = HAL_ADC_Start(led->adc);
  if (adc_start_res != HAL_OK) { return FMW_Result_Error_Hal; }
  HAL_StatusTypeDef adc_poll_res = HAL_ADC_PollForConversion(led->adc, HAL_MAX_DELAY);
  if (adc_poll_res != HAL_OK) { return FMW_Result_Error_Hal; }

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
  return FMW_Result_Ok;
}

// ============================================================
// Buzzers
// NOTE(lb): replace bool with uint8_t bitmask?
FMW_Result fmw_buzzers_set(FMW_Buzzer buzzer[], int32_t count, bool on) {
  if (count <= 0) { return FMW_Result_Error_InvalidArguments; }
  for (int32_t i = 0; i < count; ++i) {
    HAL_StatusTypeDef res;
    if (on) {
      __HAL_TIM_SET_COMPARE(buzzer[i].timer, buzzer[i].timer_channel,
                            buzzer[i].timer->Init.Period / 2);
      res = HAL_TIM_PWM_Start(buzzer[i].timer, buzzer[i].timer_channel);
    } else {
      res = HAL_TIM_PWM_Stop(buzzer[i].timer, buzzer[i].timer_channel);
    }
    if (res != HAL_OK) { return FMW_Result_Error_Hal; }
  }
  return FMW_Result_Ok;
}
