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
  int32_t motors_count;
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

static void fmw_hook_assert_fail(void *_) {
  if (fmw_state.motors != NULL) {
    fmw_motors_stop(fmw_state.motors, fmw_state.motors_count);
  }
}

// ============================================================
// Firmware initialization
void fmw_init(const FMW_InitInfo *info) {
  FMW_ASSERT(info->message_exchange.huart != NULL);
  FMW_ASSERT(info->message_exchange.hcrc != NULL);
  FMW_ASSERT(info->message_exchange.handler != NULL);
  FMW_ASSERT(info->emergency.on_begin != NULL);
  FMW_ASSERT(info->emergency.on_end != NULL);
  FMW_ASSERT(info->motors_count >= 0);
  if (info->motors_count > 0) { FMW_ASSERT(info->motors != NULL); }

  fmw_state.motors = info->motors;
  fmw_state.motors_count = info->motors_count;
  fmw_state.huart = info->message_exchange.huart;
  fmw_state.hcrc = info->message_exchange.hcrc;
  fmw_state.emergency_mode_grace_period_ms = info->emergency.wait_at_most_ms_before_emergency;
  fmw_state.callback_message_handler = info->message_exchange.handler;
  fmw_state.callback_emergency_begin = info->emergency.on_begin;
  fmw_state.callback_emergency_end = info->emergency.on_end;
  fmw_state.emergency_timer = info->emergency.timer;

  HAL_StatusTypeDef timer_init_res = HAL_TIM_Base_Start_IT(info->emergency.timer);
  FMW_ASSERT(timer_init_res == HAL_OK);

  HAL_StatusTypeDef dma_init_res = HAL_UART_Receive_DMA(fmw_state.huart, (uint8_t*)&fmw_state.uart_buffer, sizeof fmw_state.uart_buffer);
  FMW_ASSERT(dma_init_res == HAL_OK);
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
  fmw_motors_stop(fmw_state.motors, fmw_state.motors_count);
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
      fmw_motors_stop(fmw_state.motors, fmw_state.motors_count);
      fmw_motors_disable(fmw_state.motors, fmw_state.motors_count);
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
    fmw_motors_enable(fmw_state.motors, fmw_state.motors_count);
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

FMW_Mode fmw_mode_transition(FMW_Mode mode) {
  FMW_ASSERT(mode > FMW_Mode_None);
  FMW_ASSERT(mode < FMW_Mode_COUNT);
  FMW_Mode old = fmw_state.mode_previous;
  fmw_state.mode_previous = fmw_state.mode_current;
  fmw_state.mode_current = mode;
  return old;
}

// ============================================================
// Misc
FMW_Result fmw_message_uart_receive(UART_HandleTypeDef *huart, FMW_Message *msg, int32_t wait_ms) {
  FMW_ASSERT(wait_ms >= 0, .callback = fmw_hook_assert_fail);
  memset(msg, 0, sizeof *msg);

  HAL_StatusTypeDef uart_packet_status = HAL_UART_Receive(huart, (uint8_t*)msg, sizeof(*msg), wait_ms);
  if (!(uart_packet_status == HAL_OK)) { return FMW_Result_Error_UART_ReceiveTimeoutElapsed; }
  if (!(msg->header.type > FMW_MessageType_None && msg->header.type < FMW_MessageType_COUNT)) {
    return FMW_Result_Error_Command_NotRecognized;
  }
  return FMW_Result_Ok;
}

void fmw_uart_message_send(FMW_Message *msg) {
  msg->header.crc = HAL_CRC_Calculate(fmw_state.hcrc, (uint32_t*)msg, sizeof *msg);
  HAL_StatusTypeDef res = HAL_UART_Transmit(fmw_state.huart, (uint8_t*)msg, sizeof *msg, fmw_state.emergency_mode_grace_period_ms);
  if (res != HAL_OK) { fmw_emergency_begin(); }
}

FMW_Result fmw_result_from_uart_error(void) {
  FMW_ASSERT(fmw_state.huart->ErrorCode != HAL_UART_ERROR_NONE, .callback = fmw_hook_assert_fail);
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
  default: { // NOTE(lb): unreachable
    FMW_ASSERT(false, .callback = fmw_hook_assert_fail);
  } break;
  }
  return FMW_Result_Ok;
}

Vec2Float fmw_setpoint_from_velocities(const FMW_Odometry *odometry, float linear, float angular) {
  FMW_ASSERT(odometry->baseline > 0.f, .callback = fmw_hook_assert_fail);
  Vec2Float res = {
    .left = linear - (odometry->baseline * angular) / 2.f,
    .right = linear + (odometry->baseline * angular) / 2.f,
  };
  return res;
}

// ============================================================
// Motor controller
void fmw_motors_init(FMW_Motor motors[], int32_t count) {
  FMW_ASSERT(count > 0);
  fmw_state.motors = motors;
  fmw_state.motors_count = count;

  for (int32_t i = 0; i < count; ++i) {
    FMW_ASSERT(motors[i].sleep_gpio_port != NULL);
    FMW_ASSERT(motors[i].dir_gpio_port != NULL);
    FMW_ASSERT(motors[i].pwm_timer != NULL);
    FMW_ASSERT(motors[i].pwm_channel == TIM_CHANNEL_1 || motors[i].pwm_channel == TIM_CHANNEL_2 ||
               motors[i].pwm_channel == TIM_CHANNEL_3 || motors[i].pwm_channel == TIM_CHANNEL_4 ||
               motors[i].pwm_channel == TIM_CHANNEL_5 || motors[i].pwm_channel == TIM_CHANNEL_6 ||
               motors[i].pwm_channel == TIM_CHANNEL_ALL);
    FMW_ASSERT(motors[i].sleep_pin == GPIO_PIN_1 || motors[i].sleep_pin == GPIO_PIN_2 ||
               motors[i].sleep_pin == GPIO_PIN_3 || motors[i].sleep_pin == GPIO_PIN_4 ||
               motors[i].sleep_pin == GPIO_PIN_5 || motors[i].sleep_pin == GPIO_PIN_6 ||
               motors[i].sleep_pin == GPIO_PIN_7 || motors[i].sleep_pin == GPIO_PIN_8 ||
               motors[i].sleep_pin == GPIO_PIN_9 || motors[i].sleep_pin == GPIO_PIN_10 ||
               motors[i].sleep_pin == GPIO_PIN_11 || motors[i].sleep_pin == GPIO_PIN_12 ||
               motors[i].sleep_pin == GPIO_PIN_13 || motors[i].sleep_pin == GPIO_PIN_14 ||
               motors[i].sleep_pin == GPIO_PIN_15 || motors[i].sleep_pin == GPIO_PIN_All);
    FMW_ASSERT(motors[i].dir_pin == GPIO_PIN_1 || motors[i].dir_pin == GPIO_PIN_2 ||
               motors[i].dir_pin == GPIO_PIN_3 || motors[i].dir_pin == GPIO_PIN_4 ||
               motors[i].dir_pin == GPIO_PIN_5 || motors[i].dir_pin == GPIO_PIN_6 ||
               motors[i].dir_pin == GPIO_PIN_7 || motors[i].dir_pin == GPIO_PIN_8 ||
               motors[i].dir_pin == GPIO_PIN_9 || motors[i].dir_pin == GPIO_PIN_10 ||
               motors[i].dir_pin == GPIO_PIN_11 || motors[i].dir_pin == GPIO_PIN_12 ||
               motors[i].dir_pin == GPIO_PIN_13 || motors[i].dir_pin == GPIO_PIN_14 ||
               motors[i].dir_pin == GPIO_PIN_15 || motors[i].dir_pin == GPIO_PIN_All);
    FMW_ASSERT(motors[i].dir_pin != motors[i].sleep_pin);

    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(motors[i].pwm_timer, motors[i].pwm_channel);
    FMW_ASSERT(status == HAL_OK);
    motors[i].max_dutycycle = motors[i].pwm_timer->Instance->ARR;
    motors[i].active = true;
  }
  fmw_motors_stop(motors, count);
}

void fmw_motors_deinit(FMW_Motor motors[], int32_t count) {
  for (int32_t i = 0; i < count; ++i) {
    HAL_StatusTypeDef status = HAL_TIM_PWM_Stop(motors[i].pwm_timer, motors[i].pwm_channel);
    FMW_ASSERT(status == HAL_OK);
    motors[i].active = false;
  }
}

void fmw_motor_set_speed(FMW_Motor *motor, int32_t duty_cycle) {
  FMW_ASSERT(motor->dir_gpio_port != NULL, .callback = fmw_hook_assert_fail);
  FMW_ASSERT(motor->dir_pin == GPIO_PIN_1 || motor->dir_pin == GPIO_PIN_2 ||
             motor->dir_pin == GPIO_PIN_3 || motor->dir_pin == GPIO_PIN_4 ||
             motor->dir_pin == GPIO_PIN_5 || motor->dir_pin == GPIO_PIN_6 ||
             motor->dir_pin == GPIO_PIN_7 || motor->dir_pin == GPIO_PIN_8 ||
             motor->dir_pin == GPIO_PIN_9 || motor->dir_pin == GPIO_PIN_10 ||
             motor->dir_pin == GPIO_PIN_11 || motor->dir_pin == GPIO_PIN_12 ||
             motor->dir_pin == GPIO_PIN_13 || motor->dir_pin == GPIO_PIN_14 ||
             motor->dir_pin == GPIO_PIN_15 || motor->dir_pin == GPIO_PIN_All,
             .callback = fmw_hook_assert_fail);
  HAL_GPIO_WritePin(motor->dir_gpio_port, motor->dir_pin,
                    (duty_cycle >= 0 ? FMW_MotorDirection_Forward : FMW_MotorDirection_Backward));
  duty_cycle = CLAMP_TOP(ABS(duty_cycle), motor->max_dutycycle);
  __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, duty_cycle);
  HAL_GPIO_WritePin(motor->sleep_gpio_port, motor->sleep_pin, GPIO_PIN_SET);
}

void fmw_motors_stop(FMW_Motor motors[], int32_t count) {
  FMW_ASSERT(count > 0);
  for (int32_t i = 0; i < count; ++i) {
    FMW_ASSERT(motors[i].sleep_gpio_port != NULL);
    FMW_ASSERT(motors[i].sleep_pin == GPIO_PIN_1 || motors[i].sleep_pin == GPIO_PIN_2 ||
               motors[i].sleep_pin == GPIO_PIN_3 || motors[i].sleep_pin == GPIO_PIN_4 ||
               motors[i].sleep_pin == GPIO_PIN_5 || motors[i].sleep_pin == GPIO_PIN_6 ||
               motors[i].sleep_pin == GPIO_PIN_7 || motors[i].sleep_pin == GPIO_PIN_8 ||
               motors[i].sleep_pin == GPIO_PIN_9 || motors[i].sleep_pin == GPIO_PIN_10 ||
               motors[i].sleep_pin == GPIO_PIN_11 || motors[i].sleep_pin == GPIO_PIN_12 ||
               motors[i].sleep_pin == GPIO_PIN_13 || motors[i].sleep_pin == GPIO_PIN_14 ||
               motors[i].sleep_pin == GPIO_PIN_15 || motors[i].sleep_pin == GPIO_PIN_All);
    FMW_ASSERT(motors[i].pwm_timer != NULL);
    FMW_ASSERT(motors[i].pwm_channel == TIM_CHANNEL_1 || motors[i].pwm_channel == TIM_CHANNEL_2 ||
               motors[i].pwm_channel == TIM_CHANNEL_3 || motors[i].pwm_channel == TIM_CHANNEL_4 ||
               motors[i].pwm_channel == TIM_CHANNEL_5 || motors[i].pwm_channel == TIM_CHANNEL_6 ||
               motors[i].pwm_channel == TIM_CHANNEL_ALL);

    HAL_GPIO_WritePin(motors[i].sleep_gpio_port, motors[i].sleep_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motors[i].pwm_timer, motors[i].pwm_channel, 0);
  }
}

void fmw_motors_enable(FMW_Motor motors[], int32_t count) {
  FMW_ASSERT(count > 0);
  for (int32_t i = 0; i < count; ++i) {
    FMW_ASSERT(motors[i].pwm_timer != NULL);
    FMW_ASSERT(motors[i].pwm_channel == TIM_CHANNEL_1 || motors[i].pwm_channel == TIM_CHANNEL_2 ||
               motors[i].pwm_channel == TIM_CHANNEL_3 || motors[i].pwm_channel == TIM_CHANNEL_4 ||
               motors[i].pwm_channel == TIM_CHANNEL_5 || motors[i].pwm_channel == TIM_CHANNEL_6 ||
               motors[i].pwm_channel == TIM_CHANNEL_ALL);
    FMW_ASSERT(!motors[i].active);
    HAL_StatusTypeDef res = HAL_TIM_PWM_Start(motors[i].pwm_timer, motors[i].pwm_channel);
    FMW_ASSERT(res == HAL_OK);
    motors[i].active = true;
  }
}

void fmw_motors_disable(FMW_Motor motors[], int32_t count) {
  FMW_ASSERT(count > 0, .callback = fmw_hook_assert_fail);
  for (int32_t i = 0; i < count; ++i) {
    FMW_ASSERT(motors[i].pwm_timer != NULL, .callback = fmw_hook_assert_fail);
    FMW_ASSERT(motors[i].pwm_channel == TIM_CHANNEL_1 || motors[i].pwm_channel == TIM_CHANNEL_2 ||
               motors[i].pwm_channel == TIM_CHANNEL_3 || motors[i].pwm_channel == TIM_CHANNEL_4 ||
               motors[i].pwm_channel == TIM_CHANNEL_5 || motors[i].pwm_channel == TIM_CHANNEL_6 ||
               motors[i].pwm_channel == TIM_CHANNEL_ALL, .callback = fmw_hook_assert_fail);
    FMW_ASSERT(motors[i].active);
    HAL_StatusTypeDef res = HAL_TIM_PWM_Stop(motors[i].pwm_timer, motors[i].pwm_channel);
    FMW_ASSERT(res == HAL_OK, .callback = fmw_hook_assert_fail);
    motors[i].active = false;
  }
}

// ============================================================
// Encoder
void fmw_encoders_init(FMW_Encoder encoders[], int32_t count) {
  for (int32_t i = 0; i < count; ++i) {
    FMW_ASSERT(encoders[i].timer != NULL);
    FMW_ASSERT(encoders[i].ticks_per_revolution > 0);
    FMW_ASSERT(encoders[i].wheel_circumference > 0.f);

    encoders[i].previous_millis = 0;
    encoders[i].current_millis = 0;
    encoders[i].ticks = 0;

    HAL_StatusTypeDef status = HAL_TIM_Encoder_Start(encoders[i].timer, TIM_CHANNEL_ALL);
    FMW_ASSERT(status == HAL_OK);
    fmw_encoder_count_reset(&encoders[i]);
    encoders[i].current_millis = HAL_GetTick();
  }
}

void fmw_encoders_deinit(FMW_Encoder encoders[], int32_t count) {
  for (int32_t i = 0; i < count; ++i) {
    FMW_ASSERT(encoders[i].timer != NULL);
    HAL_StatusTypeDef status = HAL_TIM_Encoder_Stop(encoders[i].timer, TIM_CHANNEL_ALL);
    FMW_ASSERT(status == HAL_OK);
  }
}

void fmw_encoders_update(FMW_Encoder encoders[], int32_t count) {
  for (int32_t i = 0; i < count; ++i) {
    encoders[i].previous_millis = encoders[i].current_millis;
    encoders[i].current_millis = HAL_GetTick();
    encoders[i].ticks = fmw_encoder_count_get(&encoders[i]);
    fmw_encoder_count_reset(&encoders[i]);
    FMW_ASSERT(encoders[i].current_millis >= encoders[i].previous_millis, .callback = fmw_hook_assert_fail);
  }
}

float fmw_encoder_get_linear_velocity(const FMW_Encoder *encoder, float meters_traveled) {
  float deltatime = encoder->current_millis - encoder->previous_millis;
  if (deltatime == 0.f) { return 0.f; }
  float linear_velocity = meters_traveled / (deltatime / 1000.f);
  return linear_velocity;
}

void fmw_encoder_count_reset(FMW_Encoder *encoder) {
  FMW_ASSERT(encoder->timer != NULL, .callback = fmw_hook_assert_fail);
  __HAL_TIM_SET_COUNTER(encoder->timer, (encoder->timer->Init.Period / 2));
}

int32_t fmw_encoder_count_get(const FMW_Encoder *encoder) {
  FMW_ASSERT(encoder->timer != NULL, .callback = fmw_hook_assert_fail);
  int32_t res = (int32_t)__HAL_TIM_GET_COUNTER(encoder->timer) - (encoder->timer->Init.Period / 2);
  return res;
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
void fmw_led_init(FMW_Led *led) {
  FMW_ASSERT(led->timer != NULL);
  FMW_ASSERT(led->adc != NULL);
  FMW_ASSERT(led->timer_channel == TIM_CHANNEL_1 || led->timer_channel == TIM_CHANNEL_2 ||
             led->timer_channel == TIM_CHANNEL_3 || led->timer_channel == TIM_CHANNEL_4 ||
             led->timer_channel == TIM_CHANNEL_5 || led->timer_channel == TIM_CHANNEL_6 ||
             led->timer_channel == TIM_CHANNEL_ALL);
  FMW_ASSERT(led->voltage_red > 0.f);
  FMW_ASSERT(led->voltage_orange > 0.f);
  FMW_ASSERT(led->voltage_hysteresis >= 0.f);
  FMW_ASSERT(led->state < FMW_LedState_COUNT);

  HAL_StatusTypeDef status = HAL_TIM_PWM_Start(led->timer, led->timer_channel);
  FMW_ASSERT(status == HAL_OK);
  __HAL_TIM_SET_COMPARE(led->timer, led->timer_channel, 0);
}

void fmw_led_deinit(FMW_Led *led) {
  HAL_StatusTypeDef status = HAL_TIM_PWM_Stop(led->timer, led->timer_channel);
  FMW_ASSERT(status == HAL_OK);
}

void fmw_led_update(FMW_Led *led) {
  FMW_ASSERT(led->timer != NULL, .callback = fmw_hook_assert_fail);
  FMW_ASSERT(led->adc != NULL, .callback = fmw_hook_assert_fail);
  FMW_ASSERT(led->timer_channel == TIM_CHANNEL_1 || led->timer_channel == TIM_CHANNEL_2 ||
             led->timer_channel == TIM_CHANNEL_3 || led->timer_channel == TIM_CHANNEL_4 ||
             led->timer_channel == TIM_CHANNEL_5 || led->timer_channel == TIM_CHANNEL_6 ||
             led->timer_channel == TIM_CHANNEL_ALL, .callback = fmw_hook_assert_fail);
  FMW_ASSERT(led->voltage_red > 0.f, .callback = fmw_hook_assert_fail);
  FMW_ASSERT(led->voltage_orange > 0.f, .callback = fmw_hook_assert_fail);
  FMW_ASSERT(led->voltage_hysteresis >= 0.f, .callback = fmw_hook_assert_fail);
  FMW_ASSERT(led->state < FMW_LedState_COUNT, .callback = fmw_hook_assert_fail);

  HAL_StatusTypeDef adc_start_res = HAL_ADC_Start(led->adc);
  FMW_ASSERT(adc_start_res == HAL_OK, .callback = fmw_hook_assert_fail);
  HAL_StatusTypeDef adc_poll_res = HAL_ADC_PollForConversion(led->adc, HAL_MAX_DELAY);
  FMW_ASSERT(adc_poll_res == HAL_OK, .callback = fmw_hook_assert_fail);

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

// ============================================================
// Buzzers
// NOTE(lb): replace bool with uint8_t bitmask?
void fmw_buzzers_set(FMW_Buzzer buzzer[], int32_t count, bool on) {
  FMW_ASSERT(count >= 0, .callback = fmw_hook_assert_fail);
  for (int32_t i = 0; i < count; ++i) {
    HAL_StatusTypeDef res;
    if (on) {
      __HAL_TIM_SET_COMPARE(buzzer[i].timer, buzzer[i].timer_channel,
                            buzzer[i].timer->Init.Period / 2);
      res = HAL_TIM_PWM_Start(buzzer[i].timer, buzzer[i].timer_channel);
    } else {
      res = HAL_TIM_PWM_Stop(buzzer[i].timer, buzzer[i].timer_channel);
    }
    FMW_ASSERT(res == HAL_OK, .callback = fmw_hook_assert_fail);
  }
}
