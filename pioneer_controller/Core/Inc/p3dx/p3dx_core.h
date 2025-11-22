#ifndef P3DX_CORE_H
#define P3DX_CORE_H

typedef struct {
  TIM_HandleTypeDef *const timer;
  uint32_t previous_millis;
  uint32_t current_millis;
  int32_t ticks;
  int32_t ticks_per_revolution;
  float wheel_circumference; // NOTE(lb): measured in meters
} P3DX_Encoder;

typedef struct {
  GPIO_TypeDef *const sleep_gpio_port;
  GPIO_TypeDef *const dir_gpio_port;
  TIM_HandleTypeDef *const pwm_timer;
  uint32_t pwm_channel;
  int32_t max_dutycycle;
  uint16_t sleep_pin;
  uint16_t dir_pin;
} P3DX_Motor;

typedef uint8_t P3DX_MotorDirection;
enum {
  P3DX_MotorDirection_Backward = GPIO_PIN_RESET,
  P3DX_MotorDirection_Forward  = GPIO_PIN_SET,
};

typedef struct {
  float baseline; // NOTE(lb): measured in meters
  float velocity_linear;
  float velocity_angular;
  float setpoint_right;
  float setpoint_left;
} P3DX_Odometry;

typedef struct {
  P3DX_PidConstants ks;
  float error;
  float setpoint;
  float error_sum;      // needed for integrative term
  float previous_error; // needed for derivative term
} P3DX_PidController;

void p3dx_encoder_init(P3DX_Encoder encoders[P3DX_ENCODER_COUNT]);
void p3dx_encoder_update(P3DX_Encoder *encoder);
float p3dx_encoder_get_linear_velocity(const P3DX_Encoder *encoder);
void p3dx_encoder_count_reset(P3DX_Encoder *encoder);
int32_t p3dx_encoder_count_get(const P3DX_Encoder *encoder);

void p3dx_motor_init(P3DX_Motor motors[P3DX_MOTOR_COUNT]);
void p3dx_motor_set_speed(P3DX_Motor *motor, int32_t duty_cycle);
void p3dx_motor_brake(P3DX_Motor *motor);
void p3dx_motor_coast(P3DX_Motor * motor);

int32_t p3dx_pid_update(P3DX_PidController *pid, float measure);

void p3dx_report_handler(P3DX_Error error_code, const char *filename, int32_t filename_length, int32_t line);
void p3dx_message_handle(P3DX_State state, UART_HandleTypeDef *huart, int32_t wait_ms);

#endif
