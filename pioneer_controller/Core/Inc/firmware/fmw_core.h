#ifndef FMW_CORE_H
#define FMW_CORE_H

typedef struct {
  TIM_HandleTypeDef *const timer;
  uint32_t previous_millis;
  uint32_t current_millis;
  int32_t ticks;
  int32_t ticks_per_revolution;
  float wheel_circumference; // NOTE(lb): measured in meters
} FMW_Encoder;

typedef struct {
  GPIO_TypeDef *const sleep_gpio_port;
  GPIO_TypeDef *const dir_gpio_port;
  TIM_HandleTypeDef *const pwm_timer;
  uint32_t pwm_channel;
  int32_t max_dutycycle;
  uint16_t sleep_pin;
  uint16_t dir_pin;
  bool active;
} FMW_Motor;

typedef uint8_t FMW_MotorDirection;
enum {
  FMW_MotorDirection_Backward = GPIO_PIN_RESET,
  FMW_MotorDirection_Forward  = GPIO_PIN_SET,
};

typedef struct {
  float baseline; // NOTE(lb): measured in meters
  float velocity_linear;
  float velocity_angular;
  float setpoint_right;
  float setpoint_left;
} FMW_Odometry;

typedef struct {
  FMW_PidConstants ks;
  float error;
  float setpoint;
  float error_sum;      // needed for integrative term
  float previous_error; // needed for derivative term
} FMW_PidController;

typedef uint8_t FMW_LedState;
enum {
  FMW_LedState_Red    = 0,
  FMW_LedState_Orange = 1,
  FMW_LedState_Green  = 2,
};

typedef struct {
  TIM_HandleTypeDef *const timer;
  ADC_HandleTypeDef *const adc;
  uint32_t timer_channel;
  float voltage_red;
  float voltage_orange;
  float voltage_hysteresis;
  FMW_LedState state;
} FMW_Led;

void fmw_motor_init(FMW_Motor motors[FMW_MOTOR_COUNT]);
void fmw_motor_set_speed(FMW_Motor *motor, int32_t duty_cycle);
void fmw_motor_brake(FMW_Motor *motor);
void fmw_motor_enable(FMW_Motor * motor);
void fmw_motor_disable(FMW_Motor * motor);

void fmw_encoder_init(FMW_Encoder encoders[FMW_ENCODER_COUNT]);
void fmw_encoder_update(FMW_Encoder *encoder);
float fmw_encoder_get_linear_velocity(const FMW_Encoder *encoder);
void fmw_encoder_count_reset(FMW_Encoder *encoder);
int32_t fmw_encoder_count_get(const FMW_Encoder *encoder);

int32_t fmw_pid_update(FMW_PidController *pid, float measure);

void fmw_led_init(FMW_Led *led);
void fmw_led_update(FMW_Led *led);

void fmw_report_handler(FMW_Error error_code, const char *filename, int32_t filename_length, int32_t line);
void fmw_message_handle(FMW_State state, UART_HandleTypeDef *huart, int32_t wait_ms);

#define FMW_LED_UPDATE_PERIOD 200
#define FMW_DEBOUNCE_DELAY 200

#define FMW_V_REF 3.3f
#define FMW_ADC_RESOLUTION 4095.0f
#define FMW_VIN_SCALE_FACTOR 3.733f

#define FMW_METERS_FROM_TICKS(Ticks, WheelCircumference, TicksPerRevolution) \
  ((Ticks * WheelCircumference) / TicksPerRevolution)

#define FMW_REPORT_UNLESS(Cond, MessageStatusCode)        \
  do {                                                    \
    if (!(Cond)) {                                        \
      fmw_report_handler(MessageStatusCode, __FILE__,     \
                          ARRLENGTH(__FILE__), __LINE__); \
    }                                                     \
  } while (0)

#define FMW_REPORT(MessageStatusCode)                \
  fmw_report_handler(MessageStatusCode, __FILE__,    \
                      ARRLENGTH(__FILE__), __LINE__)

#endif
