#ifndef FMW_CORE_H
#define FMW_CORE_H

typedef struct FMW_Encoder {
  TIM_HandleTypeDef *const timer;
  uint32_t previous_millis;
  uint32_t current_millis;
  int32_t ticks;
  int32_t ticks_per_revolution;
  float wheel_circumference; // NOTE(lb): measured in meters
} FMW_Encoder;

typedef struct FMW_Motor {
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
  FMW_LedState_COUNT,
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

typedef struct FMW_Buzzer {
  TIM_HandleTypeDef *const timer;
  uint32_t timer_channel;
} FMW_Buzzer;

typedef struct FMW_Hook {
  void (*callback)(void *args);
  void *args;
} FMW_Hook;

void fmw_motor_init(FMW_Motor motors[], int32_t count)          __attribute__((nonnull));
void fmw_motor_set_speed(FMW_Motor *motor, int32_t duty_cycle)  __attribute__((nonnull));
void fmw_motor_brake(FMW_Motor motors[], int32_t count)         __attribute__((nonnull));
void fmw_motor_enable(FMW_Motor * motor)                        __attribute__((nonnull));
void fmw_motor_disable(FMW_Motor * motor)                       __attribute__((nonnull));

void fmw_encoder_init(FMW_Encoder encoders[], int32_t count)            __attribute__((nonnull));
void fmw_encoder_update(FMW_Encoder *encoder)                           __attribute__((nonnull));
float fmw_encoder_get_linear_velocity(const FMW_Encoder *encoder)       __attribute__((warn_unused_result, nonnull));
void fmw_encoder_count_reset(FMW_Encoder *encoder)                      __attribute__((nonnull));
int32_t fmw_encoder_count_get(const FMW_Encoder *encoder)               __attribute__((warn_unused_result, nonnull));

int32_t fmw_pid_update(FMW_PidController *pid, float velocity) __attribute__((warn_unused_result, nonnull));

void fmw_odometry_setpoint_from_velocities(FMW_Odometry *odometry, float linear, float angular) __attribute__((nonnull));

void fmw_led_init(FMW_Led *led)         __attribute__((nonnull));
void fmw_led_update(FMW_Led *led)       __attribute__((nonnull));

void fmw_buzzer_set(FMW_Buzzer buzzer[], int32_t count, bool on) __attribute__((nonnull));

FMW_Result fmw_message_uart_receive(UART_HandleTypeDef *huart, FMW_Message *msg, int32_t wait_ms)                       __attribute__((warn_unused_result, nonnull));
void fmw_message_uart_send(UART_HandleTypeDef *huart, CRC_HandleTypeDef *hcrc, FMW_Message *msg, int32_t wait_ms)       __attribute__((nonnull));

#define FMW_LED_UPDATE_PERIOD 200
#define FMW_DEBOUNCE_DELAY 200

#define FMW_V_REF 3.3f
#define FMW_ADC_RESOLUTION 4095.0f
#define FMW_VIN_SCALE_FACTOR 3.733f

#define FMW_METERS_FROM_TICKS(Ticks, WheelCircumference, TicksPerRevolution) \
  ((Ticks * WheelCircumference) / TicksPerRevolution)

#define FMW_ASSERT(Cond, ...)           \
  do {                                  \
    FMW_Hook hook = { __VA_ARGS__ };    \
    if (!(Cond)) {                      \
      if (hook.callback) {              \
        hook.callback(hook.args);       \
      }                                 \
      __builtin_trap();                 \
    }                                   \
  } while (0)

#endif
