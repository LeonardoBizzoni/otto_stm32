#ifndef FMW_CORE_H
#define FMW_CORE_H

typedef uint8_t FMW_Mode;
enum {
  FMW_Mode_None         = 0,
  FMW_Mode_Config       = 1,
  FMW_Mode_Run          = 2,
  FMW_Mode_Emergency    = 3,
  FMW_Mode_COUNT        = 4,
};

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
  float baseline;       // NOTE(lb): measured in meters
  Vec2Float position;   // NOTE(lb): measured in meters
  Vec2Float orientation;
  Vec2Float velocity_linear; // NOTE(lb): measured in m/s
  float velocity_angular;
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

typedef struct FMW_InitInfo {
  struct {
    UART_HandleTypeDef *huart;
    CRC_HandleTypeDef *hcrc;
    void (*handler)(FMW_Message *msg, CRC_HandleTypeDef *hcrc);
  } message_exchange;

  struct {
    TIM_HandleTypeDef *timer;
    void (*on_begin)(void);
    void (*on_end)(void);
    uint32_t wait_at_most_ms_before_emergency;
  } emergency;

  FMW_Motor *motors;
  int32_t motors_count;
} FMW_InitInfo;

void fmw_init(const FMW_InitInfo *info) __attribute__((nonnull));

void fmw_uart_message_dispatch(void);
void fmw_uart_message_send(FMW_Message *msg) __attribute__((nonnull));
void fmw_uart_error(void);

void fmw_emergency_begin(void);
void fmw_emergency_end(void);
void fmw_emergency_timer_update(void);

FMW_Mode fmw_mode_current(void);
FMW_Mode fmw_mode_transition(FMW_Mode mode);

FMW_Result fmw_result_from_uart_error(void) __attribute__((nonnull, warn_unused_result));

void fmw_motors_init(FMW_Motor motors[], int32_t count)         __attribute__((nonnull));
void fmw_motors_deinit(FMW_Motor motors[], int32_t count)       __attribute__((nonnull));
void fmw_motors_stop(FMW_Motor motors[], int32_t count)         __attribute__((nonnull));
void fmw_motors_enable(FMW_Motor motors[], int32_t count)       __attribute__((nonnull));
void fmw_motors_disable(FMW_Motor motors[], int32_t count)      __attribute__((nonnull));
void fmw_motor_set_speed(FMW_Motor *motor, int32_t duty_cycle)  __attribute__((nonnull));

void fmw_encoders_init(FMW_Encoder encoders[], int32_t count)                                   __attribute__((nonnull));
void fmw_encoders_deinit(FMW_Encoder encoders[], int32_t count)                                 __attribute__((nonnull));
void fmw_encoders_update(FMW_Encoder encoders[], int32_t count)                                 __attribute__((nonnull));
float fmw_encoder_get_linear_velocity(const FMW_Encoder *encoder, float meters_traveled)        __attribute__((warn_unused_result, nonnull));
void fmw_encoder_count_reset(FMW_Encoder *encoder)                                              __attribute__((nonnull));
int32_t fmw_encoder_count_get(const FMW_Encoder *encoder)                                       __attribute__((warn_unused_result, nonnull));

void fmw_odometry_pose_update(FMW_Odometry *odometry, float meters_traveled_left, float meters_traveled_right) __attribute__((nonnull));

int32_t fmw_pid_update(FMW_PidController *pid, float velocity) __attribute__((warn_unused_result, nonnull));

void fmw_led_init(FMW_Led *led)         __attribute__((nonnull));
void fmw_led_deinit(FMW_Led *led)       __attribute__((nonnull));
void fmw_led_update(FMW_Led *led)       __attribute__((nonnull));

void fmw_buzzers_set(FMW_Buzzer buzzer[], int32_t count, bool on) __attribute__((nonnull));

Vec2Float fmw_setpoint_from_velocities(const FMW_Odometry *odometry, float linear, float angular) __attribute__((nonnull));

#define FMW_LED_UPDATE_PERIOD 200
#define FMW_DEBOUNCE_DELAY 200
#define FMW_EMERGENCY_MODE_EXIT_BUTTON_HOLD_DURATION_MS 2000

#define FMW_V_REF 3.3f
#define FMW_ADC_RESOLUTION 4095.0f
#define FMW_VIN_SCALE_FACTOR 3.733f

#define FMW_METERS_FROM_TICKS(Ticks, WheelCircumference, TicksPerRevolution) \
  ((Ticks * WheelCircumference) / TicksPerRevolution)

// NOTE(lb): The variadic arguments are the fields of FMW_Hook, so you can just type
//           `.callback = your_function_pointer, .args = your_pointer_to_args`.
//           Calling this macro with the condition argument is GCC extension.
//           I don't use `assert` because i never figured out how to make it trigger
//           a debug breakpoint inside the STM32 ide debugger (and also because having a
//           on-failure callback is handy).
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
