#ifndef FMW_MESSAGE_H
#define FMW_MESSAGE_H

// TODO(lb): fill with sensible values
#define FMW_MIN_ACCEPTABLE_TICKS_PER_REVOLUTION 1
#define FMW_MAX_ACCEPTABLE_TICKS_PER_REVOLUTION 10000

// TODO(lb): fill with sensible values
#define FMW_MIN_ACCEPTABLE_PID_PROPORTIONAL -1.f
#define FMW_MAX_ACCEPTABLE_PID_PROPORTIONAL  1.f

// TODO(lb): fill with sensible values
#define FMW_MIN_ACCEPTABLE_PID_INTEGRAL -1.f
#define FMW_MAX_ACCEPTABLE_PID_INTEGRAL  1.f

// TODO(lb): fill with sensible values
#define FMW_MIN_ACCEPTABLE_PID_DERIVATIVE -1.f
#define FMW_MAX_ACCEPTABLE_PID_DERIVATIVE  1.f

// TODO(lb): fill with sensible values
#define FMW_MIN_ACCEPTABLE_LED_VOLTAGE 0.f
#define FMW_MAX_ACCEPTABLE_LED_VOLTAGE 1.f

// TODO(lb): fill with sensible values
#define FMW_MIN_ACCEPTABLE_LED_UPDATE_PERIOD 1
#define FMW_MAX_ACCEPTABLE_LED_UPDATE_PERIOD 10000

typedef union {
  struct {
    float proportional;
    float integral;
    float derivative;
  };
  float values[3];
} FMW_PidConstants;

typedef uint16_t FMW_MessageStatusCode;
enum {
  FMW_MessageStatusCode_None,

  FMW_MessageStatusCode_Waiting4Config = 0,
  FMW_MessageStatusCode_Running        = 1,
  FMW_MessageStatusCode_Error_Config   = 2,
  FMW_MessageStatusCode_Error_Velocity = 3,
  FMW_MessageStatusCode_Fault_HBridge  = 4,

  FMW_MessageStatusCode_COUNT,
};

typedef uint8_t FMW_State;
enum {
  FMW_State_None,

  FMW_State_Init,
  FMW_State_Error,
  FMW_State_Running,

  FMW_State_COUNT,
};

typedef uint8_t FMW_MessageType;
enum {
  FMW_MessageType_None,

  FMW_MessageType_Error,
  FMW_MessageType_Run,
  FMW_MessageType_Config_Robot,
  FMW_MessageType_Config_PID,
  FMW_MessageType_Config_LED,
  FMW_MessageType_Status,
  FMW_MessageType_Velocity,

  FMW_MessageType_COUNT,
};

#define FMW_RESULT_VARIANTS(X)                                                  \
  X(FMW_Result_Ok)                                                              \
  X(FMW_Result_Error_NilMessage)                                                \
  X(FMW_Result_Error_UART_Crc)                                                  \
  X(FMW_Result_Error_UART_NilHandle)                                            \
  X(FMW_Result_Error_UART_NegativeTimeout)                                      \
  X(FMW_Result_Error_UART_ReceiveTimeoutElapsed)                                \
  X(FMW_Result_Error_MessageHandler_InvalidState)                               \
  X(FMW_Result_Error_MessageHandler_Init_NonPositiveBaseline)                   \
  X(FMW_Result_Error_MessageHandler_Init_NonPositiveWheelCircumference)         \
  X(FMW_Result_Error_MessageHandler_Init_InvalidTicksPerRevolution)             \
  X(FMW_Result_Error_MessageHandler_Init_InvalidPIDProportionalConstant)        \
  X(FMW_Result_Error_MessageHandler_Init_InvalidPIDIntegralConstant)            \
  X(FMW_Result_Error_MessageHandler_Init_InvalidPIDDerivativeConstant)          \
  X(FMW_Result_Error_MessageHandler_Init_InvalidLEDVoltage)                     \
  X(FMW_Result_Error_Command_NotRecognized)                                     \
  X(FMW_Result_Error_Command_NotAvailable)                                      \
  X(FMW_Result_COUNT)

typedef uint8_t FMW_Result;
enum {
#define X(Variant) Variant,
  FMW_RESULT_VARIANTS(X)
#undef X
};

#pragma pack(push, 1)
typedef struct {
  struct {
    FMW_MessageType type;
    uint32_t crc;
  } header;

  union {
    struct {
      const char *filename;
      int32_t line;
      int32_t filename_size;
      FMW_Result reason;
    } error;

    struct {
      float baseline;
      float wheel_circumference_left;
      float wheel_circumference_right;
      uint32_t ticks_per_revolution_left;
      uint32_t ticks_per_revolution_right;
    } config_robot;
    struct {
      FMW_PidConstants left;
      FMW_PidConstants right;
      FMW_PidConstants cross;
    } config_pid;
    struct {
      float voltage_red;
      float voltage_orange;
      float voltage_hysteresis;
      uint32_t update_period;
    } config_led;

    struct {
      FMW_MessageStatusCode status_code;
      uint16_t delta_millis;
      int32_t left_ticks;
      int32_t right_ticks;
      uint32_t crc;
    } status;
    struct {
      float linear_velocity;
      float angular_velocity;
      uint32_t crc;
    } velocity;
  };
} FMW_Message;
#pragma pack(pop)

#endif /* INC_COMMUNICATION_OTTO_MESSAGES_H_ */
