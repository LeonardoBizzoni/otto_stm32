#ifndef FMW_MESSAGE_H
#define FMW_MESSAGE_H

typedef union {
  struct {
    float proportional;
    float integral;
    float derivative;
  };
  float values[3];
} FMW_PidConstants;

typedef uint8_t FMW_MessageType;
enum {
  FMW_MessageType_None,

  FMW_MessageType_Error,
  FMW_MessageType_Response,

  FMW_MessageType_StateChange_Init,
  FMW_MessageType_StateChange_Run,

  FMW_MessageType_Config_Robot,
  FMW_MessageType_Config_PID,
  FMW_MessageType_Config_LED,

  FMW_MessageType_Run_GetStatus,
  FMW_MessageType_Run_GetStatus_Response,
  FMW_MessageType_Run_SetVelocity,
  FMW_MessageType_Run_SetVelocity_Response,

  FMW_MessageType_COUNT,
};

#define FMW_RESULT_VARIANTS(X)                                                  \
  X(FMW_Result_Ok)                                                              \
  X(FMW_Result_Error_InvalidArguments)                                          \
  X(FMW_Result_Error_UART_Crc)                                                  \
  X(FMW_Result_Error_UART_NegativeTimeout)                                      \
  X(FMW_Result_Error_UART_ReceiveTimeoutElapsed)                                \
  X(FMW_Result_Error_Encoder_InvalidTimer)                                      \
  X(FMW_Result_Error_Encoder_NonPositiveTicksPerRevolution)                     \
  X(FMW_Result_Error_Encoder_NonPositiveWheelCircumference)                     \
  X(FMW_Result_Error_Encoder_GetTick)                                           \
  X(FMW_Result_Error_Buzzer_Timer)                                              \
  X(FMW_Result_Error_MessageHandler_InvalidState)                               \
  X(FMW_Result_Error_MessageHandler_Init_NonPositiveBaseline)                   \
  X(FMW_Result_Error_MessageHandler_Init_NonPositiveWheelCircumference)         \
  X(FMW_Result_Error_MessageHandler_Init_NonPositiveTicksPerRevolution)         \
  X(FMW_Result_Error_MessageHandler_Init_NonPositiveLEDUpdatePeriod)            \
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
      int32_t line;
      int32_t file_size;
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

    FMW_Result result;
    struct {
      FMW_Result result;
      uint16_t delta_millis;
      int32_t ticks_left;
      int32_t ticks_right;
    } status_response;
    struct {
      float linear;
      float angular;
    } velocity;
  };
} FMW_Message;
#pragma pack(pop)

#endif /* INC_COMMUNICATION_OTTO_MESSAGES_H_ */
