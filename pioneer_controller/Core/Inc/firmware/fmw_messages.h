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

#define FMW_MESSAGE_TYPE_VARIANTS(X)            \
  X(FMW_MessageType_None)                       \
  X(FMW_MessageType_Response)                   \
  X(FMW_MessageType_StateChange_Init)           \
  X(FMW_MessageType_StateChange_Run)            \
  X(FMW_MessageType_Config_Robot)               \
  X(FMW_MessageType_Config_PID)                 \
  X(FMW_MessageType_Config_LED)                 \
  X(FMW_MessageType_Run_GetStatus)              \
  X(FMW_MessageType_Run_SetVelocity)            \
  X(FMW_MessageType_COUNT)

typedef uint8_t FMW_MessageType;
enum {
#define X(Variant) Variant,
  FMW_MESSAGE_TYPE_VARIANTS(X)
#undef X
};

#define FMW_RESULT_VARIANTS(X)                                          \
  X(FMW_Result_Ok)                                                      \
  X(FMW_Result_Error_InvalidArguments)                                  \
  X(FMW_Result_Error_UART_Crc)                                          \
  X(FMW_Result_Error_UART_NegativeTimeout)                              \
  X(FMW_Result_Error_UART_ReceiveTimeoutElapsed)                        \
  X(FMW_Result_Error_UART_Parity)                                       \
  X(FMW_Result_Error_UART_Frame)                                        \
  X(FMW_Result_Error_UART_Noise)                                        \
  X(FMW_Result_Error_UART_Overrun)                                      \
  X(FMW_Result_Error_Encoder_InvalidTimer)                              \
  X(FMW_Result_Error_Encoder_NonPositiveTicksPerRevolution)             \
  X(FMW_Result_Error_Encoder_NonPositiveWheelCircumference)             \
  X(FMW_Result_Error_Encoder_GetTick)                                   \
  X(FMW_Result_Error_Buzzer_Timer)                                      \
  X(FMW_Result_Error_MessageHandler_InvalidState)                       \
  X(FMW_Result_Error_MessageHandler_Init_NonPositiveBaseline)           \
  X(FMW_Result_Error_MessageHandler_Init_NonPositiveWheelCircumference) \
  X(FMW_Result_Error_MessageHandler_Init_NonPositiveTicksPerRevolution) \
  X(FMW_Result_Error_MessageHandler_Init_NonPositiveLEDUpdatePeriod)    \
  X(FMW_Result_Error_Command_NotRecognized)                             \
  X(FMW_Result_Error_Command_NotAvailable)                              \
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
      float linear;
      float angular;
    } run_set_velocity;

    struct {
      int32_t ticks_left;
      int32_t ticks_right;
      uint16_t delta_millis;
      FMW_Result result;
    } response;
  };
} FMW_Message;
#pragma pack(pop)

#endif /* INC_COMMUNICATION_OTTO_MESSAGES_H_ */
