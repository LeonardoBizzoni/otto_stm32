#ifndef FMW_MESSAGE_H
#define FMW_MESSAGE_H

struct FMW_PidConstants_Fields {
  float proportional;
  float integral;
  float derivative;
};

typedef union {
  struct FMW_PidConstants_Fields fields;
  float values[3];
} FMW_PidConstants;

// NOTE(lb): The expansion of this macro relies on another macro called "X" (hence the name X-macro).
//           X isn't define yet but, for it to work with this macro expansion, it must be a function-like macro
//           with exactly 2 arguemnts: 1) the enumation variant name. 2) the corresponding numerical id.
#define FMW_MESSAGE_TYPE_VARIANTS()             \
  X(FMW_MessageType_None, 0)                    \
  X(FMW_MessageType_Response, 1)                \
  X(FMW_MessageType_ModeChange_Config, 2)       \
  X(FMW_MessageType_ModeChange_Run, 3)          \
  X(FMW_MessageType_Config_Robot, 4)            \
  X(FMW_MessageType_Config_PID, 5)              \
  X(FMW_MessageType_Config_LED, 6)              \
  X(FMW_MessageType_Run_GetStatus, 7)           \
  X(FMW_MessageType_Run_SetVelocity, 8)         \
  X(FMW_MessageType_COUNT, 9)

// NOTE(lb): Here i want to take all of the variants that are generated from the macro
//           expension of FMW_MESSAGE_TYPE_VARIANTS and use them to populate the enum definition.
//           For it to be a valid enum definition you need to take every `X(VariantName, Id)`
//           and turn them into `VariantName = Id,`.
//           It's important to undefine `X` after use to avoid redefinition and, more importantly,
//           to use it in another X-macro expansion by accident.
//           This seems pointless but when paired with `#VariantName` you can automatically
//           turn all of the symbolic variant names into strings and now printing the variant
//           can be done via an array lookup `[VariantName] = #VariantName` instead of a runtime check.
typedef uint8_t FMW_MessageType;
enum {
#define X(Variant, Id) Variant = Id,
  FMW_MESSAGE_TYPE_VARIANTS()
#undef X
};

#define FMW_RESULT_VARIANTS()                                                   \
  X(FMW_Result_Ok, 0)                                                           \
  X(FMW_Result_Error_InvalidArguments, 1)                                       \
  X(FMW_Result_Error_FaultPinTriggered, 2)                                      \
  X(FMW_Result_Error_UART_Crc, 3)                                               \
  X(FMW_Result_Error_UART_NegativeTimeout, 4)                                   \
  X(FMW_Result_Error_UART_ReceiveTimeoutElapsed, 5)                             \
  X(FMW_Result_Error_UART_Parity, 6)                                            \
  X(FMW_Result_Error_UART_Frame, 7)                                             \
  X(FMW_Result_Error_UART_Noise, 8)                                             \
  X(FMW_Result_Error_UART_Overrun, 9)                                           \
  X(FMW_Result_Error_Encoder_InvalidTimer, 10)                                  \
  X(FMW_Result_Error_Encoder_NonPositiveTicksPerRevolution, 11)                 \
  X(FMW_Result_Error_Encoder_NonPositiveWheelCircumference, 12)                 \
  X(FMW_Result_Error_Encoder_GetTick, 13)                                       \
  X(FMW_Result_Error_Buzzer_Timer, 14)                                          \
  X(FMW_Result_Error_MessageHandler_InvalidState, 15)                           \
  X(FMW_Result_Error_MessageHandler_Config_NonPositiveBaseline, 16)             \
  X(FMW_Result_Error_MessageHandler_Config_NonPositiveWheelCircumference, 17)   \
  X(FMW_Result_Error_MessageHandler_Config_NonPositiveTicksPerRevolution, 18)   \
  X(FMW_Result_Error_MessageHandler_Config_NonPositiveLEDUpdatePeriod, 19)      \
  X(FMW_Result_Error_Command_NotRecognized, 20)                                 \
  X(FMW_Result_Error_Command_NotAvailable, 21)                                  \
  X(FMW_Result_COUNT, 22)

typedef uint8_t FMW_Result;
enum {
#define X(Variant, Id) Variant = Id,
  FMW_RESULT_VARIANTS()
#undef X
};

#pragma pack(push, 1)
typedef struct FMW_Message {
  struct FMW_Message_Header {
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

    // TODO(lb): add odometry information
    struct {
      int32_t ticks_left;
      int32_t ticks_right;
      uint16_t delta_millis;
      FMW_Result result;
    } response;
  };
} FMW_Message;
#pragma pack(pop)

static_assert(sizeof(uint8_t)   == 1);
static_assert(sizeof(uint16_t)  == 2);
static_assert(sizeof(uint32_t)  == 4);
static_assert(sizeof(float)     == 4);

#endif /* INC_COMMUNICATION_OTTO_MESSAGES_H_ */
