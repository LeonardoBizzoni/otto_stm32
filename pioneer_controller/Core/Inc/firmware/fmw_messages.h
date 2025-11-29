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

typedef uint16_t MessageStatusCode;
enum {
  MessageStatusCode_Waiting4Config = 0,
  MessageStatusCode_Running        = 1,
  MessageStatusCode_Error_Config   = 2,
  MessageStatusCode_Error_Velocity = 3,
  MessageStatusCode_Fault_HBridge  = 4,
};

typedef uint8_t FMW_State;
enum {
  FMW_State_Init,
  FMW_State_Error,
  FMW_State_Running,
};

typedef uint8_t FMW_MessageType;
enum {
  FMW_MessageType_Error,

  FMW_MessageType_Run,
  FMW_MessageType_Config_Robot,
  FMW_MessageType_Config_PID,
  FMW_MessageType_Config_LED,

  FMW_MessageType_Status,
  FMW_MessageType_Velocity,

  FMW_MessageType_COUNT,
};

typedef uint8_t FMW_Error;
enum {
  FMW_Error_Unknown = 0,

  FMW_Error_UART_Crc,
  FMW_Error_UART_ReceiveTimeoutElapsed,

  FMW_Error_Command_NotRecognized,
  FMW_Error_Command_NotAvailable,
};

#pragma pack(push, 1)
typedef struct {
  struct {
    FMW_MessageType type;
    uint32_t crc;
  } header;

  union {
    struct {
      FMW_Error reason;
      int32_t line;
      int32_t filename_size;
      const char *filename;
    } error;

    struct {
      float baseline;
      float left_wheel_circumference;
      uint32_t left_ticks_per_revolution;
      float right_wheel_circumference;
      uint32_t right_ticks_per_revolution;
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
      MessageStatusCode status_code;
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
