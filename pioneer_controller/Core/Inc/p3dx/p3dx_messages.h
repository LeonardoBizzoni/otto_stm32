#ifndef P3DX_MESSAGE_H
#define P3DX_MESSAGE_H

typedef uint16_t MessageStatusCode;
enum {
  MessageStatusCode_Waiting4Config = 0,
  MessageStatusCode_Running        = 1,
  MessageStatusCode_Error_Config   = 2,
  MessageStatusCode_Error_Velocity = 3,
  MessageStatusCode_Fault_HBridge  = 4,
};

typedef uint8_t P3DX_State;
enum {
  P3DX_State_Init,
  P3DX_State_Error,
  P3DX_State_Running,
};

typedef uint8_t P3DX_MessageType;
enum {
  P3DX_MessageType_Error,

  P3DX_MessageType_Run,
  P3DX_MessageType_Config,
  P3DX_MessageType_Status,
  P3DX_MessageType_Velocity,

  P3DX_MessageType_COUNT,
};

typedef uint8_t P3DX_Error;
enum {
  P3DX_Error_Unknown = 0,

  P3DX_Error_UART_Crc,
  P3DX_Error_UART_ReceiveTimeoutElapsed,

  P3DX_Error_Command_NotRecognized,
  P3DX_Error_Command_NotAvailable,
};

typedef struct {
  struct {
    uint32_t crc;
    P3DX_MessageType type;
  } header;

  union {
    struct {
      int32_t filename_size;
      const char *filename;
      int32_t line;
      P3DX_Error reason;
    } error;

    struct {
      uint32_t ticks_per_revolution;
      float baseline;
      float left_wheel_circumference;
      float right_wheel_circumference;

      P3DX_PidConstants pid_ks_left;
      P3DX_PidConstants pid_ks_right;
      P3DX_PidConstants pid_ks_cross;
    } config;
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
} P3DX_Message;

#endif /* INC_COMMUNICATION_OTTO_MESSAGES_H_ */
