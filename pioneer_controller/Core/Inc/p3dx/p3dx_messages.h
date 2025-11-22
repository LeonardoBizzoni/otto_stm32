#ifndef P3DX_MESSAGE_H
#define P3DX_MESSAGE_H

typedef union {
  struct {
    float proportional;
    float integral;
    float derivative;
  };
  float values[3];
} P3DX_PidConstants;

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
  P3DX_MessageType_Config_Robot,
  P3DX_MessageType_Config_PID,


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

#pragma pack(push, 1)
typedef struct {
  struct {
    P3DX_MessageType type;
    uint32_t crc;
  } header;

  union {
    struct {
      P3DX_Error reason;
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
      P3DX_PidConstants left;
      P3DX_PidConstants right;
      P3DX_PidConstants cross;
    } config_pid;

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
#pragma pack(pop)

#endif /* INC_COMMUNICATION_OTTO_MESSAGES_H_ */
