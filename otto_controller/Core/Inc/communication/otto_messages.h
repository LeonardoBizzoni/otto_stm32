/*
 * otto_messages.h
 *
 *  Created on: Aug 5, 2021
 *      Author: fdila
 */

#ifndef INC_COMMUNICATION_OTTO_MESSAGES_H_
#define INC_COMMUNICATION_OTTO_MESSAGES_H_

typedef uint16_t MessageStatusCode;
enum {
  MessageStatusCode_Waiting4Config = 0,
  MessageStatusCode_Running        = 1,
  MessageStatusCode_Error_Config   = 2,
  MessageStatusCode_Error_Velocity = 3,
  MessageStatusCode_Fault_HBridge  = 4,
};

typedef uint8_t OttoMessageType;
enum {
  OttoMessageType_Invalid = 0,
  OttoMessageType_Error,

  OttoMessageType_Run,
  OttoMessageType_Config,
  OttoMessageType_Status,
  OttoMessageType_Velocity,

  OttoMessageType_COUNT,
};

typedef uint8_t OttoMessageError;
enum {
  OttoMessageError_Unknown = 0,

  OttoMessageError_UART_Crc,
  OttoMessageError_UART_ReceiveTimeoutElapsed,

  OttoMessageError_Command_NotRecognized,
  OttoMessageError_Command_NotAvailable,

  OttoMessageError_COUNT,
};

typedef struct {
  struct {
    uint32_t crc;
    OttoMessageType type;
  } header;

  union {
    struct {
      OttoMessageError reason;
    } error;
    struct {
      // NOTE(lb): Why do we need to receive the wheels circumference and baseline?
      //           Also why BOTH wheels circumference? Aren't they always the same?
      //           Same for ticksXrevolution, we should already know all of this stuff.
      //           Even if this has to run on different devices its weird not to
      //             use compile time constants.
      uint32_t ticks_per_revolution;    // x4 resolution
      float baseline;                   // in meters
      float left_wheel_circumference;   // in meters
      float right_wheel_circumference;  // in meters

      PidConstants pid_ks_left;
      PidConstants pid_ks_right;
      PidConstants pid_ks_cross;
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
} OttoMessage;

void otto_report_handler(OttoMessageError error_code);

#endif /* INC_COMMUNICATION_OTTO_MESSAGES_H_ */
