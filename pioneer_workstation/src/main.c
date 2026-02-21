#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <assert.h>

#include "firmware/fmw_messages.h"

// TODO(lb): implement CRC

static int serial_open(char *portname) {
  int fd = open(portname, O_RDWR | O_NOCTTY);
  assert(fd >= 0);

  struct termios tty;
  tcgetattr(fd, &tty);

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cflag &= (tcflag_t)~PARENB;
  tty.c_cflag &= (tcflag_t)~CSTOPB;
  tty.c_cflag &= (tcflag_t)~CRTSCTS;
  tty.c_cflag |= CS8 | CLOCAL | CREAD;

  tty.c_lflag = 0;
  tty.c_iflag = 0;
  tty.c_oflag = 0;

  tty.c_cc[VMIN]  = 1;
  tty.c_cc[VTIME] = 0;

  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &tty);

  return fd;
}

static void pprint_message(const FMW_Message *msg) {
  static const char *message_types[] = {
#define X(Variant) #Variant,
    FMW_MESSAGE_TYPE_VARIANTS(X)
#undef X
  };

  static const char *result_types[] = {
#define X(Variant) #Variant,
    FMW_RESULT_VARIANTS(X)
#undef X
  };

  printf("FMW_Message {"
         "\n  header.type = %s"
         "\n  header.crc  = %u",
         message_types[msg->header.type],
         msg->header.crc);
  switch (msg->header.type) {
  case FMW_MessageType_Response: {
    printf("\n  response: %s", result_types[msg->response]);
  } break;
  default: {
    assert(0 && "unreachable");
  } break;
  }
  printf("\n}\n");
}

int main(void) {
  int fd = serial_open("/dev/ttyACM0");
  assert(fd);

  FMW_Message response = {0};

  FMW_Message msg_config_robot = {
    .header = {
      .type = FMW_MessageType_Config_Robot,
      .crc = (uint32_t)-1,
    },
    .config_robot = {
      .baseline = 2.3f,
      .wheel_circumference_left = 4.f,
      .wheel_circumference_right = 3.2f,
      .ticks_per_revolution_left = 250,
      .ticks_per_revolution_right = 350,
    },
  };
  write(fd, &msg_config_robot, sizeof(FMW_Message));
  for (uint32_t bytes_read = 0; bytes_read < sizeof(response);) {
    ssize_t n = read(fd, ((uint8_t*)&response) + bytes_read, sizeof(response) - bytes_read);
    if (n >= 0) { bytes_read += (uint32_t)n; }
  }
  assert(response.header.type == FMW_MessageType_Response);
  pprint_message(&response);
  assert(response.response == FMW_Result_Ok);

  FMW_Message msg_config_pid = {
    .header = {
      .type = FMW_MessageType_Config_PID,
      .crc = (uint32_t)-1,
    },
    .config_pid = {
      .left = {
        .proportional = 0.2f,
        .integral = 0.2f,
        .derivative = 0.2f,
      },
      .right = {
        .proportional = 0.3f,
        .integral = 0.3f,
        .derivative = 0.3f,
      },
      .cross = {
        .proportional = 0.4f,
        .integral = 0.4f,
        .derivative = 0.4f,
      },
    },
  };
  write(fd, &msg_config_pid, sizeof(FMW_Message));
  for (uint32_t bytes_read = 0; bytes_read < sizeof(response);) {
    ssize_t n = read(fd, ((uint8_t*)&response) + bytes_read, sizeof(response) - bytes_read);
    if (n >= 0) { bytes_read += (uint32_t)n; }
  }
  assert(response.header.type == FMW_MessageType_Response);
  pprint_message(&response);
  assert(response.response == FMW_Result_Ok);

  FMW_Message msg_run = {
    .header = {
      .type = FMW_MessageType_StateChange_Run,
      .crc = (uint32_t)-1,
    },
  };
  write(fd, &msg_run, sizeof(FMW_Message));
  for (uint32_t bytes_read = 0; bytes_read < sizeof(response);) {
    ssize_t n = read(fd, ((uint8_t*)&response) + bytes_read, sizeof(response) - bytes_read);
    if (n >= 0) { bytes_read += (uint32_t)n; }
  }
  assert(response.header.type == FMW_MessageType_Response);
  pprint_message(&response);
  assert(response.response == FMW_Result_Ok);

  close(fd);
}
