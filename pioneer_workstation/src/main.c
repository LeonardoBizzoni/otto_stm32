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
  int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  assert(fd >= 0);
  tcflush(fd, TCIOFLUSH);

  struct termios tty;
  assert(!tcgetattr(fd, &tty));
  cfsetospeed(&tty, B115200); // output baud rate
  cfsetispeed(&tty, B115200); //  input baud rate
  tty.c_cflag = (tty.c_cflag & (tcflag_t)~CSIZE) | CS8; // 8-bit chars
  tty.c_iflag &= (tcflag_t)~IGNBRK;                     // disable break processing
  tty.c_cc[VMIN]  = 0;                                  // read doesn't block
  tty.c_cc[VTIME] = 20;                                 // 2.0 seconds read timeout
  tty.c_cflag &= (tcflag_t)~(PARENB | PARODD);          // no parity
  tty.c_cflag &= (tcflag_t)~CSTOPB;                     // 1 stop bit
  assert(!tcsetattr(fd, TCSANOW, &tty));
  return fd;
}

int main(void) {
  int fd = serial_open("/dev/ttyACM0");
  assert(fd);

#if 1
  FMW_Message msg_config_robot = {
    .header = {
      .type = FMW_MessageType_Config_Robot,
      .crc = (uint32_t)-1,
    },
    .config_robot = {
      .baseline = 2.3f,
      .left_wheel_circumference = 4.f,
      .left_ticks_per_revolution = 250,
      .right_wheel_circumference = 3.2f,
      .right_ticks_per_revolution = 350,
    },
  };
  write(fd, &msg_config_robot, sizeof(FMW_Message));

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

  FMW_Message msg_run = {
    .header = {
      .type = FMW_MessageType_Run,
      .crc = (uint32_t)-1,
    },
  };
  write(fd, &msg_run, sizeof(FMW_Message));
#else
  FMW_Message msgs[3] = {
    {
      .header = {
        .type = FMW_MessageType_Config_Robot,
        .crc = (uint32_t)-1,
      },
      .config_robot = {
        .baseline = 2.3f,
        .left_wheel_circumference = 4.f,
        .left_ticks_per_revolution = 250,
        .right_wheel_circumference = 3.2f,
        .right_ticks_per_revolution = 350,
      },
    },
    {
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
    },
    {
      .header = {
        .type = FMW_MessageType_Run,
        .crc = (uint32_t)-1,
      },
    },
  };
  write(fd, &msgs, 3 * sizeof(FMW_Message));
#endif

  close(fd);
}
