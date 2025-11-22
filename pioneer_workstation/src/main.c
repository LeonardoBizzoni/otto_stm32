#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <assert.h>

#include "p3dx/p3dx_messages.h"

// TODO(lb): implement CRC

int main(void) {
  int fd = open("/dev/ttyACM0", O_RDWR);
  assert(fd);

#if 0
  P3DX_Message msg_config_robot = {
    .header = {
      .type = P3DX_MessageType_Config_Robot,
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
  write(fd, &msg_config_robot, sizeof(P3DX_Message));

  P3DX_Message msg_config_pid = {
    .header = {
      .type = P3DX_MessageType_Config_PID,
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
  write(fd, &msg_config_pid, sizeof(P3DX_Message));

  P3DX_Message msg_run = {
    .header = {
      .type = P3DX_MessageType_Run,
      .crc = (uint32_t)-1,
    },
  };
  write(fd, &msg_run, sizeof(P3DX_Message));
#else
  P3DX_Message msgs[3] = {
    {
      .header = {
        .type = P3DX_MessageType_Config_Robot,
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
        .type = P3DX_MessageType_Config_PID,
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
        .type = P3DX_MessageType_Run,
        .crc = (uint32_t)-1,
      },
    },
  };
  write(fd, &msgs, 3 * sizeof(P3DX_Message));
#endif

  close(fd);
}
