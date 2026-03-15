#include <cstdint>
#include <cstdio>

#include "main.hpp"
#include "p3dx_node.hpp"

std::shared_ptr<P3DX_Controller_Node> p3dx_controller = nullptr;

int32_t main(int32_t argc, const char *argv[])
{
  int32_t serial_fd = serial_open("/dev/ttyACM0");
  assert(serial_fd != -1);

  rclcpp::init(argc, argv);
  p3dx_controller = std::make_shared<P3DX_Controller_Node>(serial_fd);
  assert(p3dx_controller != nullptr);
  rclcpp::spin(p3dx_controller);
  rclcpp::shutdown();
  close(serial_fd);
}

[[nodiscard]] int32_t serial_open(const char *portname)
{
  int32_t fd = open(portname, O_RDWR | O_NOCTTY);
  assert(fd >= 0);

  struct termios tty = {};
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

  tcflush(fd, TCIOFLUSH);
  tcsetattr(fd, TCSANOW, &tty);

  return fd;
}
