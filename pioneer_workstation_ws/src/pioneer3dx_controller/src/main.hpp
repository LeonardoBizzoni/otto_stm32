#ifndef MAIN_HPP
#define MAIN_HPP

#include <termios.h>
#include <fcntl.h>

[[nodiscard]] int32_t serial_open(const char *portname);

#endif
