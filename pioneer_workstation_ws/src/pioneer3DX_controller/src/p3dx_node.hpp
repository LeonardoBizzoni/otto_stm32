#ifndef P3DX_NODE_HPP
#define P3DX_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

extern "C" {
#include "stm32_messages.h"
}

#include "main.hpp"

struct P3DX_Controller_Node: public rclcpp::Node {
  const int32_t serial_fd;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odometry;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_command_velocity;
  rclcpp::TimerBase::SharedPtr publisher_odometry_timer;
  bool stm32_running;

  P3DX_Controller_Node(void);

  void callback_publish_odometry(void);
  void callback_subscribe_command_velocity(const geometry_msgs::msg::Twist::SharedPtr cmd);

  void stm32_message_print(const FMW_Message *msg);
};


#endif
