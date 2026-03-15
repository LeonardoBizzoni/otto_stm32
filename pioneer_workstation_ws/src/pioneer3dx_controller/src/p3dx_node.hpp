#ifndef P3DX_NODE_HPP
#define P3DX_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pioneer3dx_controller/srv/change_mode.hpp>
#include <pioneer3dx_controller/srv/config_pid.hpp>
#include <pioneer3dx_controller/srv/config_led.hpp>
#include <pioneer3dx_controller/srv/config_robot.hpp>

extern "C" {
#include "stm32_messages.h"
}

#include "main.hpp"

enum P3DX_Cmd_ChangeMode_Kind: uint8_t {
  None = 0,
  Config = 1,
  Run = 2,
  COUNT = 3,
};

struct P3DX_Controller_Node: public rclcpp::Node {
  const int32_t serial_fd;
  bool stm32_running;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                 publisher_odometry;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr            subscriber_velocity;
  rclcpp::Service<pioneer3dx_controller::srv::ChangeMode>::SharedPtr    service_change_mode;
  rclcpp::Service<pioneer3dx_controller::srv::ConfigPid>::SharedPtr     service_config_pid;
  rclcpp::Service<pioneer3dx_controller::srv::ConfigLed>::SharedPtr     service_config_led;
  rclcpp::Service<pioneer3dx_controller::srv::ConfigRobot>::SharedPtr   service_config_robot;
  rclcpp::TimerBase::SharedPtr                                          timer_publisher_odometry;

  P3DX_Controller_Node(void);

  void callback_publish_odometry(void);
  void callback_subscribe_command_velocity(const geometry_msgs::msg::Twist::SharedPtr cmd);
  static void callback_service_command_change_mode_s(const std::shared_ptr<pioneer3dx_controller::srv::ChangeMode::Request> request,
                                                     std::shared_ptr<pioneer3dx_controller::srv::ChangeMode::Response> response);

  void stm32_message_print(const FMW_Message *msg);
};

extern std::shared_ptr<P3DX_Controller_Node> p3dx_controller;

#endif
