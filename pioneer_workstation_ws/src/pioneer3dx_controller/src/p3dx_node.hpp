#ifndef P3DX_NODE_HPP
#define P3DX_NODE_HPP

#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pioneer3dx_controller/srv/change_mode.hpp>
#include <pioneer3dx_controller/srv/config_pid.hpp>
#include <pioneer3dx_controller/srv/config_led.hpp>
#include <pioneer3dx_controller/srv/config_robot.hpp>
#include <pioneer3dx_controller/srv/emergency_mode.hpp>

extern "C" {
#include "stm32_messages.h"
}

#include "main.hpp"

enum P3DX_Cmd_ChangeMode_Kind: uint8_t {
  None = 0,
  Config = 1,
  Run = 2,
  Emergency = 3,
  COUNT = 4,
};

struct P3DX_Controller_Node: public rclcpp::Node {
  const int32_t serial_fd;
  std::mutex    serial_mutex;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                 publisher_odometry;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr            subscriber_velocity;
  rclcpp::Service<pioneer3dx_controller::srv::ChangeMode>::SharedPtr    service_change_mode;
  rclcpp::Service<pioneer3dx_controller::srv::ConfigPid>::SharedPtr     service_config_pid;
  rclcpp::Service<pioneer3dx_controller::srv::ConfigRobot>::SharedPtr   service_config_robot;
  rclcpp::Service<pioneer3dx_controller::srv::ConfigLed>::SharedPtr     service_config_led;
  rclcpp::Service<pioneer3dx_controller::srv::EmergencyMode>::SharedPtr service_emergency_mode;
  rclcpp::TimerBase::SharedPtr                                          timer_publisher_odometry;

  P3DX_Controller_Node(int32_t serial_fd);

  void          callback_publish_odometry(void);
  void          callback_subscribe_command_velocity(const geometry_msgs::msg::Twist::SharedPtr cmd);
  static void   callback_service_command_change_mode_s(const std::shared_ptr<pioneer3dx_controller::srv::ChangeMode::Request> request,
                                                       std::shared_ptr<pioneer3dx_controller::srv::ChangeMode::Response> response);
  static void   callback_service_command_config_pid_s(const std::shared_ptr<pioneer3dx_controller::srv::ConfigPid::Request> request,
                                                      std::shared_ptr<pioneer3dx_controller::srv::ConfigPid::Response> response);
  static void   callback_service_command_config_robot_s(const std::shared_ptr<pioneer3dx_controller::srv::ConfigRobot::Request> request,
                                                        std::shared_ptr<pioneer3dx_controller::srv::ConfigRobot::Response> response);
  static void   callback_service_command_config_led_s(const std::shared_ptr<pioneer3dx_controller::srv::ConfigLed::Request> request,
                                                      std::shared_ptr<pioneer3dx_controller::srv::ConfigLed::Response> response);
  static void   callback_service_command_emergency_mode_s(const std::shared_ptr<pioneer3dx_controller::srv::EmergencyMode::Request> request,
                                                          std::shared_ptr<pioneer3dx_controller::srv::EmergencyMode::Response> response);

  FMW_Message   stm32_message_send(const FMW_Message *msg);
  void          stm32_message_print(const FMW_Message *msg);
};

extern std::shared_ptr<P3DX_Controller_Node> p3dx_controller;

#endif
