#include "p3dx_node.hpp"

#ifdef assert
#  undef assert
#endif
#define assert(cond)                    \
  do {                                  \
    if (!(cond)) { __builtin_trap(); }  \
  } while (false)

using namespace std::chrono_literals;

P3DX_Controller_Node::P3DX_Controller_Node(int32_t serial_fd) :
  rclcpp::Node("P3DX_Controller_Node"),
  serial_fd(serial_fd),
  stm32_config_mode(false),
  publisher_odometry(this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10)),
  subscriber_velocity(this->create_subscription<geometry_msgs::msg::Twist>("p3dx_command_velocity", 10,
                                                                           std::bind(&P3DX_Controller_Node::callback_subscribe_command_velocity,
                                                                                     this, std::placeholders::_1))),
  service_change_mode(this->create_service<pioneer3dx_controller::srv::ChangeMode>("p3dx_command_change_mode",
                                                                                   &P3DX_Controller_Node::callback_service_command_change_mode_s)),
  service_config_pid(this->create_service<pioneer3dx_controller::srv::ConfigPid>("p3dx_command_config_pid",
                                                                                 &P3DX_Controller_Node::callback_service_command_config_pid_s)),
  service_config_robot(this->create_service<pioneer3dx_controller::srv::ConfigRobot>("p3dx_command_config_robot",
                                                                                     &P3DX_Controller_Node::callback_service_command_config_robot_s)),
  service_config_led(this->create_service<pioneer3dx_controller::srv::ConfigLed>("p3dx_command_config_led",
                                                                                 &P3DX_Controller_Node::callback_service_command_config_led_s)),
  timer_publisher_odometry(this->create_wall_timer(50ms, std::bind(&P3DX_Controller_Node::callback_publish_odometry, this)))
{
  assert(this->serial_fd != -1);
  this->declare_parameter("frame_id_odometry", std::string("p3dx_controller_odometry"));
  RCLCPP_INFO(this->get_logger(), "ros2 node ready");
}

void P3DX_Controller_Node::callback_publish_odometry(void)
{
  if (this->stm32_config_mode) { return; }

  FMW_Message response, msg_get_status;
  msg_get_status.header.type = FMW_MessageType_Run_GetStatus;
  msg_get_status.header.crc = (uint32_t)-1;
  this->serial_mutex.lock();
  ssize_t bytes_written = ::write(this->serial_fd, &msg_get_status, sizeof(FMW_Message));
  assert(bytes_written != -1);
  assert(bytes_written == sizeof(msg_get_status));
  for (uint32_t bytes_read = 0; bytes_read < sizeof(response);) {
    ssize_t n = ::read(this->serial_fd, ((uint8_t*)&response) + bytes_read, sizeof(response) - bytes_read);
    if (n >= 0) { bytes_read += (uint32_t)n; }
    assert(n != -1);
  }
  this->serial_mutex.unlock();
  assert(response.header.type == FMW_MessageType_Response);
  this->stm32_message_print(&response);
  if (response.response.result == FMW_Result_Error_Command_NotAvailable) {
    this->stm32_config_mode = true;
    return;
  } else {
    this->stm32_config_mode = false;
  }

  nav_msgs::msg::Odometry output;
  output.header.frame_id = this->get_parameter("frame_id_odometry").as_string();
  output.header.stamp = rclcpp::Time(0, 0, this->get_clock()->now().get_clock_type());

  // output.pose.pose.position.x = ;    // TODO(lb): fill with pioneer data
  // output.pose.pose.position.y = ;    // TODO(lb): fill with pioneer data
  // output.pose.pose.orientation.x = ; // TODO(lb): fill with pioneer data
  // output.pose.pose.orientation.y = ; // TODO(lb): fill with pioneer data

  // output.twist.twist.linear.x = ;  // TODO(lb): fill with pioneer data
  // output.twist.twist.linear.y = ;  // TODO(lb): fill with pioneer data
  // output.twist.twist.angular.x = ; // TODO(lb): fill with pioneer data
  // output.twist.twist.angular.y = ; // TODO(lb): fill with pioneer data

  assert(output.pose.pose.position.z == 0);
  assert(output.pose.pose.orientation.z == 0);
  assert(output.pose.pose.orientation.w == 1);
  assert(output.twist.twist.linear.z == 0);
  assert(output.twist.twist.angular.z == 0);

  this->publisher_odometry->publish(output);
}

void P3DX_Controller_Node::callback_subscribe_command_velocity(const geometry_msgs::msg::Twist::SharedPtr cmd)
{
  (void)cmd;
}

void P3DX_Controller_Node::callback_service_command_change_mode_s(const std::shared_ptr<pioneer3dx_controller::srv::ChangeMode::Request> request,
                                                                  std::shared_ptr<pioneer3dx_controller::srv::ChangeMode::Response> response)
{
  RCLCPP_INFO(p3dx_controller->get_logger(), "Requested mode: %d", request->mode);
  P3DX_Cmd_ChangeMode_Kind mode = (P3DX_Cmd_ChangeMode_Kind)request->mode;
  if (mode == P3DX_Cmd_ChangeMode_Kind::None || mode >= P3DX_Cmd_ChangeMode_Kind::COUNT) {
    RCLCPP_INFO(p3dx_controller->get_logger(), "invalid mode received");
    response->status_response = (uint8_t)FMW_Result_Error_Command_NotRecognized;
  } else {
    FMW_MessageType type;
    switch (mode) {
    case P3DX_Cmd_ChangeMode_Kind::Config: {
      RCLCPP_INFO(p3dx_controller->get_logger(), "switching to config mode");
      type = FMW_MessageType_ModeChange_Config;
    } break;
    case P3DX_Cmd_ChangeMode_Kind::Run: {
      RCLCPP_INFO(p3dx_controller->get_logger(), "switching to run mode");
      type = FMW_MessageType_ModeChange_Run;
    } break;
    default: {
      assert(false && "unreachable");
    } break;
    }

    FMW_Message msg;
    ::memset(&msg, 0, sizeof(msg));
    msg.header.type = type;
    msg.header.crc = (uint32_t)-1;

    FMW_Message response_msg = p3dx_controller->stm32_message_send(&msg);
    assert(response_msg.header.type == FMW_MessageType_Response);
    response->status_response = (uint8_t)response_msg.response.result;
    if (response_msg.response.result == FMW_Result_Ok) {
      p3dx_controller->stm32_config_mode = (mode == P3DX_Cmd_ChangeMode_Kind::Config);
    }
  }
}

void P3DX_Controller_Node::callback_service_command_config_pid_s(const std::shared_ptr<pioneer3dx_controller::srv::ConfigPid::Request> request,
                                                                 std::shared_ptr<pioneer3dx_controller::srv::ConfigPid::Response> response)
{
  RCLCPP_INFO(p3dx_controller->get_logger(), "\nleft: {%f, %f, %f}"
                                             "\nright: {%f, %f, %f}"
                                             "\ncross: {%f, %f, %f}",
              request->left[0], request->left[1], request->left[2],
              request->right[0], request->right[1], request->right[2],
              request->cross[0], request->cross[1], request->cross[2]);

  FMW_Message msg;
  ::memset(&msg, 0, sizeof(msg));
  msg.header.type = FMW_MessageType_Config_PID;
  msg.header.crc = (uint32_t)-1;
  ::memcpy(msg.config_pid.left.values, request->left.data(), sizeof(msg.config_pid.left.values));
  ::memcpy(msg.config_pid.right.values, request->right.data(), sizeof(msg.config_pid.left.values));
  ::memcpy(msg.config_pid.cross.values, request->cross.data(), sizeof(msg.config_pid.left.values));

  FMW_Message response_msg = p3dx_controller->stm32_message_send(&msg);
  assert(response_msg.header.type == FMW_MessageType_Response);
  response->status_response = (uint8_t)response_msg.response.result;
}

void P3DX_Controller_Node::callback_service_command_config_robot_s(const std::shared_ptr<pioneer3dx_controller::srv::ConfigRobot::Request> request,
                                                                   std::shared_ptr<pioneer3dx_controller::srv::ConfigRobot::Response> response)
{
  RCLCPP_INFO(p3dx_controller->get_logger(), "\nbaseline: %f"
                                             "\nwheel_circumference_left: %f"
                                             "\nwheel_circumference_right: %f"
                                             "\nticks_per_revolution_left: %u"
                                             "\nticks_per_revolution_right: %u",
              request->baseline, request->wheel_circumference_left, request->wheel_circumference_right,
              request->ticks_per_revolution_left, request->ticks_per_revolution_right);

  FMW_Message msg;
  ::memset(&msg, 0, sizeof(msg));
  msg.header.type = FMW_MessageType_Config_Robot;
  msg.header.crc = (uint32_t)-1;
  msg.config_robot.baseline = request->baseline;
  msg.config_robot.wheel_circumference_left = request->wheel_circumference_left;
  msg.config_robot.wheel_circumference_right = request->wheel_circumference_right;
  msg.config_robot.ticks_per_revolution_left = request->ticks_per_revolution_left;
  msg.config_robot.ticks_per_revolution_right = request->ticks_per_revolution_right;

  FMW_Message response_msg = p3dx_controller->stm32_message_send(&msg);
  assert(response_msg.header.type == FMW_MessageType_Response);
  response->status_response = (uint8_t)response_msg.response.result;
}

void P3DX_Controller_Node::callback_service_command_config_led_s(const std::shared_ptr<pioneer3dx_controller::srv::ConfigLed::Request> request,
                                                                 std::shared_ptr<pioneer3dx_controller::srv::ConfigLed::Response> response)
{
  RCLCPP_INFO(p3dx_controller->get_logger(), "\nvoltage_red: %f"
                                             "\nvoltage_orange: %f"
                                             "\nvoltage_hysteresis: %f"
                                             "\nupdate_period: %u",
              request->voltage_red, request->voltage_orange, request->voltage_hysteresis, request->update_period);

  FMW_Message msg;
  ::memset(&msg, 0, sizeof(msg));
  msg.header.type = FMW_MessageType_Config_LED;
  msg.header.crc = (uint32_t)-1;
  msg.config_led.voltage_red = request->voltage_red;
  msg.config_led.voltage_orange = request->voltage_orange;
  msg.config_led.voltage_hysteresis = request->voltage_hysteresis;
  msg.config_led.update_period = request->update_period;

  FMW_Message response_msg = p3dx_controller->stm32_message_send(&msg);
  assert(response_msg.header.type == FMW_MessageType_Response);
  response->status_response = (uint8_t)response_msg.response.result;
}

FMW_Message P3DX_Controller_Node::stm32_message_send(const FMW_Message *msg)
{
  FMW_Message res;
  ::memset(&res, 0, sizeof(res));

  assert(msg != NULL);
  assert(this->serial_fd > 0);
  this->serial_mutex.lock();
  ssize_t bytes_written = ::write(this->serial_fd, msg, sizeof(FMW_Message));
  assert(bytes_written != -1);
  assert(bytes_written > 0);
  assert(bytes_written == sizeof(FMW_Message));
  RCLCPP_INFO(this->get_logger(), "message sent to stm32");
  for (uint32_t bytes_read = 0; bytes_read < sizeof(*msg);) {
    ssize_t n = ::read(this->serial_fd, ((uint8_t*)&res) + bytes_read, sizeof(res) - bytes_read);
    assert(n != -1);
    if (n >= 0) { bytes_read += (uint32_t)n; }
  }
  this->serial_mutex.unlock();
  this->stm32_message_print(&res);
  assert(res.header.type == FMW_MessageType_Response);
  return res;
}

void P3DX_Controller_Node::stm32_message_print(const FMW_Message *msg)
{
  static const char *message_types[] = {
    #define X(Variant, Id) #Variant,
    FMW_MESSAGE_TYPE_VARIANTS()
    #undef X
  };

  static const char *result_types[] = {
    #define X(Variant, Id) #Variant,
    FMW_RESULT_VARIANTS()
    #undef X
  };

  char buffer[512] = {0};
  int32_t buffer_len = snprintf(buffer, sizeof(buffer),
                                "FMW_Message {"
                                "\n  header.type = %s"
                                "\n  header.crc  = %u",
                                message_types[msg->header.type], msg->header.crc);
  assert(buffer_len > 0);
  assert(buffer_len < (int32_t)sizeof(buffer));

  switch (msg->header.type) {
  case FMW_MessageType_Response: {
    buffer_len += snprintf(buffer + buffer_len, sizeof(buffer) - buffer_len,
                           "\n  response:"
                           "\n    result       = %s"
                           "\n    delta_millis = %d"
                           "\n    ticks_left   = %d"
                           "\n    ticks_right  = %d",
                           result_types[msg->response.result], msg->response.delta_millis,
                           msg->response.ticks_left, msg->response.ticks_right);
    assert(buffer_len < (int32_t)sizeof(buffer));
  } break;
  default: {
    assert(false && "unreachable");
  } break;
  }
  RCLCPP_INFO(this->get_logger(), "%s\n}", buffer);
}
