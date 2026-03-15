#include "p3dx_node.hpp"

using namespace std::chrono_literals;

P3DX_Controller_Node::P3DX_Controller_Node() :
  rclcpp::Node("P3DX_Controller_Node"),
  serial_fd(serial_open("/dev/ttyACM0")),
  publisher_odometry(this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10)),
  subscriber_velocity(this->create_subscription<geometry_msgs::msg::Twist>("p3dx_command_velocity", 10,
                                                                           std::bind(&P3DX_Controller_Node::callback_subscribe_command_velocity,
                                                                                     this, std::placeholders::_1))),
  service_change_mode(this->create_service<pioneer3dx_controller::srv::ChangeMode>("p3dx_command_change_mode",
                                                                                   &P3DX_Controller_Node::callback_service_command_change_mode_s)),
  timer_publisher_odometry(this->create_wall_timer(50ms, std::bind(&P3DX_Controller_Node::callback_publish_odometry, this)))
{
  assert(this->serial_fd != -1);
  this->declare_parameter("frame_id_odometry", std::string("p3dx_controller_odometry"));
  RCLCPP_INFO(this->get_logger(), "ros2 node ready");
}

void P3DX_Controller_Node::callback_publish_odometry(void)
{
  if (!this->stm32_running) { return; }

#if 0
  FMW_Message response, msg_get_status;
  msg_get_status.header.type = FMW_MessageType_Run_GetStatus;
  msg_get_status.header.crc = (uint32_t)-1;
  size_t bytes_written = ::write(this->serial_fd, &msg_get_status, sizeof(FMW_Message));
  (void)bytes_written;
  for (uint32_t bytes_read = 0; bytes_read < sizeof(response);) {
    ssize_t n = ::read(this->serial_fd, ((uint8_t*)&response) + bytes_read, sizeof(response) - bytes_read);
    if (n >= 0) { bytes_read += (uint32_t)n; }
  }
  assert(response.header.type == FMW_MessageType_Response);
  stm32_message_print(&response);


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
#endif
}

void P3DX_Controller_Node::callback_subscribe_command_velocity(const geometry_msgs::msg::Twist::SharedPtr cmd)
{
  (void)cmd;
}

void P3DX_Controller_Node::callback_service_command_change_mode_s(const std::shared_ptr<pioneer3dx_controller::srv::ChangeMode::Request> request,
                                                                  std::shared_ptr<pioneer3dx_controller::srv::ChangeMode::Response> response)
{
  (void)response;

  RCLCPP_INFO(p3dx_controller->get_logger(), "%d", request->mode);
  P3DX_Cmd_ChangeMode_Kind mode = (P3DX_Cmd_ChangeMode_Kind)request->mode;
  if (mode == P3DX_Cmd_ChangeMode_Kind::None || mode >= P3DX_Cmd_ChangeMode_Kind::COUNT) {
    RCLCPP_INFO(p3dx_controller->get_logger(), "invalid mode received");
    response->status_response = (uint8_t)FMW_Result_Error_Command_NotRecognized;
  } else {
    switch (mode) {
    case P3DX_Cmd_ChangeMode_Kind::Config: {
      RCLCPP_INFO(p3dx_controller->get_logger(), "switching to config mode");
    } break;
    case P3DX_Cmd_ChangeMode_Kind::Run: {
      RCLCPP_INFO(p3dx_controller->get_logger(), "switching to run mode");
    } break;
    default: {
      assert(false && "unreachable");
    } break;
    }

    FMW_Message msg = p3dx_controller->stm32_message_send_change_mode(mode);
    assert(msg.header.type == FMW_MessageType_Response);
    response->status_response = (uint8_t)msg.response.result;
  }
}

FMW_Message P3DX_Controller_Node::stm32_message_send_change_mode(P3DX_Cmd_ChangeMode_Kind kind)
{
  assert(kind > P3DX_Cmd_ChangeMode_Kind::None);
  assert(kind < P3DX_Cmd_ChangeMode_Kind::COUNT);
  FMW_Message msg;
  ::memset(&msg, 0, sizeof(msg));
  msg.header.type = (kind == P3DX_Cmd_ChangeMode_Kind::Config
                     ? FMW_MessageType_ModeChange_Config
                     : FMW_MessageType_ModeChange_Run);
  msg.header.crc = (uint32_t)-1;
  this->stm32_message_print(&msg);
  size_t bytes_written = ::write(this->serial_fd, &msg, sizeof(FMW_Message));
  RCLCPP_INFO(this->get_logger(), "message sent to stm32");
  (void)bytes_written;
  ::memset(&msg, 0, sizeof(msg));
  for (uint32_t bytes_read = 0; bytes_read < sizeof(msg);) {
    ssize_t n = ::read(this->serial_fd, ((uint8_t*)&msg) + bytes_read, sizeof(msg) - bytes_read);
    if (n >= 0) { bytes_read += (uint32_t)n; }
  }
  this->stm32_message_print(&msg);
  assert(msg.header.type == FMW_MessageType_Response);
  return msg;
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
