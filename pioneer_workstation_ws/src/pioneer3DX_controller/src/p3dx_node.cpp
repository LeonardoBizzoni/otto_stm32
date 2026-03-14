#include "p3dx_node.hpp"

using namespace std::chrono_literals;

P3DX_Controller_Node::P3DX_Controller_Node() :
  rclcpp::Node("P3DX_Controller_Node"), serial_fd(serial_open("/dev/ttyACM0")),
  publisher_odometry(this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10)),
  subscriber_command_velocity(this->create_subscription<geometry_msgs::msg::Twist>("p3dx_command_velocity", 10,
                                                                                   std::bind(&P3DX_Controller_Node::callback_subscribe_command_velocity,
                                                                                             this, std::placeholders::_1))),
  publisher_odometry_timer(this->create_wall_timer(50ms, std::bind(&P3DX_Controller_Node::callback_publish_odometry, this))),
  stm32_running(false)
{
  assert(this->serial_fd != -1);

  this->declare_parameter("frame_id_odometry", std::string("p3dx_controller_odometry"));

  // TODO(lb): force the switch to config mode maybe?
  //           switching to run mode shouldn't happen now, this is just for testing
  FMW_Message response, msg_run;
  msg_run.header.type = FMW_MessageType_ModeChange_Run;
  msg_run.header.crc = (uint32_t)-1;
  (void)::write(this->serial_fd, &msg_run, sizeof(FMW_Message));
  for (uint32_t bytes_read = 0; bytes_read < sizeof(response);) {
    ssize_t n = ::read(this->serial_fd, ((uint8_t*)&response) + bytes_read, sizeof(response) - bytes_read);
    if (n >= 0) { bytes_read += (uint32_t)n; }
  }
  stm32_message_print(&response);
  assert(response.header.type == FMW_MessageType_Response);
  assert(response.response.result == FMW_Result_Ok || response.response.result == FMW_Result_Error_Command_NotAvailable);
  this->stm32_running = true;
}

void P3DX_Controller_Node::callback_publish_odometry(void)
{
  if (!this->stm32_running) { return; }

  FMW_Message response, msg_get_status;
  msg_get_status.header.type = FMW_MessageType_Run_GetStatus;
  msg_get_status.header.crc = (uint32_t)-1;
  (void)::write(this->serial_fd, &msg_get_status, sizeof(FMW_Message));
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
}

void P3DX_Controller_Node::callback_subscribe_command_velocity(const geometry_msgs::msg::Twist::SharedPtr cmd)
{
  (void)cmd;
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
  assert(buffer_len < sizeof(buffer));

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
    assert(buffer_len < sizeof(buffer));
  } break;
  default: {
    assert(false && "unreachable");
  } break;
  }
  RCLCPP_INFO(this->get_logger(), "%s\n}", buffer);
}
