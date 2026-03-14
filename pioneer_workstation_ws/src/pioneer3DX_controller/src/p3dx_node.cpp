#include "p3dx_node.hpp"

using namespace std::chrono_literals;

P3DX_Controller_Node::P3DX_Controller_Node() :
  rclcpp::Node("P3DX_Controller_Node"), serial_fd(serial_open("/dev/ttyACM0")),
  publisher_odometry(this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10)),
  subscriber_command_velocity(this->create_subscription<geometry_msgs::msg::Twist>("p3dx_command_velocity", 10,
                                                                                   std::bind(&P3DX_Controller_Node::callback_subscribe_command_velocity,
                                                                                             this, std::placeholders::_1))),
  publisher_odometry_timer(this->create_wall_timer(50ms, std::bind(&P3DX_Controller_Node::callback_publish_odometry, this)))
{
  this->declare_parameter("frame_id_odometry", std::string("p3dx_controller_odometry"));
  assert(this->serial_fd != -1);
}

void P3DX_Controller_Node::callback_publish_odometry(void)
{
  nav_msgs::msg::Odometry output;
  output.header.frame_id = this->get_parameter("frame_id_odometry").as_string();
  // output.header.stamp = ;            // TODO(lb): generate some kind of current timestamp

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
  RCLCPP_INFO(this->get_logger(), "Hello, World!");
}

void P3DX_Controller_Node::callback_subscribe_command_velocity(const geometry_msgs::msg::Twist::SharedPtr cmd)
{
  (void)cmd;
}
