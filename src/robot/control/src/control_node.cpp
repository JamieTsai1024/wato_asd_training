#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger(), this)) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1)
  );
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odometryCallback, this, std::placeholders::_1)
  );
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  control_.updatePath(msg);
}

void ControlNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  control_.updateOdometry(msg);
}

void ControlNode::controlLoop() {
  geometry_msgs::msg::Twist cmd_vel = control_.controlLoop();
  cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
