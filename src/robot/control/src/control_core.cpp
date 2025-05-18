#include "control_core.hpp"

namespace robot {

ControlCore::ControlCore(const rclcpp::Logger& logger, rclcpp::Node* node) : logger_(logger) {
  lookahead_distance_ = node->declare_parameter<double>("lookahead_distance", 1.0);
  goal_tolerance_ = node->declare_parameter<double>("goal_tolerance", 0.1);
  linear_speed_ = node->declare_parameter<double>("linear_speed", 0.5);
}

void ControlCore::updatePath(const nav_msgs::msg::Path::SharedPtr path) {
  current_path_ = path;
}

void ControlCore::updateOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry) {
  robot_odometry_ = odometry;
}

std::optional<geometry_msgs::msg::Twist> ControlCore::controlLoop() {
  // Skip control if no path or odometry data is available
  if (!current_path_ || !robot_odometry_) return std::nullopt;

  // Find the lookahead point
  auto target = findLookaheadPoint();
  if (!target) return std::nullopt;

  // Compute velocity command
  geometry_msgs::msg::Twist cmd = computeVelocity(*target);
  return cmd; 
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint() {
  // Logic to find the lookahead point on the path
  const auto& robot_pos = robot_odometry_->pose.pose.position;
  for (const auto& pose : current_path_->poses) {
    if (computeDistance(robot_pos, pose.pose.position) >= lookahead_distance_) {
      return pose;
    }
  }
  return std::nullopt;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const geometry_msgs::msg::PoseStamped& target) {
  // Logic to compute velocity commands
  geometry_msgs::msg::Twist cmd_vel;

  const auto& robot_pose = robot_odometry_->pose.pose;
  double robot_yaw = extractYaw(robot_pose.orientation);

  double dx = target.pose.position.x - robot_pose.position.x;
  double dy = target.pose.position.y - robot_pose.position.y;

  double target_angle = std::atan2(dy, dx);
  double angle_error = target_angle - robot_yaw;

  // Normalize angle to [-π, π]
  while (angle_error > M_PI) angle_error -= 2 * M_PI;
  while (angle_error < -M_PI) angle_error += 2 * M_PI;

  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = 2.0 * angle_error; 

  return cmd_vel;
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  // Distance calculation between two points
  return std::hypot(b.x - a.x, b.y - a.y);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat) {
  // Quaternion to yaw conversion
  tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
  return yaw;
}

}  
