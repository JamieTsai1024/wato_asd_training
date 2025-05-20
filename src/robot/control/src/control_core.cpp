#include "control_core.hpp"

namespace robot {

ControlCore::ControlCore(const rclcpp::Logger& logger, rclcpp::Node* node) : logger_(logger) {
  lookahead_distance_ = node->declare_parameter<double>("lookahead_distance", 1.5);
  goal_threshold_ = node->declare_parameter<double>("goal_threshold", 0.5);
  linear_speed_ = node->declare_parameter<double>("linear_speed", 0.5);
  max_steering_angle_ = node->declare_parameter<double>("max_steering_angle", 1.5);
}

void ControlCore::updatePath(const nav_msgs::msg::Path::SharedPtr path) {
  current_path_ = path;
}

void ControlCore::updateOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry) {
  robot_odometry_ = odometry;
}

geometry_msgs::msg::Twist ControlCore::controlLoop() {
  // Skip control (publish 0 velocity) if no path or odometry data is available
  if (!robot_odometry_ || !current_path_ || current_path_->poses.empty()) return geometry_msgs::msg::Twist(); 

  // Check if the robot is close to the goal (redundant catch for delays in path planning)
  const auto& robot_pos = robot_odometry_->pose.pose.position;
  const auto& goal_pos = current_path_->poses.back().pose.position;
  double distance_to_goal = computeDistance(robot_pos, goal_pos);
  if (distance_to_goal < goal_threshold_) {
    return geometry_msgs::msg::Twist(); 
  }

  // Find the lookahead point
  auto target = findLookaheadPoint();
  if (!target) return geometry_msgs::msg::Twist();

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
  // Use last point if no lookahead point is found
  if (!current_path_->poses.empty()) return current_path_->poses.back();
  return std::nullopt;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const geometry_msgs::msg::PoseStamped& target) {
  geometry_msgs::msg::Twist cmd_vel;

  const auto& robot_pose = robot_odometry_->pose.pose;
  double robot_angle = extractYaw(robot_pose.orientation);

  double dx = target.pose.position.x - robot_pose.position.x;
  double dy = target.pose.position.y - robot_pose.position.y;

  double target_angle = std::atan2(dy, dx);
  double steering_angle = target_angle - robot_angle;

  // Normalize angle to [-π, π] (numerically stable)
  steering_angle = std::atan2(std::sin(steering_angle), std::cos(steering_angle)); 
  steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_); 

  double abs_angle = std::abs(steering_angle);
  double speed_scaling = std::max(0.1, 1.0 - abs_angle / max_steering_angle_); 

  cmd_vel.linear.x = linear_speed_ * speed_scaling;
  cmd_vel.angular.z = steering_angle; 

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
