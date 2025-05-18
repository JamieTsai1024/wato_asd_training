#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot
{

class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger, rclcpp::Node* node);
    void updatePath(const nav_msgs::msg::Path::SharedPtr path);
    void updateOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry);
    std::optional<geometry_msgs::msg::Twist> controlLoop();

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odometry_;
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped& target);
    double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);
    double extractYaw(const geometry_msgs::msg::Quaternion& q);
};

} 

#endif 
