#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())), state_(State::WAITING_FOR_GOAL) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
  );
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
  );
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odometryCallback, this, std::placeholders::_1)
  );
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  planner_.updateMap(*msg);
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    path_pub_->publish(planner_.planPath(this->get_clock()->now()));
  } else if (state_ == State::WAITING_FOR_GOAL) {
    // Stop the robot if no goal is set
    path_pub_->publish(nav_msgs::msg::Path()); 
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  planner_.updateGoal(*msg);
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  path_pub_->publish(planner_.planPath(this->get_clock()->now()));
}

void PlannerNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  planner_.updatePose(msg->pose.pose);
}

void PlannerNode::timerCallback() {  
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (planner_.goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      path_pub_->publish(planner_.planPath(this->get_clock()->now()));
    }
  }
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
