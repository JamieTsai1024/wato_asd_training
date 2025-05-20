#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger(), this)) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odometryCallback, this, std::placeholders::_1)
  );
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  bool firstCall = map_memory_.storeCostmap(*msg);
  if (firstCall) updateMap(); // Publish map on initialization
}

void MapMemoryNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double yaw = extractYaw(msg->pose.pose.orientation);
  map_memory_.checkUpdateMap(x, y, yaw); 
}

void MapMemoryNode::updateMap() {
  if (map_memory_.getShouldUpdate()) {
    map_memory_.integrateCostmap();
    publishMap(); 
  }
}

void MapMemoryNode::publishMap() {
  nav_msgs::msg::OccupancyGrid& map = map_memory_.getGlobalMap();
  map.header.stamp = this->get_clock()->now();
  map_pub_->publish(map);
}

double MapMemoryNode::extractYaw(const geometry_msgs::msg::Quaternion& quat) {
  // Quaternion to yaw conversion
  tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
