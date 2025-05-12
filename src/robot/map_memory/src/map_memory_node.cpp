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
  map_memory_.storeCostmap(*msg);
}

void MapMemoryNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  map_memory_.checkUpdateMap(x, y); 
}

void MapMemoryNode::updateMap() {
  if (map_memory_.getShouldUpdate()) {
    map_memory_.integrateCostmap();
    nav_msgs::msg::OccupancyGrid& map = map_memory_.getGlobalMap();
    map.header.stamp = this->get_clock()->now();
    map.header.frame_id = "map";
    map_pub_->publish(map);
  }
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
