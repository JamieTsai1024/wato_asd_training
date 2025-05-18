#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger(), this)) {  
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1)
  );
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}
 
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Step 1: Initialize costmap to free space 
  costmap_.initialize();

  // Step 2: Convert scan to grid and mark obstacles 
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment; 
    double range = scan->ranges[i];
    if (scan->range_min <= range && range <= scan->range_max) {
      costmap_.markObstacleFromLaser(range, angle);
    }
  }

  // Step 3: Inflate
  costmap_.inflateObstacles();  

  // Step 4: Publish message
  rclcpp::Time timestamp = this->now();
  std::string frame_id = scan->header.frame_id;
  nav_msgs::msg::OccupancyGrid message = costmap_.getOccupancyGrid(timestamp, frame_id);
  costmap_pub_->publish(message);
}
 
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}