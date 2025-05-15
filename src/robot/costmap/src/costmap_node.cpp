#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger(), this)) {  
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1)
  );
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishCostmap, this));
}
 
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Step 1: Initialize costmap 
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

  // Step 4: Publish 
  publishCostmap();
}

void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid message = costmap_.toOccupancyGrid(this->now());
  costmap_pub_->publish(message);
}
 
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}