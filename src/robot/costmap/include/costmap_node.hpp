#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "costmap_core.hpp"

#include <vector>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void publishCostmap();

  private:
    robot::CostmapCore costmap_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif 