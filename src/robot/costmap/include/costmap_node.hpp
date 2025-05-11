#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    void markObstacle(int x, int y);
    void inflateObstacles();
    void publishCostmap();

  private:
    robot::CostmapCore costmap_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif 