#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot {

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger, rclcpp::Node* node);
    void storeCostmap(const nav_msgs::msg::OccupancyGrid& costmap);
    void checkUpdateMap(double x, double y, double yaw);
    nav_msgs::msg::OccupancyGrid& getGlobalMap();
    bool getShouldUpdate();
    void integrateCostmap();

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    double last_x_;
    double last_y_;
    double robot_yaw_;
    double distance_threshold_;
    bool costmap_updated_;
    bool should_update_map_;
};

}  

#endif  
