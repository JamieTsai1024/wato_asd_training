#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot {

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger, rclcpp::Node* node);
    void initialize();
    void markObstacleFromLaser(double range, double angle);
    void inflateObstacles();
    nav_msgs::msg::OccupancyGrid getOccupancyGrid(rclcpp::Time timestamp, std::string frame_id) const;

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid grid_; 
    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    double inflation_radius_;
    int max_cost_;
};

}  

#endif  