#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger, rclcpp::Node* node);
    void initialize();
    void markObstacleFromLaser(double range, double angle);
    void inflateObstacles();
    nav_msgs::msg::OccupancyGrid toOccupancyGrid(rclcpp::Time timestamp, std::string frame_id) const;

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid grid_; 
    double resolution_;        // meters per cell
    int width_;                // number of cells in x
    int height_;               // number of cells in y
    double origin_x_;          // real-world origin (x)
    double origin_y_;          // real-world origin (y)
    double inflation_radius_;  // meters
    int max_cost_;             // max cost (e.g., 100)
};

}  

#endif  