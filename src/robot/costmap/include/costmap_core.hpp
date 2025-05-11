#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);
    std::vector<std::vector<int>> costs; // 2D grid of costs
    double resolution;                   // meters per cell
    int width;                           // number of cells in x
    int height;                          // number of cells in y
    double origin_x;                     // real-world origin (x)
    double origin_y;                     // real-world origin (y)
    double inflation_radius;             // meters
    int max_cost;                        // max cost (e.g., 100)
    void initialize();

  private:
    rclcpp::Logger logger_;

};

}  

#endif  