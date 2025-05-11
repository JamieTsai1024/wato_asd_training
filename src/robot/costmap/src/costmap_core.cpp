#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) 
: logger_(logger),
  resolution(0.1),
  width(100),
  height(100),
  origin_x(-5.0),
  origin_y(-5.0),
  inflation_radius(1.0),
  max_cost(100) {
  costs.resize(height, std::vector<int>(width, 0));
}

void CostmapCore::initialize() {
  costs.assign(height, std::vector<int>(width, 0));
}

}