#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger, rclcpp::Node* node) : logger_(logger) {
  last_x_ = node->declare_parameter<double>("last_x", 0.0);
  last_y_ = node->declare_parameter<double>("last_y", 0.0);
  distance_threshold_ = node->declare_parameter<double>("distance_threshold", 1.5);
  costmap_updated_ = node->declare_parameter<bool>("costmap_updated", false);
  should_update_map_ = node->declare_parameter<bool>("should_update_map", false);
}

void MapMemoryCore::storeCostmap(const nav_msgs::msg::OccupancyGrid& costmap) {
  latest_costmap_ = costmap;
  costmap_updated_ = true;
}

void MapMemoryCore::checkUpdateMap(double x, double y) {
  double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));
  if (distance >= distance_threshold_) {
    last_x_ = x;
    last_y_ = y;
    should_update_map_ = true;
  }
}

nav_msgs::msg::OccupancyGrid& MapMemoryCore::getGlobalMap() {
  return global_map_;
}

bool MapMemoryCore::getShouldUpdate() {
  return should_update_map_ && costmap_updated_;
}

void MapMemoryCore::integrateCostmap() {
  int local_width = latest_costmap_.info.width;
  int local_height = latest_costmap_.info.height;
  float resolution = latest_costmap_.info.resolution;
  float origin_x = latest_costmap_.info.origin.position.x;
  float origin_y = latest_costmap_.info.origin.position.y;

  if (global_map_.data.empty()) {
    global_map_ = latest_costmap_;
    return;
  }

  for (int y = 0; y < local_height; ++y) {
    for (int x = 0; x < local_width; ++x) {
      int index = y * local_width + x;
      int8_t cost = latest_costmap_.data[index];

      // Skip unknown values
      if (cost < 0) continue;

      // Global coordinates of this cell
      float world_x = origin_x + x * resolution;
      float world_y = origin_y + y * resolution;

      // Convert to global map index 
      int global_x = static_cast<int>((world_x - global_map_.info.origin.position.x) / resolution);
      int global_y = static_cast<int>((world_y - global_map_.info.origin.position.y) / resolution);
      int global_index = global_y * global_map_.info.width + global_x;

      if (global_index >= 0 && global_index < static_cast<int>(global_map_.data.size())) {
        // Overwrite with new data 
        global_map_.data[global_index] = cost;
      }
    }
  }
  should_update_map_ = false;
}

} 
