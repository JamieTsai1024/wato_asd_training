#include "map_memory_core.hpp"

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger, rclcpp::Node* node) : logger_(logger) {
  last_x_ = node->declare_parameter<double>("last_x", 0.0);
  last_y_ = node->declare_parameter<double>("last_y", 0.0);
  robot_yaw_ = node->declare_parameter<double>("robot_yaw", 0.0);
  distance_threshold_ = node->declare_parameter<double>("distance_threshold", 1.5);
  costmap_updated_ = false;
  should_update_map_ = false ;
}

void MapMemoryCore::storeCostmap(const nav_msgs::msg::OccupancyGrid& costmap) {
  latest_costmap_ = costmap;
  costmap_updated_ = true;

  // Initialize the global map if it is empty
  if (global_map_.data.empty()) {
    // Calculate global map dimensions
    int local_width = latest_costmap_.info.width;
    int local_height = latest_costmap_.info.height;
    int global_width = local_width * 2;
    int global_height = local_height * 2;
    float resolution = latest_costmap_.info.resolution;
    int global_origin_x = -static_cast<int>(global_width * resolution) / 2;
    int global_origin_y = -static_cast<int>(global_height * resolution) / 2;

    // Set global map parameters
    global_map_.info.width = global_width;
    global_map_.info.height = global_height;
    global_map_.info.origin.position.x = global_origin_x;
    global_map_.info.origin.position.y = global_origin_y;
    global_map_.info.origin.orientation.w = 1.0;
    global_map_.info.resolution = resolution;
    global_map_.data.assign(global_width * global_height, -1);
  }
}

void MapMemoryCore::checkUpdateMap(double x, double y, double yaw) {
  // Check if the robot has moved significantly or if the last position is not set
  double distance = std::hypot(x - last_x_, y - last_y_);
  if (distance >= distance_threshold_) {
    last_x_ = x;
    last_y_ = y;
    robot_yaw_ = yaw;
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
  float local_origin_x = latest_costmap_.info.origin.position.x;
  float local_origin_y = latest_costmap_.info.origin.position.y;

  int global_width = global_map_.info.width;
  int global_height = global_map_.info.height;
  int global_origin_x = global_map_.info.origin.position.x;
  int global_origin_y = global_map_.info.origin.position.y;

  float resolution = latest_costmap_.info.resolution;
  float cos_yaw = std::cos(robot_yaw_); 
  float sin_yaw = std::sin(robot_yaw_);

  for (int y = 0; y < local_height; ++y) {
    for (int x = 0; x < local_width; ++x) {
      int index = y * local_width + x;
      int8_t cost = latest_costmap_.data[index];

      // Skip unknown values
      if (cost < 0) continue;

      // Local cell position in world coordinates (relative to costmap origin)
      float local_x = local_origin_x + x * resolution;
      float local_y = local_origin_y + y * resolution;
        
      // Rotate + translate to global frame
      float global_world_x = cos_yaw * local_x - sin_yaw * local_y + last_x_;
      float global_world_y = sin_yaw * local_x + cos_yaw * local_y + last_y_;

      // Convert to global map indices  
      int global_x = static_cast<int>((global_world_x - global_origin_x) / resolution);
      int global_y = static_cast<int>((global_world_y - global_origin_y) / resolution);
      
      if (0 <= global_x && global_x < global_width && 0 <= global_y && global_y < global_height) {
        int global_index = global_y * global_map_.info.width + global_x;
        global_map_.data[global_index] = cost;
      }
    }
  }
  should_update_map_ = false;
}

} 
