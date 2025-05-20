#include "costmap_core.hpp"

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger& logger, rclcpp::Node* node) : logger_(logger) {
  // Get parameters 
  int width_in_meters, height_in_meters;
  resolution_ = node->declare_parameter<double>("resolution", 0.1);
  width_in_meters = node->declare_parameter<int>("width", 10);
  height_in_meters = node->declare_parameter<int>("height", 10);
  inflation_radius_ = node->declare_parameter<double>("inflation_radius", 1.5);
  max_cost_ = node->declare_parameter<int>("max_cost", 100);

  // Calculate grid dimensions
  width_ = width_in_meters / resolution_;
  height_ = height_in_meters / resolution_;
  origin_x_ = -width_in_meters / 2.0;
  origin_y_ = -height_in_meters / 2.0;

  // Initialize occupancy grid
  grid_.info.resolution = resolution_;
  grid_.info.width = width_;
  grid_.info.height = height_;
  grid_.info.origin.position.x = origin_x_;
  grid_.info.origin.position.y = origin_y_;
  grid_.info.origin.orientation.w = 1.0;
  grid_.data.resize(width_ * height_, 0);
}

void CostmapCore::initialize() {
  std::fill(grid_.data.begin(), grid_.data.end(), 0);
}

void CostmapCore::markObstacleFromLaser(double range, double angle) {
  // Convert polar coordinates to cartesian  
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);
  int x_grid = static_cast<int>((x - origin_x_) / resolution_);
  int y_grid = static_cast<int>((y - origin_y_) / resolution_);
  // Mark obstacle 
  if (0 <= x_grid && x_grid < width_ && 0 <= y_grid && y_grid < height_) {
    int index = y_grid * width_ + x_grid;
    grid_.data[index] = max_cost_;
  }
}

void CostmapCore::inflateObstacles() {
  std::vector<int8_t> inflated = grid_.data;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      // Inflate if the cell is an obstacle
      if (grid_.data[y * width_ + x] == max_cost_) {
        int inflation_radius = static_cast<int>(inflation_radius_ / resolution_);
        // Inflate around x and y
        for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
          for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
            // Neighboring cell coordinates
            int nx = x + dx;
            int ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) continue;
            double distance = std::hypot(dx, dy) * resolution_;
            if (distance <= inflation_radius_) {
              // Inflate neighbur cell if within radius
              int inflated_cost = static_cast<int>(max_cost_ * (1.0 - distance / inflation_radius_));
              int index = ny * width_ + nx;
              inflated[index] = static_cast<int8_t>(
                std::max(static_cast<int>(inflated[index]), inflated_cost)
              );
            }
          }
        }
      }
    }
  }
  grid_.data = inflated;
}

nav_msgs::msg::OccupancyGrid CostmapCore::getOccupancyGrid(rclcpp::Time timestamp, std::string frame_id) const {
  nav_msgs::msg::OccupancyGrid result = grid_;
  result.header.frame_id = frame_id; 
  result.header.stamp = timestamp;
  result.info.map_load_time = timestamp;
  return result;
}

}