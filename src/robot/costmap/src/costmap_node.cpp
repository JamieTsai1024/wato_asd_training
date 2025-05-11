#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) { 
  // Declare parameters with default values
  this->declare_parameter("resolution", 0.1);
  this->declare_parameter("width", 100);
  this->declare_parameter("height", 100);
  this->declare_parameter("origin_x", -5.0);
  this->declare_parameter("origin_y", -5.0);
  this->declare_parameter("inflation_radius", 1.0);
  this->declare_parameter("max_cost", 100);

  // Retrieve values into member variables
  this->get_parameter("resolution", costmap_.resolution);
  this->get_parameter("width", costmap_.width);
  this->get_parameter("height", costmap_.height);
  this->get_parameter("origin_x", costmap_.origin_x);
  this->get_parameter("origin_y", costmap_.origin_y);
  this->get_parameter("inflation_radius", costmap_.inflation_radius);
  this->get_parameter("max_cost", costmap_.max_cost);
  
  // Subscriber and publisher 
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(
    &CostmapNode::laserCallback, this, std::placeholders::_1
  ));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishCostmap, this));
}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);
  x_grid = static_cast<int>((x - costmap_.origin_x) / costmap_.resolution);
  y_grid = static_cast<int>((y - costmap_.origin_y) / costmap_.resolution);
}

void CostmapNode::markObstacle(int x, int y) {
  if (x >= 0 && x < costmap_.width && y >= 0 && y < costmap_.height) {
    // Mark cell as occupied
    costmap_.costs[y][x] = costmap_.max_cost;
  }
}

void CostmapNode::inflateObstacles() {
  std::vector<std::vector<int>> inflated = costmap_.costs; 

  for (int y = 0; y < costmap_.height; ++y) {
    for (int x = 0; x < costmap_.width; ++x) {
      // Inflate if the cell is an obstacle
      if (costmap_.costs[y][x] == costmap_.max_cost) {
        int dy_dx_range = static_cast<int>(costmap_.inflation_radius / costmap_.resolution);
        for (int dy = -dy_dx_range; dy <= dy_dx_range; ++dy) {
          for (int dx = -dy_dx_range; dx <= dy_dx_range; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= costmap_.width || ny >= costmap_.height) continue;

            double distance = std::sqrt(dx * dx + dy * dy) * costmap_.resolution;
            if (distance <= costmap_.inflation_radius) {
              int inflated_cost = static_cast<int>(costmap_.max_cost * (1.0 - distance / costmap_.inflation_radius));
              inflated[ny][nx] = std::max(inflated[ny][nx], inflated_cost);
            }
          }
        }
      }
    }
  }
  costmap_.costs = inflated;
}

void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid message;

  // header 
  message.header.stamp = this->now();
  message.header.frame_id = "map";

  // info 
  message.info.map_load_time = this->now();
  message.info.resolution = costmap_.resolution;
  message.info.width = costmap_.width;
  message.info.height = costmap_.height;
  message.info.origin.position.x = costmap_.origin_x;
  message.info.origin.position.y = costmap_.origin_y;
  message.info.origin.position.z = 0.0;

  // data 
  message.data.resize(costmap_.width * costmap_.height);
  for (int y = 0; y < costmap_.height; y++) {
    for (int x = 0; x < costmap_.width; x++) {
      message.data[y * costmap_.width + x] = costmap_.costs[y][x];
    }
  }

  // Publish 
  costmap_pub_->publish(message);
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Step 1: Initialize costmap 
  costmap_.initialize();

  // Step 2: Convert scan to grid and mark obstacles 
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment; 
    double range = scan->ranges[i];

    if (range < scan->range_max && range > scan->range_min) {
      int x_grid, y_grid;
      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }

  // Step 3: Inflate
  inflateObstacles();  

  // Step 4: Publish 
  publishCostmap();
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}