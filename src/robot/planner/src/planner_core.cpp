#include "planner_core.hpp"

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger& logger, rclcpp::Node* node) : logger_(logger) {
  goal_threshold_ = node->declare_parameter<double>("goal_threshold", 0.5);; // Distance threshold for reaching the goal 
}

void PlannerCore::updateMap(const nav_msgs::msg::OccupancyGrid& map) {
  current_map_ = map;
}

void PlannerCore::updateGoal(const geometry_msgs::msg::PointStamped& goal) {
  goal_ = goal;
  goal_received_ = true;
}

void PlannerCore::updatePose(const geometry_msgs::msg::Pose& pose) {
  robot_pose_ = pose;
}

bool PlannerCore::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  double distance_to_goal = std::hypot(dx, dy);
  bool reached = distance_to_goal < goal_threshold_;
  if (reached) goal_received_ = false; // Reset goal when reached
  return reached;
}

nav_msgs::msg::Path PlannerCore::planPath(const rclcpp::Time& now) {
  // Check if goal and map are set
  if (!goal_received_) {
    RCLCPP_WARN(logger_, "Cannot plan path: Missing goal!");
    return nav_msgs::msg::Path();
  }
  if (current_map_.data.empty()) {
    RCLCPP_WARN(logger_, "Cannot plan path: Missing map!");
    return nav_msgs::msg::Path();
  }
  // Plan path using A* algorithm
  nav_msgs::msg::Path path = runAStar(now);
  RCLCPP_INFO(logger_, "Path planned of length %d", path.poses.size());
  return path; 
}

nav_msgs::msg::Path PlannerCore::runAStar(const rclcpp::Time& now) {
  nav_msgs::msg::Path path;
  CellIndex start = worldToGrid(robot_pose_.position);
  CellIndex goal = worldToGrid(goal_.point);  

  path.header.stamp = now;
  path.header.frame_id = current_map_.header.frame_id;

  int index = start.x + start.y * current_map_.info.width;
  int goal_index = goal.x + goal.y * current_map_.info.width;
  RCLCPP_INFO(logger_, "Planning A* from start (%d, %d), value %d, to goal (%d, %d), value %d", start.x, start.y, current_map_.data[index], goal.x, goal.y, current_map_.data[goal_index]);

  // Compute path using A* on current_map_ and fill path.poses with the resulting waypoints
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_set<CellIndex, CellIndexHash> closed;

  open.emplace(start, heuristic(start, goal));
  g_score[start] = 0.0;

  while (!open.empty()) {
    CellIndex current_node = open.top().index;
    open.pop();

    if (current_node == goal) {
      // Reconstruct path
      CellIndex current = goal; 
      std::vector<CellIndex> path_cells;
      path_cells.push_back(current);
      while (came_from.find(current) != came_from.end()) {
        current = came_from[current];
        path_cells.push_back(current);
      }
      // Reverse and convert to poses 
      reverse(path_cells.begin(), path_cells.end());
      for (const CellIndex& cell : path_cells) {
        path.poses.push_back(cellToPose(cell, now));
      }
      return path;
    }

    // Skip if node is already processed 
    if (closed.find(current_node) != closed.end()) continue;
    closed.insert(current_node);
    // Explore neighbours 
    for (const CellIndex& neighbor : getNeighbors(current_node)) {
      if (!isCellValid(neighbor) || !isCellFree(neighbor) || closed.find(neighbor) != closed.end()) continue;
      double obstacle_cost = getObstacleCost(neighbor); // Discourage cells near obstacles 
      double tentative_g_score = g_score[current_node] + 5.0 * obstacle_cost + distance(current_node, neighbor);
      if (!g_score.count(neighbor) || tentative_g_score < g_score[neighbor]) {
        came_from[neighbor] = current_node;
        g_score[neighbor] = tentative_g_score;
        double f = tentative_g_score + heuristic(neighbor, goal);
        AStarNode neighbor_node = AStarNode(neighbor, f);
        open.emplace(neighbor_node);
      }
    }
  }

  RCLCPP_WARN(logger_, "No path found - A* failed: open set exhausted.");
  return path;
}

double PlannerCore::getObstacleCost(const CellIndex& cell) const {
  int index = cell.y * current_map_.info.width + cell.x;
  int value = current_map_.data[index];
  double cost = 0.0; 
  if (value == -1) {
    // Unknown cell: discourage but not forbid 
    cost = 0.8; 
  } else {
    // Normalize cost to [0, 1]
    cost = std::clamp(value / 100.0, 0.0, 1.0); 
  }
  return cost; 
}

bool PlannerCore::isCellValid(const CellIndex& cell) const {
  return 0 < cell.x && 0 < cell.y && cell.x < static_cast<int>(current_map_.info.width) && cell.y < static_cast<int>(current_map_.info.height);
}

bool PlannerCore::isCellFree(const CellIndex& cell) const {
  int index = cell.y * current_map_.info.width + cell.x;
  int value = current_map_.data[index];
  bool isUnknown = value == -1;
  // Threshold of 50 for free cells 
  bool isFree = 0 <= value && value < 50; 
  return isUnknown || isFree; 
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex& idx) const {
  std::vector<CellIndex> deltas = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {-1, 1}, {-1, 1}, {-1, -1}
  };
  std::vector<CellIndex> neighbors;
  for (const auto& d : deltas) {
    // Add free cells 
    CellIndex n(idx.x + d.x, idx.y + d.y);
    if (isCellValid(n) && isCellFree(n)) neighbors.push_back(n);
  }
  return neighbors;
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double PlannerCore::distance(const CellIndex& a, const CellIndex& b) const {
  return std::hypot(a.x - b.x, a.y - b.y);  
}

CellIndex PlannerCore::worldToGrid(const geometry_msgs::msg::Point& pt) const {
  int x = static_cast<int>((pt.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
  int y = static_cast<int>((pt.y - current_map_.info.origin.position.y) / current_map_.info.resolution);
  return CellIndex(x, y);
}

geometry_msgs::msg::PoseStamped PlannerCore::cellToPose(const CellIndex& idx, const rclcpp::Time& now) const {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = now;
  pose.header.frame_id = current_map_.header.frame_id;
  pose.pose.position.x = current_map_.info.origin.position.x + (idx.x + 0.5) * current_map_.info.resolution;
  pose.pose.position.y = current_map_.info.origin.position.y + (idx.y + 0.5) * current_map_.info.resolution; 
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

} 
