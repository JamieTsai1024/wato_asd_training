#include "planner_core.hpp"

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

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

bool PlannerCore::goalReached() const {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  double distance_to_goal = std::hypot(dx, dy);
  double threshold = 0.5; // Threshold for reaching the goal
  return distance_to_goal < threshold;
}

nav_msgs::msg::Path PlannerCore::planPath(const rclcpp::Time& now) {
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(logger_, "Cannot plan path: Missing map or goal!");
    return nav_msgs::msg::Path();
  }
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

  // Check for occupied start or goal nodes 
  if (!isCellFree(start)) {
    RCLCPP_WARN(logger_, "Start cell is not free!");
    return path;
  }
  if (!isCellFree(goal)) {
    RCLCPP_WARN(logger_, "Goal cell is not free!");
    return path;
  }

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
      for (const CellIndex& cell : path_cells)
        path.poses.push_back(cellToPose(cell, now));
      
      RCLCPP_INFO(logger_, "A* succeeded with %zu points", path.poses.size());
      return path;
    }

    // Skip if node is already processed 
    if (closed.find(current_node) != closed.end()) continue;
    closed.insert(current_node);

    // Explore neighbours 
    for (const CellIndex& neighbor : getNeighbors(current_node)) {
      if (!isCellFree(neighbor) || closed.find(neighbor) != closed.end()) continue;

      double tentative_g = g_score[current_node] + distance(current_node, neighbor);
      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current_node;
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal);
        open.emplace(neighbor, f);
      }
    }
  }

  RCLCPP_WARN(logger_, "No path found - A* failed: open set exhausted.");
  return path;
}

bool PlannerCore::isCellFree(const CellIndex& cell) const {
  if (cell.x < 0 || cell.y < 0 || cell.x >= static_cast<int>(current_map_.info.width) || cell.y >= static_cast<int>(current_map_.info.height)) return false;
  int index = cell.y * current_map_.info.width + cell.x;
  int value = current_map_.data[index];
  bool cellFree = value == -1 || (0 <= value && value < 50); // threshold of 50 for free cells 
  return cellFree; 
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex& idx) const {
  std::vector<CellIndex> deltas = {{1,0}, {-1,0}, {0,1}, {0,-1}};
  std::vector<CellIndex> neighbors;
  for (const auto& d : deltas) {
    // Add free cells 
    CellIndex n(idx.x + d.x, idx.y + d.y);
    if (isCellFree(n)) neighbors.push_back(n);
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
  pose.pose.position.x = current_map_.info.origin.position.x + idx.x * current_map_.info.resolution;
  pose.pose.position.y = current_map_.info.origin.position.y + idx.y * current_map_.info.resolution;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

} 
