#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <cmath>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <algorithm>

#include "nav_msgs/msg/occupancy_grid.hpp" 
#include "nav_msgs/msg/path.hpp" 
#include "geometry_msgs/msg/point_stamped.hpp" 
#include "geometry_msgs/msg/pose.hpp" 

namespace robot {

// ------------------- Supporting Structures -------------------
 
// 2D grid index
struct CellIndex {
  int x, y;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
  bool operator==(const CellIndex &other) const { return (x == other.x && y == other.y); }
  bool operator!=(const CellIndex &other) const { return (x != other.x || y != other.y); }
};
 
// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};
 
// Structure representing a node in the A* open set
struct AStarNode {
  CellIndex index;
  double f_score;  // f = g + h
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};
 
// Comparator for the priority queue (min-heap by f_score)
struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

// ------------------- PlannerCore Class -------------------

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);
    void updateMap(const nav_msgs::msg::OccupancyGrid& map);
    void updateGoal(const geometry_msgs::msg::PointStamped& goal);
    void updatePose(const geometry_msgs::msg::Pose& pose);
    bool goalReached() const;
    nav_msgs::msg::Path planPath(const rclcpp::Time& now);

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;
    bool goal_received_ = false;
    
    nav_msgs::msg::Path runAStar(const rclcpp::Time& now);
    bool isCellFree(const CellIndex& idx) const;
    std::vector<CellIndex> getNeighbors(const CellIndex& idx) const;
    double heuristic(const CellIndex& a, const CellIndex& b) const;
    double distance(const CellIndex& a, const CellIndex& b) const;
    CellIndex worldToGrid(const geometry_msgs::msg::Point& pt) const;
    geometry_msgs::msg::PoseStamped cellToPose(const CellIndex& idx,  const rclcpp::Time& now) const;
};

}  

#endif  
