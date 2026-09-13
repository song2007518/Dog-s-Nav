#pragma once

#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace custom_nav {

struct GridCell {
  int x = 0;
  int y = 0;
  double f = 0.0;  // f = g + h
  double g = 0.0;
  bool operator>(const GridCell& o) const { return f > o.f; }
};

class PathPlanner {
 public:
  PathPlanner() = default;

  void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& map);

  // Plan a path from (sx, sy) to (gx, gy) in map/world coordinates.
  // Returns true on success and fills out_path (world coordinates).
  bool plan(double sx, double sy, double gx, double gy,
            nav_msgs::msg::Path& out_path);

  // Set dynamic obstacles (world/map coordinates) that are merged on top of
  // the static map costmap.  Obstacles are inflated by `radius` (m).
  void setLocalObstacles(const std::vector<std::pair<double, double>>& pts,
                         double radius);

  // Returns true if the cell at world (wx, wy) is occupied (static or dynamic).
  bool isOccupied(double wx, double wy) const;

  // Returns true if any of the given world points falls into an obstacle.
  bool anyOccupied(const std::vector<std::pair<double, double>>& pts) const;

  void setRobotRadius(double r);
  void setUnknownFree(bool f) { unknown_free_ = f; }

  // True if the path goes through a currently-occupied cell at world (wx, wy).
  bool pathBlockedAhead(double wx, double wy, double lookahead) const;

 private:
  void buildCostmap();
  void buildDynamicCostmap();
  void worldToCell(double wx, double wy, int& cx, int& cy) const;
  bool inBounds(int cx, int cy) const;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::vector<int8_t> costmap_;       // static map costmap (0=free, 1=blocked)
  std::vector<int8_t> dynamic_cost_;  // dynamic obstacle layer (0/1)
  std::vector<int8_t> merged_cost_;   // static | dynamic
  int width_ = 0;
  int height_ = 0;
  double res_ = 0.05;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  int inflate_cells_ = 7;  // 0.35m / 0.05m

  double robot_radius_ = 0.35;
  bool unknown_free_ = true;
};

}  // namespace custom_nav
