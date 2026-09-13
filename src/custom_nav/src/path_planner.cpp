#include "custom_nav/path_planner.hpp"

namespace custom_nav {

namespace {
inline double heur(int x1, int y1, int x2, int y2) {
  const double dx = static_cast<double>(x1 - x2);
  const double dy = static_cast<double>(y1 - y2);
  return std::sqrt(dx * dx + dy * dy);
}
}  // namespace

void PathPlanner::setRobotRadius(double r) {
  robot_radius_ = r;
  inflate_cells_ = static_cast<int>(std::max(1.0, r / res_));
  if (map_) buildCostmap();
}

void PathPlanner::setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& map) {
  map_ = map;
  width_ = static_cast<int>(map->info.width);
  height_ = static_cast<int>(map->info.height);
  res_ = map->info.resolution;
  origin_x_ = map->info.origin.position.x;
  origin_y_ = map->info.origin.position.y;
  inflate_cells_ = static_cast<int>(std::max(1.0, robot_radius_ / res_));
  buildCostmap();
  dynamic_cost_.assign(width_ * height_, 0);
  merged_cost_ = costmap_;
}

void PathPlanner::worldToCell(double wx, double wy, int& cx, int& cy) const {
  cx = static_cast<int>((wx - origin_x_) / res_);
  cy = static_cast<int>((wy - origin_y_) / res_);
}

bool PathPlanner::inBounds(int cx, int cy) const {
  return cx >= 0 && cx < width_ && cy >= 0 && cy < height_;
}

void PathPlanner::buildCostmap() {
  costmap_.assign(width_ * height_, 0);

  // mark occupied cells (and unknown if not free)
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const int v = map_->data[y * width_ + x];
      if (v >= 50) {
        costmap_[y * width_ + x] = 1;
      } else if (v < 0 && !unknown_free_) {
        costmap_[y * width_ + x] = 1;
      }
    }
  }

  // inflate obstacles by robot radius
  std::vector<int8_t> inflated = costmap_;
  const int r = inflate_cells_;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (costmap_[y * width_ + x] == 1) {
        for (int dy = -r; dy <= r; ++dy) {
          for (int dx = -r; dx <= r; ++dx) {
            if (dx * dx + dy * dy > r * r) continue;
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
              inflated[ny * width_ + nx] = 1;
            }
          }
        }
      }
    }
  }
  costmap_ = inflated;
}

void PathPlanner::buildDynamicCostmap() {
  dynamic_cost_.assign(width_ * height_, 0);
  merged_cost_ = costmap_;
}

void PathPlanner::setLocalObstacles(
    const std::vector<std::pair<double, double>>& pts, double radius) {
  if (!map_) return;
  buildDynamicCostmap();

  const int r = std::max(1, static_cast<int>(radius / res_));
  for (const auto& p : pts) {
    int cx, cy;
    worldToCell(p.first, p.second, cx, cy);
    if (!inBounds(cx, cy)) continue;
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (dx * dx + dy * dy > r * r) continue;
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (!inBounds(nx, ny)) continue;
        const int idx = ny * width_ + nx;
        dynamic_cost_[idx] = 1;
        merged_cost_[idx] = 1;
      }
    }
  }
}

bool PathPlanner::isOccupied(double wx, double wy) const {
  if (!map_) return false;
  int cx, cy;
  worldToCell(wx, wy, cx, cy);
  if (!inBounds(cx, cy)) return true;  // outside map -> treat as occupied
  return merged_cost_[cy * width_ + cx] == 1;
}

bool PathPlanner::anyOccupied(
    const std::vector<std::pair<double, double>>& pts) const {
  for (const auto& p : pts) {
    if (isOccupied(p.first, p.second)) return true;
  }
  return false;
}

bool PathPlanner::pathBlockedAhead(double wx, double wy, double lookahead) const {
  if (!map_) return false;
  // check the cell containing (wx, wy) and a small neighborhood
  int cx, cy;
  worldToCell(wx, wy, cx, cy);
  if (!inBounds(cx, cy)) return false;
  const int rr = static_cast<int>(std::max(1.0, lookahead / res_));
  for (int dy = -rr; dy <= rr; ++dy) {
    for (int dx = -rr; dx <= rr; ++dx) {
      if (dx * dx + dy * dy > rr * rr) continue;
      const int nx = cx + dx;
      const int ny = cy + dy;
      if (!inBounds(nx, ny)) continue;
      if (merged_cost_[ny * width_ + nx] == 1) return true;
    }
  }
  return false;
}

bool PathPlanner::plan(double sx, double sy, double gx, double gy,
                       nav_msgs::msg::Path& out_path) {
  out_path.poses.clear();
  if (!map_) return false;
  out_path.header.frame_id = map_->header.frame_id;

  const auto to_cell = [&](double wx, double wy, int& cx, int& cy) {
    cx = static_cast<int>((wx - origin_x_) / res_);
    cy = static_cast<int>((wy - origin_y_) / res_);
    return cx >= 0 && cx < width_ && cy >= 0 && cy < height_;
  };

  int gcx, gcy;
  if (!to_cell(gx, gy, gcx, gcy)) return false;
  int scx, scy;
  if (!to_cell(sx, sy, scx, scy)) return false;

  // Start-cell tolerance: if the robot's current cell is marked occupied by a
  // dynamic obstacle (e.g. a nearby object inflated around it), relax the start
  // to the nearest free cell so A* can still escape instead of failing.
  if (merged_cost_[scy * width_ + scx] == 1) {
    bool found = false;
    for (int r = 1; r <= inflate_cells_ + 2 && !found; ++r) {
      for (int dy = -r; dy <= r && !found; ++dy) {
        for (int dx = -r; dx <= r && !found; ++dx) {
          if (std::abs(dx) != r && std::abs(dy) != r) continue;
          const int nx = scx + dx;
          const int ny = scy + dy;
          if (!inBounds(nx, ny)) continue;
          if (merged_cost_[ny * width_ + nx] == 0) {
            scx = nx;
            scy = ny;
            found = true;
          }
        }
      }
    }
  }

  // Goal-cell tolerance: relax the goal to the nearest free cell too, so a goal
  // placed on/near an obstacle still plans to the closest reachable point.
  if (merged_cost_[gcy * width_ + gcx] == 1) {
    bool found = false;
    for (int r = 1; r <= inflate_cells_ + 2 && !found; ++r) {
      for (int dy = -r; dy <= r && !found; ++dy) {
        for (int dx = -r; dx <= r && !found; ++dx) {
          if (std::abs(dx) != r && std::abs(dy) != r) continue;
          const int nx = gcx + dx;
          const int ny = gcy + dy;
          if (!inBounds(nx, ny)) continue;
          if (merged_cost_[ny * width_ + nx] == 0) {
            gcx = nx;
            gcy = ny;
            found = true;
          }
        }
      }
    }
    // if goal still not free, give up
    if (merged_cost_[gcy * width_ + gcx] == 1) return false;
  }

  const int dirs[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1},
                          {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  const double dir_cost[8] = {1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414};

  std::unordered_map<int, int> parent;  // child cell -> parent cell
  std::unordered_map<int, double> gscore;
  std::priority_queue<GridCell, std::vector<GridCell>, std::greater<GridCell>> open;

  const auto cell_id = [&](int cx, int cy) { return cy * width_ + cx; };
  const int sid = cell_id(scx, scy);
  const int gid = cell_id(gcx, gcy);

  gscore[sid] = 0.0;
  GridCell start;
  start.x = scx;
  start.y = scy;
  start.g = 0.0;
  start.f = heur(scx, scy, gcx, gcy);
  open.push(start);

  bool found = false;
  int iter = 0;
  while (!open.empty() && iter < 500000) {
    GridCell cur = open.top();
    open.pop();
    const int cid = cell_id(cur.x, cur.y);
    if (cid == gid) {
      found = true;
      break;
    }
    ++iter;
    for (int d = 0; d < 8; ++d) {
      const int nx = cur.x + dirs[d][0];
      const int ny = cur.y + dirs[d][1];
      if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;
      if (merged_cost_[ny * width_ + nx] == 1) continue;
      const int nid = cell_id(nx, ny);
      const double ng = cur.g + dir_cost[d];
      const auto it = gscore.find(nid);
      if (it == gscore.end() || ng < it->second) {
        gscore[nid] = ng;
        parent[nid] = cid;
        GridCell n;
        n.x = nx;
        n.y = ny;
        n.g = ng;
        n.f = ng + heur(nx, ny, gcx, gcy);
        open.push(n);
      }
    }
  }

  if (!found) return false;

  // reconstruct path
  std::vector<std::pair<int, int>> cells;
  int cur = gid;
  while (true) {
    const int cx = cur % width_;
    const int cy = cur / width_;
    cells.emplace_back(cx, cy);
    if (cur == sid) break;
    const auto it = parent.find(cur);
    if (it == parent.end()) break;
    cur = it->second;
  }
  std::reverse(cells.begin(), cells.end());

  // convert to world path, keep only every Nth point to thin the path
  const int stride = 2;
  for (size_t i = 0; i < cells.size(); i += stride) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = map_->header.frame_id;
    p.pose.position.x = origin_x_ + (static_cast<double>(cells[i].first) + 0.5) * res_;
    p.pose.position.y = origin_y_ + (static_cast<double>(cells[i].second) + 0.5) * res_;
    p.pose.position.z = 0.0;
    // yaw: direction to next point
    if (i + stride < cells.size()) {
      const double nx = origin_x_ + (static_cast<double>(cells[i + stride].first) + 0.5) * res_;
      const double ny = origin_y_ + (static_cast<double>(cells[i + stride].second) + 0.5) * res_;
      const double yaw = std::atan2(ny - p.pose.position.y, nx - p.pose.position.x);
      p.pose.orientation.z = std::sin(yaw * 0.5);
      p.pose.orientation.w = std::cos(yaw * 0.5);
    } else {
      p.pose.orientation.w = 1.0;
    }
    out_path.poses.push_back(p);
  }
  // ensure last pose == goal
  geometry_msgs::msg::PoseStamped last;
  last.header = out_path.poses.front().header;
  last.pose.position.x = gx;
  last.pose.position.y = gy;
  last.pose.orientation.w = 1.0;
  out_path.poses.push_back(last);

  return true;
}

}  // namespace custom_nav
