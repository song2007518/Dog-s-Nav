#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

using namespace std::chrono_literals;

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "custom_nav/path_planner.hpp"

namespace custom_nav {

namespace {
double yawFromQuat(const geometry_msgs::msg::Quaternion& q) {
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}
double normAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

class CustomNavNode : public rclcpp::Node {
 public:
  CustomNavNode() : Node("custom_nav_node") {
    // ---------- parameters ----------
    declare_parameter<std::string>("odom_topic", "/odom/current_pose");
    declare_parameter<std::string>("goal_topic", "/goal");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("standup_service", "/zs1w/stand_up");
    declare_parameter<double>("control_hz", 20.0);
    declare_parameter<double>("linear_limit", 0.5);
    declare_parameter<double>("angular_limit", 1.0);
    declare_parameter<double>("goal_tolerance_xy", 0.15);
    declare_parameter<double>("goal_tolerance_yaw", 0.1);
    declare_parameter<bool>("auto_standup", false);
    declare_parameter<bool>("motion_enabled", false);
    declare_parameter<bool>("auto_enable_on_goal", false);
    declare_parameter<std::string>("map_topic", "/map");
    declare_parameter<double>("robot_radius", 0.35);
    declare_parameter<double>("lookahead", 0.5);
    declare_parameter<bool>("treat_unknown_free", true);
    declare_parameter<std::string>("laser_topic", "/laser_scan");
    declare_parameter<double>("laser_inflation", 0.5);
    declare_parameter<double>("replan_period", 0.25);
    declare_parameter<double>("laser_range_max", 6.0);
    declare_parameter<double>("laser_block_check_ahead", 0.8);
    declare_parameter<double>("obstacle_detect_ahead", 1.5);
    // frame of the 2D scan: "base_link" (calibration applied upstream by NX) or
    // a raw sensor frame such as "livox_frame"/"front_lidar" (then laser_ext_xy
    // applies the lidar->base_link offset from /ota/calibration_results.yaml)
    declare_parameter<std::string>("laser_frame", "base_link");
    declare_parameter<double>("laser_ext_x", 0.0);
    declare_parameter<double>("laser_ext_y", 0.0);
    // proximity safety: emergency stop / speed scaling on nearest front obstacle
    declare_parameter<double>("emergency_stop_dist", 0.45);
    declare_parameter<double>("slow_down_dist", 1.5);
    declare_parameter<double>("front_clearance_sector", 1.2);
    // raw 3D lidar cloud layer (front lidar): closes the 2D scan's ~0.5m blind
    // spot and detects low obstacles, using the calibrated lidar->base extrinsic
    declare_parameter<bool>("use_raw_lidar_cloud", true);
    declare_parameter<std::string>("raw_cloud_topic", "/front_lidar");
    declare_parameter<double>("lidar_ext_x", 0.20985);
    declare_parameter<double>("lidar_ext_y", 0.0);
    declare_parameter<double>("lidar_ext_z", 0.16952);
    // obstacle band is height ABOVE the lidar plane (m), same convention as the
    // NX laser_scan node (min_height/max_height) so ground is never a false hit
    declare_parameter<double>("lidar_obstacle_height_min", 0.1);
    declare_parameter<double>("lidar_obstacle_height_max", 1.0);
    declare_parameter<double>("lidar_min_xy", 0.45);

    const std::string odom_topic = get_parameter("odom_topic").as_string();
    const std::string goal_topic = get_parameter("goal_topic").as_string();
    const std::string cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
    const std::string standup_service = get_parameter("standup_service").as_string();
    const std::string map_topic = get_parameter("map_topic").as_string();
    const std::string laser_topic = get_parameter("laser_topic").as_string();
    const double control_hz = get_parameter("control_hz").as_double();
    linear_limit_ = get_parameter("linear_limit").as_double();
    angular_limit_ = get_parameter("angular_limit").as_double();
    tol_xy_ = get_parameter("goal_tolerance_xy").as_double();
    tol_yaw_ = get_parameter("goal_tolerance_yaw").as_double();
    auto_standup_ = get_parameter("auto_standup").as_bool();
    motion_enabled_ = get_parameter("motion_enabled").as_bool();
    auto_enable_on_goal_ = get_parameter("auto_enable_on_goal").as_bool();
    lookahead_ = get_parameter("lookahead").as_double();
    laser_inflation_ = get_parameter("laser_inflation").as_double();
    robot_radius_ = get_parameter("robot_radius").as_double();
    replan_period_ = get_parameter("replan_period").as_double();
    laser_range_max_ = get_parameter("laser_range_max").as_double();
    laser_block_check_ahead_ = get_parameter("laser_block_check_ahead").as_double();
    obstacle_detect_ahead_ = get_parameter("obstacle_detect_ahead").as_double();
    laser_frame_ = get_parameter("laser_frame").as_string();
    laser_ext_x_ = get_parameter("laser_ext_x").as_double();
    laser_ext_y_ = get_parameter("laser_ext_y").as_double();
    emergency_stop_dist_ = get_parameter("emergency_stop_dist").as_double();
    slow_down_dist_ = get_parameter("slow_down_dist").as_double();
    front_clearance_sector_ = get_parameter("front_clearance_sector").as_double();
    use_raw_lidar_cloud_ = get_parameter("use_raw_lidar_cloud").as_bool();
    raw_cloud_topic_ = get_parameter("raw_cloud_topic").as_string();
    lidar_ext_x_ = get_parameter("lidar_ext_x").as_double();
    lidar_ext_y_ = get_parameter("lidar_ext_y").as_double();
    lidar_ext_z_ = get_parameter("lidar_ext_z").as_double();
    lidar_h_min_ = get_parameter("lidar_obstacle_height_min").as_double();
    lidar_h_max_ = get_parameter("lidar_obstacle_height_max").as_double();
    lidar_min_xy_ = get_parameter("lidar_min_xy").as_double();
    planner_.setRobotRadius(get_parameter("robot_radius").as_double());
    planner_.setUnknownFree(get_parameter("treat_unknown_free").as_bool());

    // ---------- clients / services ----------
    standup_client_ = create_client<std_srvs::srv::Trigger>(standup_service);
    enable_srv_ = create_service<std_srvs::srv::Trigger>(
        "enable", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          res->success = sendStandup();
          res->message = res->success ? "standup requested" : "standup request failed";
        });
    motion_srv_ = create_service<std_srvs::srv::SetBool>(
        "set_motion_enabled",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          motion_enabled_ = req->data;
          res->success = true;
          RCLCPP_WARN(get_logger(), "motion_enabled -> %s (EXERCISE EXTREME CAUTION)",
                      motion_enabled_ ? "true" : "false");
        });
    clear_goal_srv_ = create_service<std_srvs::srv::Trigger>(
        "clear_goal", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          {
            std::lock_guard<std::mutex> lock(mtx_);
            has_goal_ = false;
          }
          publishZero();
          res->success = true;
        });

    // ---------- subscriptions ----------
    // position source: /odom/current_pose (robot_tf -> localization_odom, map-aligned + tracks)
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          robot_x_ = msg->pose.pose.position.x;
          robot_y_ = msg->pose.pose.position.y;
          robot_yaw_ = yawFromQuat(msg->pose.pose.orientation);
          has_pose_ = true;
        });
    // occupancy grid map for path planning (transient_local from pc_map_pub)
    {
      rclcpp::QoS qos(1);
      qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
      map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
          map_topic, qos,
          [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mtx_);
            planner_.setMap(msg);
            has_map_ = true;
          });
    }
    // 2D lidar scan -> dynamic obstacles. Frame is usually base_link (NX already
    // applies the lidar calibration); if a raw sensor frame is supplied the
    // calibrated offset (laser_ext_xy) is applied before the map transform.
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        laser_topic, rclcpp::QoS(10),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          handleLaser(msg);
        });
    // raw 3D front lidar cloud -> close-range / low obstacles (see handleRawCloud)
    if (use_raw_lidar_cloud_) {
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          raw_cloud_topic_, rclcpp::QoS(10).best_effort(),
          [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            handleRawCloud(msg);
          });
    }
    const auto goal_cb = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mtx_);
      goal_x_ = msg->pose.position.x;
      goal_y_ = msg->pose.position.y;
      goal_yaw_ = yawFromQuat(msg->pose.orientation);
      has_goal_ = true;
      goal_frame_ = msg->header.frame_id;
      planned_path_.poses.clear();
      path_idx_ = 0;
      // try to plan an obstacle-avoiding path using the occupancy grid
      if (has_map_ && has_pose_) {
        nav_msgs::msg::Path p;
        if (planner_.plan(robot_x_, robot_y_, goal_x_, goal_y_, p)) {
          planned_path_ = p;
          RCLCPP_INFO(get_logger(), "A* path planned: %zu waypoints",
                      planned_path_.poses.size());
        } else {
          RCLCPP_WARN(get_logger(), "A* planning failed, using straight line");
        }
      } else {
        RCLCPP_WARN(get_logger(), "no map/pose for planning, using straight line");
      }
      RCLCPP_INFO(get_logger(), "goal received: frame=%s (%.3f, %.3f) yaw=%.3f",
                  goal_frame_.c_str(), goal_x_, goal_y_, goal_yaw_);
      // auto-enable: stand up (if not yet) and enable motion so the robot goes
      // to the goal by itself without a manual /enable + /set_motion_enabled.
      if (auto_enable_on_goal_) {
        if (!enabled_) {
          RCLCPP_WARN(get_logger(), "auto_enable_on_goal=true: standing up + enabling motion");
          sendStandup();
        } else {
          RCLCPP_INFO(get_logger(), "auto_enable_on_goal=true: already enabled, enabling motion");
        }
        motion_enabled_ = true;
      }
    };
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic, rclcpp::QoS(10), goal_cb);
    // rviz GoalTool publishes to /goal_pose by default
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", rclcpp::QoS(10), goal_cb);

    // ---------- publishers ----------
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("nav_status", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("custom_nav/path", 10);

    // ---------- control loop ----------
    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_hz),
        [this]() { controlTick(); });
    replan_timer_ = create_wall_timer(
        std::chrono::duration<double>(replan_period_),
        [this]() { replanTick(); });
    status_timer_ = create_wall_timer(1s, [this]() { publishStatus(); });

    // ---------- startup ----------
    if (auto_standup_) {
      RCLCPP_WARN(get_logger(), "auto_standup=true: standing up on start...");
      sendStandup();
    }
    RCLCPP_INFO(get_logger(),
                "custom_nav ready. odom=%s goal=%s cmd_vel=%s motion_enabled=%s",
                odom_topic.c_str(), goal_topic.c_str(), cmd_vel_topic.c_str(),
                motion_enabled_ ? "true" : "false");
  }

 private:
  bool sendStandup() {
    if (!standup_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(get_logger(), "standup service not available");
      return false;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto cb = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut) {
      auto resp = fut.get();
      if (resp->success) {
        std::lock_guard<std::mutex> lock(mtx_);
        enabled_ = true;
        ref_x_ = robot_x_;
        ref_y_ = robot_y_;
        ref_yaw_ = robot_yaw_;
        RCLCPP_WARN(get_logger(), "robot ENABLED (standing) at map (%.3f, %.3f)",
                    robot_x_, robot_y_);
      } else {
        RCLCPP_ERROR(get_logger(), "standup command returned failure");
      }
    };
    standup_client_->async_send_request(req, cb);
    return true;
  }

  void controlTick() {
    double gx = 0.0, gy = 0.0, gyaw = 0.0;
    double rx = 0.0, ry = 0.0, ryaw = 0.0;
    bool has_goal = false;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      rx = robot_x_;
      ry = robot_y_;
      ryaw = robot_yaw_;
      if (has_goal_) {
        gx = goal_x_;
        gy = goal_y_;
        gyaw = goal_yaw_;
        has_goal = true;
      }
    }

    if (!motion_enabled_) {
      // safety: never command motion while disabled
      if (cmd_vel_pub_->get_subscription_count() > 0) publishZero();
      return;
    }
    if (!enabled_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "robot not enabled; call 'enable' first");
      publishZero();
      return;
    }
    if (!has_goal) {
      publishZero();
      return;
    }

    // goal reached check
    const double dx = gx - rx;
    const double dy = gy - ry;
    const double dist = std::hypot(dx, dy);
    if (dist < tol_xy_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "goal reached (%.3f m)", dist);
      publishZero();
      return;
    }

    // determine target point: lookahead on planned path, or goal directly
    double tx = gx, ty = gy;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!planned_path_.poses.empty()) {
        // advance past waypoints already reached
        while (path_idx_ + 1 < planned_path_.poses.size()) {
          const auto& wp = planned_path_.poses[path_idx_].pose.position;
          if (std::hypot(wp.x - rx, wp.y - ry) < tol_xy_ * 0.5) {
            ++path_idx_;
          } else {
            break;
          }
        }
        // lookahead: furthest point within lookahead_ ahead of robot
        for (size_t i = path_idx_; i < planned_path_.poses.size(); ++i) {
          const auto& wp = planned_path_.poses[i].pose.position;
          const double d = std::hypot(wp.x - rx, wp.y - ry);
          tx = wp.x;
          ty = wp.y;
          if (d > lookahead_) break;
        }
      }
    }

    // proximity safety: nearest obstacle within the front sector
    double front = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(mtx_);
      front = front_clearance_;
    }
    if (front < emergency_stop_dist_) {
      // If the planned avoidance path's lookahead target is clear, steer (rotate
      // in place, no forward) toward it so the dog can finish the detour instead
      // of parking forever next to a new obstacle.  Only freeze when boxed in.
      const bool escape_clear =
          !planner_.pathBlockedAhead(tx, ty, robot_radius_ * 0.5);
      if (escape_clear) {
        const double yaw_err = normAngle(std::atan2(ty - ry, tx - rx) - ryaw);
        double wz = std::clamp(1.5 * yaw_err, -angular_limit_, angular_limit_);
        if (std::abs(wz) < 0.1) wz = (wz > 0 ? 0.1 : -0.1);  // SDK deadzone
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                             "obstacle %.2f m ahead, steering to avoid", front);
        geometry_msgs::msg::Twist t;
        t.angular.z = wz;
        cmd_vel_pub_->publish(t);
        return;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                           "emergency stop: obstacle %.2f m ahead, no escape",
                           front);
      if (cmd_vel_pub_->get_subscription_count() > 0) publishZero();
      return;
    }
    // slow down as the nearest front obstacle closes in (between stop and slow dist)
    double vscale = 1.0;
    if (front < slow_down_dist_) {
      vscale = std::clamp((front - emergency_stop_dist_) /
                              (slow_down_dist_ - emergency_stop_dist_),
                          0.0, 1.0);
    }

    // SDK velocity deadzone: vx 0 or >=0.05 m/s, wz 0 or >=0.1 rad/s.
    // Smooth follower: combine forward + turning so obstacle avoidance follows
    // an arc instead of a hard "turn-then-drive" polyline. Large heading error
    // still rotates in place first (safer, avoids sideways drift).
    const double t_yaw_local = std::atan2(ty - ry, tx - rx);
    double yaw_err = t_yaw_local - ryaw;
    while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

    const double tdist = std::hypot(tx - rx, ty - ry);
    const double yaw_tol = 0.12;      // ~7deg, dead-band before turning kicks in
    const double rot_only_rad = 0.7;  // heading error beyond this -> rotate in place

    geometry_msgs::msg::Twist t;
    t.linear.x = 0.0;
    t.linear.y = 0.0;
    t.angular.z = 0.0;

    if (std::abs(yaw_err) > rot_only_rad) {
      // big heading error: rotate in place toward the target
      double wz = std::clamp(1.5 * yaw_err, -angular_limit_, angular_limit_);
      if (std::abs(wz) < 0.1) wz = (wz > 0 ? 0.1 : -0.1);  // SDK deadzone
      t.angular.z = wz;
      cmd_vel_pub_->publish(t);
      return;
    }

    if (std::abs(yaw_err) > yaw_tol) {
      // moderate heading error: drive forward while turning (arc), slow down
      // as heading error grows to keep the turn tight and avoid overshoot
      double base_vx = std::clamp(tdist * 0.8, 0.05, linear_limit_);
      double vx = base_vx * std::cos(yaw_err) * vscale;
      if (vx < 0.05) vx = 0.0;  // SDK deadzone: only emit >=0.05 or 0
      double wz = std::clamp(1.2 * yaw_err, -angular_limit_, angular_limit_);
      if (std::abs(wz) < 0.1) wz = (wz > 0 ? 0.1 : -0.1);
      t.linear.x = vx;
      t.angular.z = wz;
      cmd_vel_pub_->publish(t);
      return;
    }

    // aligned: drive straight toward target
    double vx = std::clamp(tdist * 0.8, 0.05, linear_limit_) * vscale;
    t.linear.x = vx;
    cmd_vel_pub_->publish(t);
  }

  // Convert a 2D scan (in laser_frame_, default base_link) into robot-frame
  // obstacle points.  If the scan comes in a raw sensor frame, the calibrated
  // lidar->base_link offset (laser_ext_xy) is applied.
  void handleLaser(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<std::pair<double, double>> robot_pts;
    const bool raw_frame = (msg->header.frame_id != laser_frame_);
    {
      std::lock_guard<std::mutex> lock(mtx_);
      robot_pts.reserve(msg->ranges.size());
      // ignore points closer than the robot radius so the robot itself and the
      // ground directly under it are never treated as obstacles (would make
      // replanning fail from inside an inflated blob)
      const double min_r = robot_radius_;
      const double ex = raw_frame ? laser_ext_x_ : 0.0;
      const double ey = raw_frame ? laser_ext_y_ : 0.0;
      for (size_t i = 0; i < msg->ranges.size(); ++i) {
        const float r = msg->ranges[i];
        if (r < min_r || r > laser_range_max_) continue;
        const double a = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
        // robot frame (sensor at base_link + extrinsic offset): x forward, y left
        robot_pts.emplace_back(r * std::cos(a) + ex, r * std::sin(a) + ey);
      }
      scan_robot_pts_ = std::move(robot_pts);
      scan_seen_ = this->now();
    }
    publishObstacleLayer();
  }

  // Raw 3D front lidar cloud (frame livox_frame) -> robot-frame obstacles using
  // the calibrated lidar->base_link extrinsic and a height gate, closing the 2D
  // scan's ~0.5m blind spot and catching low obstacles.
  void handleRawCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::vector<std::pair<double, double>> robot_pts;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      const auto fld = pointFieldOffset(msg, "x");
      const auto fly = pointFieldOffset(msg, "y");
      const auto flz = pointFieldOffset(msg, "z");
      if (!fld || !fly || !flz) return;
      const size_t step = msg->point_step > 0 ? msg->point_step : 16;
      const size_t n = msg->data.size() / step;
      robot_pts.reserve(n / 2);
      const double ex = lidar_ext_x_, ey = lidar_ext_y_, ez = lidar_ext_z_;
      const double min_xy = lidar_min_xy_;
      for (size_t i = 0; i < n; ++i) {
        const uint8_t* base = msg->data.data() + i * step;
        float x, y, z;
        std::memcpy(&x, base + *fld, sizeof(float));
        std::memcpy(&y, base + *fly, sizeof(float));
        std::memcpy(&z, base + *flz, sizeof(float));
        // to base_link (extrinsic translation; calibration rotation ~ identity)
        const double bx = static_cast<double>(x) + ex;
        const double by = static_cast<double>(y) + ey;
        const double bz = static_cast<double>(z) + ez;
        if (std::hypot(bx, by) < min_xy) continue;     // skip robot body footprint
        if (bx < 0.1) continue;                        // front-facing lidar only
        // obstacle band = height ABOVE the lidar plane (ground-safe, matches NX)
        const double h = bz - ez;
        if (h < lidar_h_min_ || h > lidar_h_max_) continue;
        robot_pts.emplace_back(bx, by);
      }
      cloud_robot_pts_ = std::move(robot_pts);
      cloud_seen_ = this->now();
    }
    publishObstacleLayer();
  }

  // Merge scan + raw-cloud robot-frame points, transform to map frame, feed the
  // planner's dynamic layer, update front clearance, and trigger replanning.
  void publishObstacleLayer() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_pose_ || !has_map_) return;

    std::vector<std::pair<double, double>> pts;
    pts.reserve(scan_robot_pts_.size() + cloud_robot_pts_.size());
    pts.insert(pts.end(), scan_robot_pts_.begin(), scan_robot_pts_.end());
    // expire raw cloud if it stopped arriving
    if ((this->now() - cloud_seen_).seconds() < 0.6) {
      pts.insert(pts.end(), cloud_robot_pts_.begin(), cloud_robot_pts_.end());
    }

    const double c = std::cos(robot_yaw_);
    const double s = std::sin(robot_yaw_);
    std::vector<std::pair<double, double>> world;
    world.reserve(pts.size());
    double best_front = std::numeric_limits<double>::infinity();
    for (const auto& p : pts) {
      const double dist = std::hypot(p.first, p.second);
      if (dist < robot_radius_ * 0.6) continue;  // inside inflated robot blob
      if (std::fabs(normAngle(std::atan2(p.second, p.first))) <= front_clearance_sector_ &&
          dist < best_front) {
        best_front = dist;
      }
      world.emplace_back(robot_x_ + c * p.first - s * p.second,
                         robot_y_ + s * p.first + c * p.second);
    }
    front_clearance_ = best_front;

    planner_.setLocalObstacles(world, laser_inflation_);
    last_obstacle_count_ = world.size();

    // obstacle-change trigger: if a waypoint within the detection horizon is
    // now blocked by a laser obstacle, request an immediate replan.
    if (has_goal_ && !planned_path_.poses.empty()) {
      for (size_t i = path_idx_; i < planned_path_.poses.size(); ++i) {
        const auto& wp = planned_path_.poses[i].pose.position;
        const double d = std::hypot(wp.x - robot_x_, wp.y - robot_y_);
        if (d > obstacle_detect_ahead_) break;
        if (planner_.pathBlockedAhead(wp.x, wp.y, laser_block_check_ahead_)) {
          replanNeeded_ = true;
          break;
        }
      }
    }
  }

  // Byte offset of a named field in a PointCloud2, or nullopt.
  static std::optional<size_t> pointFieldOffset(
      const sensor_msgs::msg::PointCloud2::SharedPtr& msg, const char* name) {
    for (const auto& f : msg->fields) {
      if (f.name == name && f.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        return f.offset;
      }
    }
    return std::nullopt;
  }

  // Replan only when a laser obstacle blocks the current path ahead
  // (set by handleLaser).  Keeping the planned path stable unless a real
  // obstacle requires rerouting avoids path jitter and "swerving".
  void replanTick() {
    bool do_replan = false;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (replanNeeded_ && has_goal_ && has_map_ && has_pose_) {
        do_replan = true;
        replanNeeded_ = false;
      }
    }
    if (do_replan) {
      if (!replan()) {
        // unable to find a path to the goal (blocked/surrounded): stop and
        // clear the goal so we never drive into an obstacle
        std::lock_guard<std::mutex> lock(mtx_);
        has_goal_ = false;
        planned_path_.poses.clear();
        publishZero();
        RCLCPP_WARN(get_logger(), "goal unreachable (blocked); cleared goal, stopped");
      }
    }
  }

  // Returns false if replanning failed (robot surrounded / no path), so the
  // caller can stop motion instead of driving along a stale path into a wall.
  bool replan() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_goal_ || !has_map_ || !has_pose_) return true;
    nav_msgs::msg::Path p;
    if (planner_.plan(robot_x_, robot_y_, goal_x_, goal_y_, p)) {
      // find the closest waypoint on the new path to the robot and start
      // lookahead from there (avoids jumping the lookahead target behind the
      // robot, which would cause the follower to jitter)
      size_t start = 0;
      double best = std::numeric_limits<double>::max();
      for (size_t i = 0; i < p.poses.size(); ++i) {
        const auto& wp = p.poses[i].pose.position;
        const double d = std::hypot(wp.x - robot_x_, wp.y - robot_y_);
        if (d < best) { best = d; start = i; }
      }
      planned_path_ = p;
      path_idx_ = start;
      RCLCPP_INFO(get_logger(), "replanned A* path: %zu waypoints (obst=%zu)",
                  planned_path_.poses.size(), last_obstacle_count_);
      return true;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "replanning failed (blocked); stopping to avoid collision");
      return false;
    }
  }

  void publishZero() {
    geometry_msgs::msg::Twist t;
    t.linear.x = 0.0;
    t.linear.y = 0.0;
    t.angular.z = 0.0;
    cmd_vel_pub_->publish(t);
  }

  void publishStatus() {
    // publish planned path (A*) if available, else straight line to goal
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (has_goal_) {
        if (!planned_path_.poses.empty()) {
          path = planned_path_;
          path.header.stamp = now();
          path.header.frame_id = "map";  // ensure frame_id is always set
        } else {
          const int steps = 20;
          for (int i = 0; i <= steps; ++i) {
            const double t = static_cast<double>(i) / steps;
            geometry_msgs::msg::PoseStamped p;
            p.header.stamp = path.header.stamp;
            p.header.frame_id = "map";
            p.pose.position.x = robot_x_ + (goal_x_ - robot_x_) * t;
            p.pose.position.y = robot_y_ + (goal_y_ - robot_y_) * t;
            path.poses.push_back(p);
          }
        }
      }
    }
    path_pub_->publish(path);

    std_msgs::msg::String msg;
    std::lock_guard<std::mutex> lock(mtx_);
    std::string s = "enabled=" + std::string(enabled_ ? "1" : "0") +
                    " motion=" + std::string(motion_enabled_ ? "1" : "0") +
                    " has_goal=" + std::string(has_goal_ ? "1" : "0") +
                    " pose=(" + (has_pose_ ? std::to_string(robot_x_) + "," + std::to_string(robot_y_) : "n/a") + ")" +
                    " ref=(" + std::to_string(ref_x_) + "," + std::to_string(ref_y_) + ")" +
                    " front=" +
                    (std::isfinite(front_clearance_)
                         ? std::to_string(front_clearance_)
                         : "inf") +
                    " obs=" + std::to_string(last_obstacle_count_);
    msg.data = s;
    status_pub_->publish(msg);
  }

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr standup_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr motion_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_goal_srv_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr replan_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  PathPlanner planner_;
  nav_msgs::msg::Path planned_path_;
  size_t path_idx_{0};
  double lookahead_{0.5};

  std::mutex mtx_;
  double robot_x_{0.0}, robot_y_{0.0}, robot_yaw_{0.0};
  double ref_x_{0.0}, ref_y_{0.0}, ref_yaw_{0.0};
  double goal_x_{0.0}, goal_y_{0.0}, goal_yaw_{0.0};
  bool has_pose_{false};
  bool has_map_{false};
  bool has_goal_{false};
  bool enabled_{false};
  bool auto_standup_{false};
  bool auto_enable_on_goal_{false};
  std::atomic<bool> motion_enabled_{false};
  std::string goal_frame_;

  double linear_limit_{0.5};
  double angular_limit_{1.0};
  double tol_xy_{0.15};
  double tol_yaw_{0.1};
  double robot_radius_{0.35};
  double laser_inflation_{0.5};
  double replan_period_{0.25};
  double laser_range_max_{6.0};
  double laser_block_check_ahead_{0.8};
  double obstacle_detect_ahead_{1.5};
  std::string laser_frame_{"base_link"};
  double laser_ext_x_{0.0};
  double laser_ext_y_{0.0};
  double emergency_stop_dist_{0.45};
  double slow_down_dist_{1.5};
  double front_clearance_sector_{1.2};
  bool use_raw_lidar_cloud_{true};
  std::string raw_cloud_topic_{"/front_lidar"};
  double lidar_ext_x_{0.20985};
  double lidar_ext_y_{0.0};
  double lidar_ext_z_{0.16952};
  double lidar_h_min_{0.1};
  double lidar_h_max_{1.0};
  double lidar_min_xy_{0.45};
  std::vector<std::pair<double, double>> scan_robot_pts_;
  std::vector<std::pair<double, double>> cloud_robot_pts_;
  rclcpp::Time scan_seen_{0, 0, RCL_ROS_TIME};
  rclcpp::Time cloud_seen_{0, 0, RCL_ROS_TIME};
  double front_clearance_{std::numeric_limits<double>::infinity()};
  bool replanNeeded_{false};
  size_t last_obstacle_count_{0};
};

}  // namespace custom_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<custom_nav::CustomNavNode>());
  rclcpp::shutdown();
  return 0;
}
