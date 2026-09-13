#include "robot_sdk_driver/robot_sdk_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace robot_sdk_driver {

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

RobotSdkNode::RobotSdkNode(const rclcpp::NodeOptions& options) : Node("robot_sdk_driver", options) {
  // -------- parameters --------
  ip_ = declare_parameter<std::string>("ip", "192.168.234.1");
  port_ = declare_parameter<std::string>("port", "8082");
  transport_ = declare_parameter<std::string>("transport", "udp");
  connect_timeout_ = declare_parameter<double>("connect_timeout", 10.0);
  auto_take_control_ = declare_parameter<bool>("auto_take_control", true);
  auto_standup_ = declare_parameter<bool>("auto_standup", false);
  connect_only_ = declare_parameter<bool>("connect_only", false);
  speed_level_ = declare_parameter<int>("speed_level", 1);
  mode_ = declare_parameter<int>("mode", 1);
  enable_mc_ = declare_parameter<bool>("enable_mc", true);
  enable_imu_ = declare_parameter<bool>("enable_imu", false);
  imu_freq_ = declare_parameter<int>("imu_freq", 50);
  enable_joint_state_ = declare_parameter<bool>("enable_joint_state", false);
  enable_speed_report_ = declare_parameter<bool>("enable_speed_report", false);
  speed_report_freq_ = declare_parameter<int>("speed_report_freq", 50);
  enable_lux_ = declare_parameter<bool>("enable_lux", false);
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
  joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
  battery_topic_ = declare_parameter<std::string>("battery_topic", "/battery_state");
  diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
  frame_odom_ = declare_parameter<std::string>("frame_odom", "odom");
  frame_base_ = declare_parameter<std::string>("frame_base", "base_link");
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 1.0);
  watchdog_period_ = declare_parameter<double>("watchdog_period", 0.2);
  reconnect_period_ = declare_parameter<double>("reconnect_period", 3.0);
  vx_max_ = declare_parameter<std::vector<double>>("vx_max", {1.0, 2.0, 3.0});
  vy_max_ = declare_parameter<std::vector<double>>("vy_max", {0.5, 0.5, 0.5});
  wz_max_ = declare_parameter<std::vector<double>>("wz_max", {1.5, 1.5, 1.5});

  // -------- SDK wrapper --------
  sdk_ = std::make_shared<SdkClientWrapper>(
      [this](const std::error_code& ec) {
        RCLCPP_WARN(get_logger(), "SDK internal error: %s", ec.message().c_str());
      });

  // -------- publishers --------
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(50));
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::QoS(50));
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, rclcpp::QoS(50));
  battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>(battery_topic_, rclcpp::QoS(10));
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, rclcpp::QoS(10));
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // -------- subscriptions --------
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10), std::bind(&RobotSdkNode::onCmdVel, this, _1));
  mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/cmd_mode", rclcpp::QoS(10), std::bind(&RobotSdkNode::onModeCmd, this, _1));
  speed_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/cmd_speed", rclcpp::QoS(10), std::bind(&RobotSdkNode::onSpeedCmd, this, _1));
  turn_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/cmd_turn", rclcpp::QoS(10), std::bind(&RobotSdkNode::onTurnCmd, this, _1));
  stance_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/cmd_stance", rclcpp::QoS(10), std::bind(&RobotSdkNode::onStanceCmd, this, _1));
  head_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "/cmd_head", rclcpp::QoS(10), std::bind(&RobotSdkNode::onHeadCmd, this, _1));

  // -------- services --------
  auto trigger_cb = [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
                           std_srvs::srv::Trigger::Response::SharedPtr res,
                           std::function<std::error_code(int)> f) {
    (void)req;
    if (!checkConnected()) {
      res->success = false;
      res->message = "Not connected";
      return;
    }
    auto ec = f(0);
    res->success = !ec;
    res->message = ec ? ec.message() : "ok";
  };
  auto trigger0 = [this, trigger_cb](std::function<std::error_code()> f) {
    return [this, trigger_cb, f](const std_srvs::srv::Trigger::Request::SharedPtr req,
                                 std_srvs::srv::Trigger::Response::SharedPtr res) {
      trigger_cb(req, res, [f](int) { return f(); });
    };
  };
  auto bool_cb = [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                        std_srvs::srv::SetBool::Response::SharedPtr res,
                        std::function<std::error_code(bool)> f) {
    if (!checkConnected()) {
      res->success = false;
      res->message = "Not connected";
      return;
    }
    auto ec = f(req->data);
    res->success = !ec;
    res->message = ec ? ec.message() : "ok";
  };

  stand_up_srv_ = create_service<std_srvs::srv::Trigger>(
      "stand_up", trigger0([this] { return sdk_->standUp(2000); }));
  lie_down_srv_ = create_service<std_srvs::srv::Trigger>(
      "lie_down", trigger0([this] { return sdk_->lieDown(2000); }));
  crawl_srv_ = create_service<std_srvs::srv::Trigger>(
      "crawl", trigger0([this] { return sdk_->crawl(2000); }));
  climb_srv_ = create_service<std_srvs::srv::Trigger>(
      "climb", trigger0([this] { return sdk_->climb(2000); }));
  slim_srv_ = create_service<std_srvs::srv::Trigger>(
      "slim", trigger0([this] { return sdk_->slim(2000); }));
  gait_srv_ = create_service<std_srvs::srv::Trigger>(
      "gait", trigger0([this] { return sdk_->gait(2000); }));
  dsb_srv_ = create_service<std_srvs::srv::Trigger>(
      "dsb", trigger0([this] { return sdk_->dsb(2000); }));
  locked_srv_ = create_service<std_srvs::srv::Trigger>(
      "locked", trigger0([this] { return sdk_->locked(2000); }));
  reverse_head_tail_srv_ = create_service<std_srvs::srv::Trigger>(
      "reverse_head_tail", trigger0([this] { return sdk_->reverseHeadTail(2000); }));
  take_control_srv_ = create_service<std_srvs::srv::Trigger>(
      "take_control", trigger0([this] { return sdk_->takeControl(2000); }));
  release_control_srv_ = create_service<std_srvs::srv::Trigger>(
      "release_control", trigger0([this] { return sdk_->releaseControl(2000); }));
  estop_srv_ = create_service<std_srvs::srv::SetBool>(
      "estop", [this, bool_cb](const std_srvs::srv::SetBool::Request::SharedPtr req,
                               std_srvs::srv::SetBool::Response::SharedPtr res) {
        bool_cb(req, res, [this](bool on) { return sdk_->softEmergencyStop(on, 2000); });
      });
  front_light_srv_ = create_service<std_srvs::srv::SetBool>(
      "front_light", [this, bool_cb](const std_srvs::srv::SetBool::Request::SharedPtr req,
                                     std_srvs::srv::SetBool::Response::SharedPtr res) {
        bool_cb(req, res, [this](bool on) { return sdk_->frontLight(on, 2000); });
      });
  back_light_srv_ = create_service<std_srvs::srv::SetBool>(
      "back_light", [this, bool_cb](const std_srvs::srv::SetBool::Request::SharedPtr req,
                                    std_srvs::srv::SetBool::Response::SharedPtr res) {
        bool_cb(req, res, [this](bool on) { return sdk_->backLight(on, 2000); });
      });
  auto_light_srv_ = create_service<std_srvs::srv::SetBool>(
      "auto_light", [this, bool_cb](const std_srvs::srv::SetBool::Request::SharedPtr req,
                                    std_srvs::srv::SetBool::Response::SharedPtr res) {
        bool_cb(req, res, [this](bool on) { return sdk_->autoModeLight(on, 2000); });
      });

  // -------- timers --------
  watchdog_timer_ = create_wall_timer(
      std::chrono::duration<double>(watchdog_period_), std::bind(&RobotSdkNode::watchdogTick, this));
  motion_timer_ = create_wall_timer(20ms, std::bind(&RobotSdkNode::publishMotion, this));
  state_timer_ = create_wall_timer(1s, std::bind(&RobotSdkNode::publishState, this));
  if (enable_imu_) {
    imu_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1, imu_freq_)),
        std::bind(&RobotSdkNode::publishImu, this));
  }
  if (enable_joint_state_) {
    joint_timer_ = create_wall_timer(20ms, std::bind(&RobotSdkNode::publishJointStates, this));
  }

  // -------- connect & initialize (blocking) --------
  connectWithRetry();
  if (sdk_->isConnected()) {
    initializeRobot();
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to connect to robot at %s:%s", ip_.c_str(), port_.c_str());
  }
}

RobotSdkNode::~RobotSdkNode() {
  shutdownRobot();
}

bool RobotSdkNode::checkConnected() const {
  if (!sdk_->isConnected()) {
    RCLCPP_WARN(get_logger(), "Robot not connected, command ignored");
    return false;
  }
  return true;
}

double RobotSdkNode::vxMax(int level) const {
  if (level >= 1 && level <= static_cast<int>(vx_max_.size())) return vx_max_[level - 1];
  return 1.0;
}
double RobotSdkNode::vyMax(int level) const {
  if (level >= 1 && level <= static_cast<int>(vy_max_.size())) return vy_max_[level - 1];
  return 0.5;
}
double RobotSdkNode::wzMax(int level) const {
  if (level >= 1 && level <= static_cast<int>(wz_max_.size())) return wz_max_[level - 1];
  return 1.5;
}

void RobotSdkNode::connectWithRetry() {
  auto transport = transport_ == "websocket" ? robot_sdk::TransportProtocol::WebSocket
                                             : robot_sdk::TransportProtocol::Udp;
  RCLCPP_INFO(get_logger(), "Connecting to %s:%s (%s)...", ip_.c_str(), port_.c_str(),
              transport_.c_str());

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(connect_timeout_));

  while (std::chrono::steady_clock::now() < deadline) {
    auto ec = sdk_->connect(ip_, port_, true, transport);
    if (!ec && sdk_->isConnected()) {
      RCLCPP_INFO(get_logger(), "Connected successfully");
      return;
    }
    RCLCPP_WARN(get_logger(), "Connect failed: %s, retrying...", ec.message().c_str());
    std::this_thread::sleep_for(1s);
  }
}

void RobotSdkNode::initializeRobot() {
  if (connect_only_) {
    RCLCPP_WARN(get_logger(), "connect_only mode: not taking control, not enabling reporting");
    return;
  }
  if (auto_take_control_) {
    auto ec = sdk_->takeControl(2000);
    if (ec) RCLCPP_WARN(get_logger(), "takeControl: %s", ec.message().c_str());
  }
  sdk_->setMode(mode_, 2000);
  sdk_->setSpeed(speed_level_, 2000);

  if (enable_mc_) sdk_->setMcConfig(true, 2000);
  if (enable_imu_) sdk_->setImuConfig(imu_freq_, 2000);
  if (enable_joint_state_) sdk_->setJointStateConfig(true, 2000);
  if (enable_speed_report_) sdk_->setSpeedReportConfig(true, speed_report_freq_, 2000);
  if (enable_lux_) sdk_->setLuxConfig(true, 2000);

  if (auto_standup_) {
    RCLCPP_INFO(get_logger(), "Standing up...");
    sdk_->standUp(3000);
  }
  initialized_ = true;
  RCLCPP_INFO(get_logger(), "Robot initialized");
}

void RobotSdkNode::shutdownRobot() {
  if (shutting_down_) return;
  shutting_down_ = true;
  if (sdk_ && sdk_->isConnected()) {
    RCLCPP_INFO(get_logger(), "Shutting down: stop motion, release control, disconnect");
    sdk_->move(0.0f, 0.0f, 0.0f, 1000);
    if (initialized_ && !connect_only_) {
      sdk_->releaseControl(1000);
    }
    sdk_->disconnect(true);
  }
  initialized_ = false;
}

// ---------------------------------------------------------------------------
// cmd_vel
// ---------------------------------------------------------------------------
void RobotSdkNode::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (connect_only_ || !initialized_) return;

  const int level = speed_level_;
  const double vx_max = vxMax(level);
  const double vy_max = vyMax(level);
  const double wz_max = wzMax(level);

  double vx = msg->linear.x;
  double vy = msg->linear.y;
  double wz = msg->angular.z;

  double nvx = vx_max > 0.0 ? std::clamp(vx / vx_max, -1.0, 1.0) : 0.0;
  double nvy = vy_max > 0.0 ? std::clamp(vy / vy_max, -1.0, 1.0) : 0.0;
  double nwz = wz_max > 0.0 ? std::clamp(wz / wz_max, -1.0, 1.0) : 0.0;

  {
    std::lock_guard<std::mutex> lock(cmd_mtx_);
    last_lr_ = static_cast<float>(nvy);
    last_fb_ = static_cast<float>(nvx);
    last_yaw_ = static_cast<float>(nwz);
    have_cmd_ = true;
    last_cmd_time_ = now();
  }
}

void RobotSdkNode::sendMove(float left_right, float forward_back, float yaw) {
  if (connect_only_ || !initialized_) return;
  auto ec = sdk_->move(left_right, forward_back, yaw, 0);
  if (ec) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), static_cast<int64_t>(2.0 * 1e9),
                         "Move failed: %s", ec.message().c_str());
  }
}

void RobotSdkNode::watchdogTick() {
  bool send_zero = false;
  float lr = 0.0f, fb = 0.0f, yaw = 0.0f;
  {
    std::lock_guard<std::mutex> lock(cmd_mtx_);
    if (have_cmd_) {
      double elapsed = (now() - last_cmd_time_).seconds();
      if (elapsed > cmd_vel_timeout_) {
        send_zero = true;
        have_cmd_ = false;
      } else {
        lr = last_lr_;
        fb = last_fb_;
        yaw = last_yaw_;
      }
    } else {
      send_zero = true;
    }
  }
  if (send_zero) {
    sendMove(0.0f, 0.0f, 0.0f);
  } else {
    sendMove(lr, fb, yaw);
  }
}

// ---------------------------------------------------------------------------
// one-shot command topics
// ---------------------------------------------------------------------------
void RobotSdkNode::onModeCmd(const std_msgs::msg::UInt8::SharedPtr msg) {
  if (connect_only_) return;
  if (!checkConnected()) return;
  auto ec = sdk_->setMode(static_cast<int>(msg->data), 2000);
  if (ec) RCLCPP_WARN(get_logger(), "setMode: %s", ec.message().c_str());
  else RCLCPP_INFO(get_logger(), "setMode -> %d", msg->data);
}

void RobotSdkNode::onSpeedCmd(const std_msgs::msg::UInt8::SharedPtr msg) {
  if (connect_only_) return;
  if (!checkConnected()) return;
  auto ec = sdk_->setSpeed(static_cast<int>(msg->data), 2000);
  if (ec) RCLCPP_WARN(get_logger(), "setSpeed: %s", ec.message().c_str());
  else {
    speed_level_ = static_cast<int>(msg->data);
    RCLCPP_INFO(get_logger(), "setSpeed -> %d", msg->data);
  }
}

void RobotSdkNode::onTurnCmd(const std_msgs::msg::UInt8::SharedPtr msg) {
  if (connect_only_) return;
  if (!checkConnected()) return;
  auto ec = sdk_->turn(static_cast<int>(msg->data), 2000);
  if (ec) RCLCPP_WARN(get_logger(), "turn: %s", ec.message().c_str());
}

void RobotSdkNode::onStanceCmd(const std_msgs::msg::UInt8::SharedPtr msg) {
  if (connect_only_) return;
  if (!checkConnected()) return;
  auto ec = sdk_->highLowStance(static_cast<int>(msg->data), 2000);
  if (ec) RCLCPP_WARN(get_logger(), "highLowStance: %s", ec.message().c_str());
}

void RobotSdkNode::onHeadCmd(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  if (connect_only_) return;
  if (!checkConnected()) return;
  auto ec = sdk_->controlHead(msg->x, msg->y, 2000);
  if (ec) RCLCPP_WARN(get_logger(), "controlHead: %s", ec.message().c_str());
}

// ---------------------------------------------------------------------------
// data publishers
// ---------------------------------------------------------------------------
void RobotSdkNode::publishMotion() {
  auto data = sdk_->data();
  std::lock_guard<std::mutex> lock(data->mtx);
  if (!data->has_motion) return;

  const auto& m = data->motion;
  rclcpp::Time ts(m.time_stamp);

  if (odom_pub_->get_subscription_count() > 0 || publish_tf_) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = ts;
    odom.header.frame_id = frame_odom_;
    odom.child_frame_id = frame_base_;
    // MotionData quaternion order is [w, x, y, z]
    odom.pose.pose.orientation.w = m.quat[0];
    odom.pose.pose.orientation.x = m.quat[1];
    odom.pose.pose.orientation.y = m.quat[2];
    odom.pose.pose.orientation.z = m.quat[3];
    odom.pose.pose.position.x = m.position[0];
    odom.pose.pose.position.y = m.position[1];
    odom.pose.pose.position.z = m.position[2];
    // Twist in body frame
    odom.twist.twist.linear.x = m.v_body[0];
    odom.twist.twist.linear.y = m.v_body[1];
    odom.twist.twist.linear.z = m.v_body[2];
    odom.twist.twist.angular.x = m.omega_body[0];
    odom.twist.twist.angular.y = m.omega_body[1];
    odom.twist.twist.angular.z = m.omega_body[2];
    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = ts;
      t.header.frame_id = frame_odom_;
      t.child_frame_id = frame_base_;
      t.transform.translation.x = m.position[0];
      t.transform.translation.y = m.position[1];
      t.transform.translation.z = m.position[2];
      t.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(t);
    }
  }
}

void RobotSdkNode::publishImu() {
  auto data = sdk_->data();
  std::lock_guard<std::mutex> lock(data->mtx);
  if (!data->has_imu) return;

  const auto& imu = data->imu;
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_base_;
  msg.orientation.x = imu.quat_x;
  msg.orientation.y = imu.quat_y;
  msg.orientation.z = imu.quat_z;
  msg.orientation.w = imu.quat_w;
  msg.angular_velocity.x = imu.gyro_x;
  msg.angular_velocity.y = imu.gyro_y;
  msg.angular_velocity.z = imu.gyro_z;
  msg.linear_acceleration.x = imu.acc_x;
  msg.linear_acceleration.y = imu.acc_y;
  msg.linear_acceleration.z = imu.acc_z;
  imu_pub_->publish(msg);
}

void RobotSdkNode::publishJointStates() {
  auto data = sdk_->data();
  std::lock_guard<std::mutex> lock(data->mtx);
  if (!data->has_joint_state) return;

  const auto& js = data->joint_state;
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();
  msg.name = js.names;
  msg.position = js.positions;
  msg.velocity = js.velocities;
  msg.effort = js.efforts;
  joint_state_pub_->publish(msg);
}

void RobotSdkNode::publishState() {
  auto data = sdk_->data();
  std::lock_guard<std::mutex> lock(data->mtx);

  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = now();

  if (data->has_robot_state) {
    const auto& rs = data->robot_state;

    sensor_msgs::msg::BatteryState batt;
    batt.header.stamp = now();
    batt.design_capacity = 0.0f;
    batt.percentage = (rs.battery.power1 + rs.battery.power2) / 200.0f;
    batt.voltage = (rs.battery.voltage1 + rs.battery.voltage2) / 2.0f;
    batt.current = rs.battery.current1 + rs.battery.current2;
    batt.temperature = std::max(rs.battery.temperature1, rs.battery.temperature2);
    battery_pub_->publish(batt);

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = "robot_sdk_driver::robot_state";
    status.hardware_id = ip_ + ":" + port_;
    auto add = [&status](const std::string& key, const std::string& value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      status.values.push_back(kv);
    };
    add("sport_mode", std::to_string(static_cast<int>(rs.sport_mode)));
    add("motion_status", std::to_string(static_cast<int>(rs.motion_status)));
    add("speed_level", std::to_string(static_cast<int>(rs.speed_level)));
    add("head_direction", std::to_string(static_cast<int>(rs.head_direction)));
    add("software_emergency", std::to_string(static_cast<int>(rs.software_emergency_status)));
    add("hardware_emergency", std::to_string(static_cast<int>(rs.hardware_emergency_status)));
    add("control_source", std::to_string(static_cast<int>(rs.control_source)));
    add("front_light", std::to_string(static_cast<int>(rs.front_fill_light)));
    add("back_light", std::to_string(static_cast<int>(rs.back_fill_light)));
    add("auto_light", rs.auto_mode_light ? "1" : "0");
    add("head_angle_rad", std::to_string(rs.head_angle));
    add("mileage_m", std::to_string(rs.mile_data));
    for (const auto& [joint, temp] : rs.joint_temps) {
      add("temp_" + joint, std::to_string(temp));
    }
    diag.status.push_back(status);

    if (!data->faults.empty()) {
      for (const auto& f : data->faults) {
        diagnostic_msgs::msg::DiagnosticStatus fs;
        fs.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        fs.name = "robot_sdk_driver::fault";
        fs.message = f.message;
        fs.hardware_id = std::to_string(static_cast<int>(f.code));
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "level";
        kv.value = std::to_string(static_cast<int>(f.level));
        fs.values.push_back(kv);
        diag.status.push_back(fs);
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), static_cast<int64_t>(5.0 * 1e9),
                           "%zu active fault(s) on robot", data->faults.size());
    }
  } else {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    status.name = "robot_sdk_driver::robot_state";
    status.message = "no robot state received";
    status.hardware_id = ip_ + ":" + port_;
    diag.status.push_back(status);
  }

  diagnostics_pub_->publish(diag);
}

}  // namespace robot_sdk_driver
