#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "robot_sdk_driver/sdk_client_wrapper.hpp"

namespace robot_sdk_driver {

class RobotSdkNode : public rclcpp::Node {
 public:
  explicit RobotSdkNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~RobotSdkNode() override;

  // Called on shutdown signal: stop motion, release control, disconnect.
  void shutdownRobot();

 private:
  // Lifecycle -------------------------------------------------------------
  void connectWithRetry();
  void initializeRobot();

  // cmd_vel ---------------------------------------------------------------
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void sendMove(float left_right, float forward_back, float yaw);
  void watchdogTick();

  // one-shot command topics -----------------------------------------------
  void onModeCmd(const std_msgs::msg::UInt8::SharedPtr msg);
  void onSpeedCmd(const std_msgs::msg::UInt8::SharedPtr msg);
  void onTurnCmd(const std_msgs::msg::UInt8::SharedPtr msg);
  void onStanceCmd(const std_msgs::msg::UInt8::SharedPtr msg);
  void onHeadCmd(const geometry_msgs::msg::Vector3::SharedPtr msg);

  // data publishers -------------------------------------------------------
  void publishMotion();
  void publishImu();
  void publishJointStates();
  void publishState();

  // service helpers -------------------------------------------------------
  bool checkConnected() const;

  // Config ----------------------------------------------------------------
  double vxMax(int level) const;
  double vyMax(int level) const;
  double wzMax(int level) const;

  std::shared_ptr<SdkClientWrapper> sdk_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr motion_timer_;
  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr joint_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr turn_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr stance_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr head_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> stand_up_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> lie_down_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> crawl_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> climb_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> slim_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> gait_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> dsb_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> locked_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> reverse_head_tail_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> take_control_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> release_control_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> estop_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> front_light_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> back_light_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> auto_light_srv_;

  // cmd_vel state ---------------------------------------------------------
  float last_lr_{0.0f};
  float last_fb_{0.0f};
  float last_yaw_{0.0f};
  bool have_cmd_{false};
  std::mutex cmd_mtx_;
  rclcpp::Time last_cmd_time_;

  // params ----------------------------------------------------------------
  std::string ip_;
  std::string port_;
  std::string transport_;
  double connect_timeout_{10.0};
  bool auto_take_control_{true};
  bool auto_standup_{false};
  bool connect_only_{false};
  int speed_level_{1};
  int mode_{1};
  bool enable_mc_{true};
  bool enable_imu_{false};
  int imu_freq_{50};
  bool enable_joint_state_{false};
  bool enable_speed_report_{false};
  int speed_report_freq_{50};
  bool enable_lux_{false};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string odom_topic_{"/odom"};
  std::string imu_topic_{"/imu/data"};
  std::string joint_states_topic_{"/joint_states"};
  std::string battery_topic_{"/battery_state"};
  std::string diagnostics_topic_{"/diagnostics"};
  std::string frame_odom_{"odom"};
  std::string frame_base_{"base_link"};
  bool publish_tf_{true};
  double cmd_vel_timeout_{1.0};
  double watchdog_period_{0.2};
  double reconnect_period_{3.0};
  std::vector<double> vx_max_{1.0, 2.0, 3.0};
  std::vector<double> vy_max_{0.5, 0.5, 0.5};
  std::vector<double> wz_max_{1.5, 1.5, 1.5};

  bool initialized_{false};
  bool shutting_down_{false};
};

}  // namespace robot_sdk_driver
