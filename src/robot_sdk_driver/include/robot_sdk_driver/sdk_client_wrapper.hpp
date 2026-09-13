#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <system_error>

#include "robot_sdk/sdk_client.hpp"

namespace robot_sdk_driver {

// ---------------------------------------------------------------------------
// Latest data snapshots received through SDK data callbacks. The SDK requires
// callbacks to be lightweight, so callbacks only copy data here; publishing is
// done later from ROS timer callbacks.
// ---------------------------------------------------------------------------
struct SdkDataSnapshot {
  std::mutex mtx;

  robot_sdk::RobotState robot_state{};
  bool has_robot_state{false};

  robot_sdk::MotionData motion{};
  bool has_motion{false};

  robot_sdk::ImuData imu{};
  bool has_imu{false};

  robot_sdk::JointStateData joint_state{};
  bool has_joint_state{false};

  robot_sdk::SpeedData speed{};
  bool has_speed{false};

  robot_sdk::LuxData lux{};
  bool has_lux{false};

  robot_sdk::FaultDatas faults{};
};

// ---------------------------------------------------------------------------
// Thread-safe wrapper around robot_sdk::SDKClient.
// ---------------------------------------------------------------------------
class SdkClientWrapper {
 public:
  using ErrorHandler = std::function<void(const std::error_code&)>;

  explicit SdkClientWrapper(ErrorHandler error_handler = [](const std::error_code&) {});
  ~SdkClientWrapper();

  SdkClientWrapper(const SdkClientWrapper&) = delete;
  SdkClientWrapper& operator=(const SdkClientWrapper&) = delete;

  // Connection -----------------------------------------------------------
  std::error_code connect(const std::string& ip, const std::string& port,
                          bool block = true,
                          robot_sdk::TransportProtocol transport = robot_sdk::TransportProtocol::Udp);
  std::error_code disconnect(bool block = true);
  bool isConnected() const;
  robot_sdk::ConnectionState connectionState() const;

  // Data access ----------------------------------------------------------
  std::shared_ptr<SdkDataSnapshot> data() const { return data_; }

  // Motion control -------------------------------------------------------
  std::error_code move(float left_right, float forward_back, float yaw, int timeout_ms = 0);
  std::error_code standUp(int timeout_ms = 0);
  std::error_code lieDown(int timeout_ms = 0);
  std::error_code crawl(int timeout_ms = 0);
  std::error_code climb(int timeout_ms = 0);
  std::error_code slim(int timeout_ms = 0);
  std::error_code gait(int timeout_ms = 0);
  std::error_code dsb(int timeout_ms = 0);
  std::error_code locked(int timeout_ms = 0);
  std::error_code reverseHeadTail(int timeout_ms = 0);
  std::error_code softEmergencyStop(bool on, int timeout_ms = 0);
  std::error_code setMode(int mode, int timeout_ms = 0);
  std::error_code setSpeed(int speed_level, int timeout_ms = 0);
  std::error_code turn(int direction, int timeout_ms = 0);
  std::error_code controlHead(float left_right, float up_down, int timeout_ms = 0);
  std::error_code highLowStance(int stance, int timeout_ms = 0);

  // Lights ---------------------------------------------------------------
  std::error_code frontLight(bool on, int timeout_ms = 0);
  std::error_code backLight(bool on, int timeout_ms = 0);
  std::error_code autoModeLight(bool on, int timeout_ms = 0);

  // Ownership ------------------------------------------------------------
  std::error_code takeControl(int timeout_ms = 0);
  std::error_code releaseControl(int timeout_ms = 0);

  // Data reporting config ------------------------------------------------
  std::error_code setMcConfig(bool on, int timeout_ms = 0);
  std::error_code setImuConfig(int freq, int timeout_ms = 0);
  std::error_code setJointStateConfig(bool on, int timeout_ms = 0);
  std::error_code setSpeedReportConfig(bool on, uint32_t frequency, int timeout_ms = 0);
  std::error_code setLuxConfig(bool on, int timeout_ms = 0);

 private:
  // Internal callback bridge objects. They must stay alive while the SDK
  // client holds them, hence held by shared_ptr members.
  class DataCallback;
  class ControlCallback;

  std::shared_ptr<SdkDataSnapshot> data_{std::make_shared<SdkDataSnapshot>()};
  std::shared_ptr<DataCallback> data_cb_;
  std::shared_ptr<ControlCallback> ctrl_cb_;
  std::shared_ptr<robot_sdk::SDKClient> client_;
  ErrorHandler error_handler_;
};

}  // namespace robot_sdk_driver
