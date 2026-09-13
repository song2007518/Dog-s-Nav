#include "robot_sdk_driver/sdk_client_wrapper.hpp"

#include <utility>

namespace robot_sdk_driver {

using namespace robot_sdk;

// ---------------------------------------------------------------------------
// Data callback: copies incoming data into the shared snapshot. Lightweight.
// ---------------------------------------------------------------------------
class SdkClientWrapper::DataCallback : public IDataCallback {
 public:
  explicit DataCallback(std::shared_ptr<SdkDataSnapshot> data) : data_(std::move(data)) {}

  void OnRobotStateData(const RobotState& data) override {
    std::lock_guard<std::mutex> lock(data_->mtx);
    data_->robot_state = data;
    data_->has_robot_state = true;
  }

  void OnMcData(const MotionData& data) override {
    std::lock_guard<std::mutex> lock(data_->mtx);
    data_->motion = data;
    data_->has_motion = true;
  }

  void OnImuData(const ImuData& data) override {
    std::lock_guard<std::mutex> lock(data_->mtx);
    data_->imu = data;
    data_->has_imu = true;
  }

  void OnJointStateData(const JointStateData& data) override {
    std::lock_guard<std::mutex> lock(data_->mtx);
    data_->joint_state = data;
    data_->has_joint_state = true;
  }

  void OnSpeedData(const SpeedData& data) override {
    std::lock_guard<std::mutex> lock(data_->mtx);
    data_->speed = data;
    data_->has_speed = true;
  }

  void OnLuxData(const LuxData& data) override {
    std::lock_guard<std::mutex> lock(data_->mtx);
    data_->lux = data;
    data_->has_lux = true;
  }

  void OnFaultData(const FaultDatas& data) override {
    std::lock_guard<std::mutex> lock(data_->mtx);
    data_->faults = data;
  }

 private:
  std::shared_ptr<SdkDataSnapshot> data_;
};

// ---------------------------------------------------------------------------
// Control callback: acks for sent commands. Logged by the node via a hook.
// ---------------------------------------------------------------------------
class SdkClientWrapper::ControlCallback : public IControlCallback {
 public:
  explicit ControlCallback(std::function<void(const std::string&)> on_event)
      : on_event_(std::move(on_event)) {}

  void OnStandUp() override { notify("stand_up ack"); }
  void OnLieDown() override { notify("lie_down ack"); }
  void OnCrawl() override { notify("crawl ack"); }
  void OnClimb() override { notify("climb ack"); }
  void OnSlim() override { notify("slim ack"); }
  void OnGait() override { notify("gait ack"); }
  void OnDSB() override { notify("dsb ack"); }
  void OnLocked() override { notify("locked ack"); }
  void OnReverseHeadTail() override { notify("reverse_head_tail ack"); }
  void OnSoftEmergencyStop(bool on) override {
    notify(on ? "estop ON" : "estop OFF");
  }
  void OnMode(int mode) override { notify("set_mode ack: " + std::to_string(mode)); }
  void OnSpeed(int speed_level) override {
    notify("set_speed ack: " + std::to_string(speed_level));
  }
  void OnFrontLight(bool on) override { notify(on ? "front_light ON" : "front_light OFF"); }
  void OnBackLight(bool on) override { notify(on ? "back_light ON" : "back_light OFF"); }
  void OnAutoModeLight(bool on) override {
    notify(on ? "auto_light ON" : "auto_light OFF");
  }
  void OnTakeControlAck(const TakeControlAck& ack) override {
    notify(ack.error_code == 0 ? "take_control success"
                               : "take_control failed: " + ack.reason);
  }
  void OnReleaseControlAck(const ReleaseControlAck& ack) override {
    notify(ack.error_code == 0 ? "release_control success"
                               : "release_control failed: " + ack.reason);
  }

 private:
  void notify(const std::string& msg) {
    if (on_event_) on_event_(msg);
  }
  std::function<void(const std::string&)> on_event_;
};

// ---------------------------------------------------------------------------
// SdkClientWrapper implementation
// ---------------------------------------------------------------------------
SdkClientWrapper::SdkClientWrapper(ErrorHandler error_handler)
    : error_handler_(std::move(error_handler)) {
  data_cb_ = std::make_shared<DataCallback>(data_);
  ctrl_cb_ = std::make_shared<ControlCallback>([this](const std::string&) {});
  client_ = std::make_shared<SDKClient>(
      [this](const std::error_code& ec) { error_handler_(ec); },
      ConnectionConfig(), TransportProtocol::Udp);
  client_->SetDataCallback(data_cb_);
  client_->SetControlCallback(ctrl_cb_);
}

SdkClientWrapper::~SdkClientWrapper() {
  if (isConnected()) {
    disconnect(true);
  }
}

std::error_code SdkClientWrapper::connect(const std::string& ip, const std::string& port,
                                          bool block,
                                          TransportProtocol transport) {
  if (transport != TransportProtocol::Udp && transport != TransportProtocol::WebSocket) {
    transport = TransportProtocol::Udp;
  }
  client_->SetDataCallback(data_cb_);
  client_->SetControlCallback(ctrl_cb_);
  return client_->Connect(ip, port, block);
}

std::error_code SdkClientWrapper::disconnect(bool block) {
  return client_->Disconnect(block);
}

bool SdkClientWrapper::isConnected() const { return client_->IsConnected(); }

ConnectionState SdkClientWrapper::connectionState() const {
  return client_->GetConnectionState();
}

std::error_code SdkClientWrapper::move(float left_right, float forward_back, float yaw,
                                       int timeout_ms) {
  return client_->Move(left_right, forward_back, yaw, timeout_ms);
}

std::error_code SdkClientWrapper::standUp(int timeout_ms) {
  return client_->StandUp(timeout_ms);
}

std::error_code SdkClientWrapper::lieDown(int timeout_ms) {
  return client_->LieDown(timeout_ms);
}

std::error_code SdkClientWrapper::crawl(int timeout_ms) {
  return client_->Crawl(timeout_ms);
}

std::error_code SdkClientWrapper::climb(int timeout_ms) {
  return client_->Climb(timeout_ms);
}

std::error_code SdkClientWrapper::slim(int timeout_ms) {
  return client_->Slim(timeout_ms);
}

std::error_code SdkClientWrapper::gait(int timeout_ms) {
  return client_->Gait(timeout_ms);
}

std::error_code SdkClientWrapper::dsb(int timeout_ms) {
  return client_->DSB(timeout_ms);
}

std::error_code SdkClientWrapper::locked(int timeout_ms) {
  return client_->Locked(timeout_ms);
}

std::error_code SdkClientWrapper::reverseHeadTail(int timeout_ms) {
  return client_->ReverseHeadTail(timeout_ms);
}

std::error_code SdkClientWrapper::softEmergencyStop(bool on, int timeout_ms) {
  return client_->SoftEmergencyStop(on, timeout_ms);
}

std::error_code SdkClientWrapper::setMode(int mode, int timeout_ms) {
  return client_->SetMode(mode, timeout_ms);
}

std::error_code SdkClientWrapper::setSpeed(int speed_level, int timeout_ms) {
  return client_->SetSpeed(speed_level, timeout_ms);
}

std::error_code SdkClientWrapper::turn(int direction, int timeout_ms) {
  return client_->Turn(direction, timeout_ms);
}

std::error_code SdkClientWrapper::controlHead(float left_right, float up_down, int timeout_ms) {
  return client_->ControlHead(left_right, up_down, timeout_ms);
}

std::error_code SdkClientWrapper::highLowStance(int stance, int timeout_ms) {
  return client_->HighLowStance(stance, timeout_ms);
}

std::error_code SdkClientWrapper::frontLight(bool on, int timeout_ms) {
  return client_->FrontLight(on, timeout_ms);
}

std::error_code SdkClientWrapper::backLight(bool on, int timeout_ms) {
  return client_->BackLight(on, timeout_ms);
}

std::error_code SdkClientWrapper::autoModeLight(bool on, int timeout_ms) {
  return client_->AutoModeLight(on, timeout_ms);
}

std::error_code SdkClientWrapper::takeControl(int timeout_ms) {
  return client_->TakeControl(timeout_ms);
}

std::error_code SdkClientWrapper::releaseControl(int timeout_ms) {
  return client_->ReleaseControl(timeout_ms);
}

std::error_code SdkClientWrapper::setMcConfig(bool on, int timeout_ms) {
  return client_->SetMcConfig(on, timeout_ms);
}

std::error_code SdkClientWrapper::setImuConfig(int freq, int timeout_ms) {
  return client_->SetImuConfig(freq, timeout_ms);
}

std::error_code SdkClientWrapper::setJointStateConfig(bool on, int timeout_ms) {
  return client_->SetJointStateConfig(on, timeout_ms);
}

std::error_code SdkClientWrapper::setSpeedReportConfig(bool on, uint32_t frequency,
                                                       int timeout_ms) {
  return client_->SetSpeedReportConfig(on, frequency, timeout_ms);
}

std::error_code SdkClientWrapper::setLuxConfig(bool on, int timeout_ms) {
  return client_->SetLuxConfig(on, timeout_ms);
}

}  // namespace robot_sdk_driver
