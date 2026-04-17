#ifndef ROKAE_XMATE3_ROS2_RUNTIME_SESSION_STATE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_SESSION_STATE_HPP

#include <array>
#include <cstdint>
#include <mutex>
#include <string>

#include "runtime/runtime_snapshots.hpp"
#include "runtime/operation_state_adapter.hpp"
#include "rokae_xmate3_ros2/msg/operate_mode.hpp"

namespace rokae_xmate3_ros2::runtime {

inline constexpr int kSessionMotionModeNrt = 0;
inline constexpr int kSessionMotionModeRt = 1;
inline constexpr int kSessionMotionModeRl = 2;
inline constexpr int kSessionMotionModeMin = kSessionMotionModeNrt;
inline constexpr int kSessionMotionModeMax = kSessionMotionModeRl;

class SessionState {
 public:
  SessionState();

  void connect(const std::string &remote_ip);
  void disconnect();

  [[nodiscard]] bool connected() const;
  [[nodiscard]] std::string remoteIp() const;

  void setPowerOn(bool on);
  [[nodiscard]] bool powerOn() const;

  void setDragMode(bool enabled);
  [[nodiscard]] bool dragMode() const;

  void setSimulationMode(bool enabled);
  [[nodiscard]] bool simulationMode() const;

  void setCollisionDetectionEnabled(bool enabled);
  [[nodiscard]] bool collisionDetectionEnabled() const;
  void setCollisionDetectionConfig(const std::array<double, 6> &sensitivity,
                                   std::uint8_t behaviour,
                                   double fallback);
  [[nodiscard]] CollisionDetectionSnapshot collisionDetection() const;

  void setMotionMode(int mode);
  [[nodiscard]] int motionMode() const;

  void setOperateMode(uint8_t mode);
  [[nodiscard]] rokae_xmate3_ros2::msg::OperateMode operateMode() const;

  void setRtControlMode(int mode);
  [[nodiscard]] int rtControlMode() const;

  [[nodiscard]] OperationStateContext makeOperationStateContext(bool rl_project_running) const;

 private:
  mutable std::mutex mutex_;
  bool connected_ = false;
  bool power_on_ = false;
  bool drag_mode_ = false;
  bool simulation_mode_ = true;
  bool collision_detection_enabled_ = false;
  std::array<double, 6> collision_sensitivity_{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  std::uint8_t collision_behaviour_ = 1;
  double collision_fallback_ = 0.0;
  int motion_mode_ = kSessionMotionModeNrt;
  rokae_xmate3_ros2::msg::OperateMode operate_mode_{};
  int rt_control_mode_ = -1;
  std::string remote_ip_;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
