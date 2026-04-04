#ifndef ROKAE_XMATE3_ROS2_RUNTIME_MOCK_RUNTIME_BACKEND_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_MOCK_RUNTIME_BACKEND_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>

#include "runtime/runtime_types.hpp"
#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"

namespace rokae_xmate3_ros2::runtime {

class HeadlessMockRuntimeBackend final : public BackendInterface {
 public:
  HeadlessMockRuntimeBackend() = default;

  [[nodiscard]] RobotSnapshot readSnapshot() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

  void setControlOwner(ControlOwner owner) override { control_owner_.store(owner); }

  [[nodiscard]] ControlOwner controlOwner() const override { return control_owner_.load(); }

  void applyControl(const ControlCommand &command) override {
    std::lock_guard<std::mutex> lock(mutex_);
    last_command_ = command;
    has_command_ = command.has_effort;
  }

  void clearControl() override {
    std::lock_guard<std::mutex> lock(mutex_);
    last_command_ = {};
    has_command_ = false;
    snapshot_.joint_torque.fill(0.0);
  }

  void setBrakeLock(const RobotSnapshot &, bool locked) override {
    brakes_locked_.store(locked);
    if (locked) {
      clearControl();
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot_.joint_velocity.fill(0.0);
    }
  }

  [[nodiscard]] bool brakesLocked() const override { return brakes_locked_.load(); }

  void step(double dt, bool power_on) {
    const double safe_dt = std::clamp(dt, 1e-4, 0.05);
    std::lock_guard<std::mutex> lock(mutex_);

    snapshot_.power_on = power_on;
    if (!power_on || brakes_locked_.load()) {
      for (std::size_t i = 0; i < snapshot_.joint_velocity.size(); ++i) {
        snapshot_.joint_velocity[i] *= 0.90;
        snapshot_.joint_torque[i] = 0.0;
      }
      has_command_ = false;
      return;
    }

    for (std::size_t i = 0; i < snapshot_.joint_position.size(); ++i) {
      double effort = 0.0;
      if (has_command_ && control_owner_.load() == ControlOwner::effort) {
        effort = std::clamp(last_command_.effort[i],
                            -rokae_xmate3_ros2::spec::xmate3::kDirectTorqueLimit[i],
                            rokae_xmate3_ros2::spec::xmate3::kDirectTorqueLimit[i]);
      }

      const double accel = std::clamp(0.05 * effort, -2.5, 2.5);
      snapshot_.joint_velocity[i] =
          std::clamp(snapshot_.joint_velocity[i] + accel * safe_dt,
                     -rokae_xmate3_ros2::spec::xmate3::kJointVelocityLimit[i],
                     rokae_xmate3_ros2::spec::xmate3::kJointVelocityLimit[i]);
      snapshot_.joint_velocity[i] *= 0.995;
      snapshot_.joint_position[i] += snapshot_.joint_velocity[i] * safe_dt;
      snapshot_.joint_position[i] = std::clamp(snapshot_.joint_position[i],
                                               rokae_xmate3_ros2::spec::xmate3::kJointLimitMin[i],
                                               rokae_xmate3_ros2::spec::xmate3::kJointLimitMax[i]);
      snapshot_.joint_torque[i] = effort;
    }
  }

 private:
  mutable std::mutex mutex_;
  RobotSnapshot snapshot_{};
  ControlCommand last_command_{};
  bool has_command_ = false;
  std::atomic<ControlOwner> control_owner_{ControlOwner::none};
  std::atomic<bool> brakes_locked_{false};
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
