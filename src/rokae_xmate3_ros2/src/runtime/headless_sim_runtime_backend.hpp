#ifndef ROKAE_XMATE3_ROS2_RUNTIME_HEADLESS_SIM_RUNTIME_BACKEND_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_HEADLESS_SIM_RUNTIME_BACKEND_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>

#include "runtime/runtime_types.hpp"
#include "rokae_xmate3_ros2/spec/xmate_er3_truth.hpp"

namespace rokae_xmate3_ros2::runtime {

class HeadlessSimRuntimeBackend final : public BackendInterface {
  /**
   * @brief Headless deterministic six-axis simulation backend.
   *
   * Function: executes SDK NRT/RT effort commands without Gazebo.
   * Inputs: effort commands from MotionExecutor, power/brake state from runtime session.
   * Outputs: RobotSnapshot with joint position, velocity, torque, and power state.
   * Exceptions: does not throw during the control tick; invalid commands are clamped.
   * Boundary behavior: no trajectory controller and no contact physics; motion is integration-based
   * and must be surfaced as simulation_grade through diagnostics.
   */
 public:
  HeadlessSimRuntimeBackend() = default;

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

  void beginShutdown(const std::string &reason) override {
    (void)reason;
    shutting_down_.store(true);
    clearControl();
    control_owner_.store(ControlOwner::none);
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

  [[nodiscard]] bool supportsEffortExecution() const override { return true; }

  void stepSimulation(double dt, bool power_on) override {
    if (shutting_down_.load()) {
      return;
    }
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
                            -rokae_xmate3_ros2::spec::xmate_er3_truth::kDirectTorqueLimit[i],
                            rokae_xmate3_ros2::spec::xmate_er3_truth::kDirectTorqueLimit[i]);
      }

      const double accel = std::clamp(0.05 * effort, -2.5, 2.5);
      snapshot_.joint_velocity[i] =
          std::clamp(snapshot_.joint_velocity[i] + accel * safe_dt,
                     -rokae_xmate3_ros2::spec::xmate_er3_truth::kJointVelocityLimit[i],
                     rokae_xmate3_ros2::spec::xmate_er3_truth::kJointVelocityLimit[i]);
      snapshot_.joint_velocity[i] *= 0.995;
      snapshot_.joint_position[i] += snapshot_.joint_velocity[i] * safe_dt;
      snapshot_.joint_position[i] = std::clamp(snapshot_.joint_position[i],
                                               rokae_xmate3_ros2::spec::xmate_er3_truth::kJointLimitMin[i],
                                               rokae_xmate3_ros2::spec::xmate_er3_truth::kJointLimitMax[i]);
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
  std::atomic<bool> shutting_down_{false};
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
