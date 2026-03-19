#ifndef ROKAE_XMATE3_ROS2_RUNTIME_EXECUTOR_CORE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_EXECUTOR_CORE_HPP

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include "runtime/runtime_types.hpp"

namespace rokae_xmate3_ros2::runtime {

struct ExecutionStep {
  ControlCommand command;
  ExecutionState state = ExecutionState::idle;
  std::size_t current_segment_index = 0;
  std::size_t completed_segments = 0;
  bool plan_completed = false;
  std::string message;
};

class MotionExecutor {
 public:
  MotionExecutor();

  void reset();
  void stop(const RobotSnapshot &snapshot);
  void loadPlan(MotionPlan plan);

  [[nodiscard]] bool hasActivePlan() const noexcept;
  [[nodiscard]] std::size_t currentSegmentIndex() const noexcept;
  [[nodiscard]] std::size_t totalSegments() const noexcept;

  [[nodiscard]] ExecutionStep tick(const RobotSnapshot &snapshot, double dt);

 private:
  struct ActiveSegment {
    PlannedSegment segment;
    double trajectory_elapsed = 0.0;
    int fine_tuning_steps = 0;
    int settle_attempts = 0;
    bool in_fine_tuning = false;
  };

  void resetControllerState();
  void ensureHoldPosition(const RobotSnapshot &snapshot);
  void startCurrentSegment();

  [[nodiscard]] std::vector<double> buildTerminalVelocityAssist(const RobotSnapshot &snapshot,
                                                                const std::vector<double> &target_position,
                                                                double gain,
                                                                double velocity_limit) const;
  [[nodiscard]] ControlCommand makeControlCommand(const std::array<double, 6> &effort) const;
  [[nodiscard]] ExecutionStep holdPositionStep(const RobotSnapshot &snapshot, double dt);

  struct TrackingStatus {
    ControlCommand command;
    bool settled = false;
    double max_position_error = 0.0;
    double max_velocity_error = 0.0;
  };

  [[nodiscard]] TrackingStatus applyEffortTracking(const RobotSnapshot &snapshot,
                                                   double dt,
                                                   const std::vector<double> &target_position,
                                                   const std::vector<double> &target_velocity,
                                                   const std::array<double, 6> &kp,
                                                   const std::array<double, 6> &kd,
                                                   const std::array<double, 6> &ki,
                                                   double position_tolerance,
                                                   double velocity_tolerance);

  std::optional<MotionPlan> active_plan_;
  std::optional<ActiveSegment> active_segment_;
  std::size_t segment_index_ = 0;
  std::vector<double> hold_position_;
  bool hold_position_initialized_ = false;
  std::array<double, 6> effort_error_integral_{};
  std::array<double, 6> effort_last_position_{};
  std::array<double, 6> effort_filtered_velocity_{};
  bool effort_velocity_initialized_ = false;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
