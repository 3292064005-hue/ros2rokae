#ifndef ROKAE_XMATE3_ROS2_RUNTIME_EXECUTOR_CORE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_EXECUTOR_CORE_HPP

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include "runtime/runtime_types.hpp"

namespace rokae_xmate3_ros2::runtime {

struct MotionExecutorConfig {
  std::array<double, 6> active_track_kp{{260.0, 280.0, 240.0, 120.0, 90.0, 45.0}};
  std::array<double, 6> active_track_kd{{28.0, 30.0, 24.0, 12.0, 8.0, 3.5}};
  std::array<double, 6> active_track_ki{{10.0, 12.0, 10.0, 4.0, 2.0, 1.0}};
  std::array<double, 6> hold_track_kp{{380.0, 400.0, 350.0, 170.0, 125.0, 60.0}};
  std::array<double, 6> hold_track_kd{{38.0, 40.0, 32.0, 16.0, 11.0, 5.5}};
  std::array<double, 6> hold_track_ki{{24.0, 26.0, 22.0, 9.0, 4.5, 2.0}};
  std::array<double, 6> strong_hold_track_kp{{560.0, 580.0, 500.0, 235.0, 170.0, 82.0}};
  std::array<double, 6> strong_hold_track_kd{{54.0, 56.0, 45.0, 22.0, 15.0, 7.2}};
  std::array<double, 6> strong_hold_track_ki{{40.0, 42.0, 34.0, 14.0, 7.0, 3.0}};
  std::array<double, 6> static_friction{{0.8, 1.0, 0.8, 0.3, 0.2, 0.1}};
  std::array<double, 6> gravity_compensation{{0.0, 10.0, 7.0, 1.5, 0.8, 0.2}};
  std::array<double, 6> effort_limit{{300.0, 300.0, 300.0, 300.0, 300.0, 300.0}};
  std::array<double, 6> torque_rate_limit{{1200.0, 1200.0, 1000.0, 450.0, 320.0, 180.0}};
  double tracking_position_tolerance_rad = 0.03;
  double tracking_velocity_tolerance_rad = 0.40;
  double final_position_tolerance_rad = 0.01;
  double final_velocity_tolerance_rad = 0.08;
  double soft_final_position_tolerance_rad = 0.02;
  double soft_final_velocity_tolerance_rad = 0.18;
};

struct ExecutionStep {
  ControlCommand command;
  ExecutionState state = ExecutionState::idle;
  std::size_t current_segment_index = 0;
  std::size_t completed_segments = 0;
  bool plan_completed = false;
  bool terminal_success = false;
  std::string message;
};

class MotionExecutor {
 public:
  explicit MotionExecutor(MotionExecutorConfig config = {});

  void reset();
  void stop(const RobotSnapshot &snapshot);
  void loadPlan(MotionPlan plan);
  void setConfig(const MotionExecutorConfig &config);
  [[nodiscard]] const MotionExecutorConfig &config() const noexcept;

  [[nodiscard]] bool hasActivePlan() const noexcept;
  [[nodiscard]] std::size_t currentSegmentIndex() const noexcept;
  [[nodiscard]] std::size_t totalSegments() const noexcept;

  [[nodiscard]] ExecutionStep tick(const RobotSnapshot &snapshot, double dt, double trajectory_time_scale = 1.0);

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
                                                   const std::vector<double> &target_acceleration,
                                                   const std::array<double, 6> &kp,
                                                   const std::array<double, 6> &kd,
                                                   const std::array<double, 6> &ki,
                                                   double position_tolerance,
                                                   double velocity_tolerance);

  std::optional<MotionPlan> active_plan_;
  std::optional<ActiveSegment> active_segment_;
  MotionExecutorConfig config_;
  std::size_t segment_index_ = 0;
  std::vector<double> hold_position_;
  bool hold_position_initialized_ = false;
  std::array<double, 6> effort_error_integral_{};
  std::array<double, 6> effort_last_position_{};
  std::array<double, 6> effort_last_velocity_{};
  std::array<double, 6> effort_filtered_velocity_{};
  std::array<double, 6> effort_filtered_acceleration_{};
  std::array<double, 6> effort_last_command_{};
  bool effort_velocity_initialized_ = false;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
