#ifndef ROKAE_XMATE3_ROS2_RUNTIME_REQUEST_COORDINATOR_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_REQUEST_COORDINATOR_HPP

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "runtime/runtime_state.hpp"
#include "runtime/motion_runtime.hpp"
#include "rokae_xmate3_ros2/action/move_append.hpp"

namespace rokae_xmate3_ros2::runtime {

struct SubmissionResult {
  bool success = false;
  std::string request_id;
  std::string message;
};

struct FeedbackSnapshot {
  bool should_publish = false;
  double progress = 0.0;
  std::string current_state;
  int current_cmd_index = 0;
};

class MotionRequestCoordinator {
 public:
  MotionRequestCoordinator(MotionOptionsState &motion_options_state, MotionRuntime &motion_runtime);

  [[nodiscard]] RuntimeView currentView() const;
  [[nodiscard]] bool canAcceptRequest() const;

  [[nodiscard]] SubmissionResult submitMoveAppend(
      const rokae_xmate3_ros2::action::MoveAppend::Goal &goal,
      const std::array<double, 6> &joint_position,
      double trajectory_dt,
      const std::string &request_id);

  [[nodiscard]] SubmissionResult submitReplayPath(
      const std::vector<std::vector<double>> &recorded_path,
      double rate,
      const std::array<double, 6> &joint_position,
      double trajectory_dt,
      const std::string &request_id);

  [[nodiscard]] RuntimeStatus waitForUpdate(const std::string &request_id,
                                            std::uint64_t last_revision,
                                            std::chrono::milliseconds timeout) const;

  void stop(const std::string &message);
  void reset();

 private:
  [[nodiscard]] MotionRequestContext buildContext(const std::string &request_id,
                                                  const std::array<double, 6> &joint_position,
                                                  double trajectory_dt) const;

  MotionOptionsState &motion_options_state_;
  MotionRuntime &motion_runtime_;
};

[[nodiscard]] FeedbackSnapshot buildMoveAppendFeedback(const RuntimeStatus &status,
                                                       std::size_t last_completed_segments,
                                                       const std::string &last_message);

}  // namespace rokae_xmate3_ros2::runtime

#endif
