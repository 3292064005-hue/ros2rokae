#include "runtime/request_coordinator.hpp"

#include <algorithm>
#include <utility>

namespace rokae_xmate3_ros2::runtime {

MotionRequestCoordinator::MotionRequestCoordinator(MotionOptionsState &motion_options_state,
                                                   ToolingState &tooling_state,
                                                   MotionRuntime &motion_runtime)
    : motion_options_state_(motion_options_state),
      tooling_state_(tooling_state),
      motion_runtime_(motion_runtime) {}

RuntimeView MotionRequestCoordinator::currentView() const {
  return motion_runtime_.view();
}

bool MotionRequestCoordinator::canAcceptRequest() const {
  return currentView().can_accept_request;
}

MotionRequestContext MotionRequestCoordinator::buildContext(const std::string &request_id,
                                                            const std::array<double, 6> &joint_position,
                                                            double trajectory_dt) const {
  std::vector<double> start_joints(joint_position.begin(), joint_position.end());
  auto context = motion_options_state_.makeMotionRequestContext(
      request_id,
      start_joints,
      std::max(trajectory_dt, 1e-3));
  const auto toolset = tooling_state_.toolset();
  context.tool_pose = toolset.tool_pose;
  context.wobj_pose = toolset.wobj_pose;
  return context;
}

SubmissionResult MotionRequestCoordinator::submitMoveAppend(
    const rokae_xmate3_ros2::action::MoveAppend::Goal &goal,
    const std::array<double, 6> &joint_position,
    double trajectory_dt,
    const std::string &request_id) {
  SubmissionResult result;
  result.request_id = request_id;

  MotionRequest request;
  std::string request_error;
  const auto context = buildContext(request_id, joint_position, trajectory_dt);
  if (!build_motion_request(goal, context, request, request_error)) {
    result.message = request_error;
    return result;
  }

  std::string submit_message;
  if (!motion_runtime_.submit(request, submit_message)) {
    result.message = submit_message.empty() ? "failed to submit runtime request" : submit_message;
    return result;
  }

  result.success = true;
  result.message = "submitted";
  return result;
}

SubmissionResult MotionRequestCoordinator::submitReplayPath(
    const std::vector<std::vector<double>> &recorded_path,
    double rate,
    const std::array<double, 6> &joint_position,
    double trajectory_dt,
    const std::string &request_id) {
  SubmissionResult result;
  result.request_id = request_id;

  MotionRequest request;
  std::string request_error;
  const auto context = buildContext(request_id, joint_position, trajectory_dt);
  if (!build_replay_request(recorded_path, rate, context, request, request_error)) {
    result.message = request_error;
    return result;
  }

  std::string submit_message;
  if (!motion_runtime_.submit(request, submit_message)) {
    result.message = submit_message.empty() ? "failed to submit replay request" : submit_message;
    return result;
  }

  result.success = true;
  result.message = "submitted";
  return result;
}

RuntimeStatus MotionRequestCoordinator::waitForUpdate(const std::string &request_id,
                                                      std::uint64_t last_revision,
                                                      std::chrono::milliseconds timeout) const {
  return motion_runtime_.waitForUpdate(request_id, last_revision, timeout);
}

void MotionRequestCoordinator::stop(const std::string &message) {
  motion_runtime_.stop(message);
}

void MotionRequestCoordinator::reset() {
  motion_runtime_.reset();
}

}  // namespace rokae_xmate3_ros2::runtime
