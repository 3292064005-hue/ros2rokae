#include "runtime/request_coordinator.hpp"

#include <algorithm>
#include <utility>

namespace rokae_xmate3_ros2::runtime {

MotionRequestCoordinator::MotionRequestCoordinator(MotionOptionsState &motion_options_state,
                                                   ToolingState &tooling_state,
                                                   SessionState &session_state,
                                                   MotionRuntime &motion_runtime)
    : motion_options_state_(motion_options_state),
      tooling_state_(tooling_state),
      session_state_(session_state),
      motion_runtime_(motion_runtime) {}

RuntimeView MotionRequestCoordinator::currentView() const {
  auto view = motion_runtime_.view();
  view.connected = session_state_.connected();
  view.power_on = session_state_.powerOn();
  view.drag_mode = session_state_.dragMode();
  view.motion_mode = session_state_.motionMode();
  if (!view.connected) {
    view.can_accept_request = false;
    view.snapshot_stale_reason = "session_disconnected";
  } else if (!view.power_on) {
    view.can_accept_request = false;
    view.snapshot_stale_reason = "power_off";
  } else if (view.drag_mode) {
    view.can_accept_request = false;
    view.snapshot_stale_reason = "drag_mode_active";
  } else if (view.motion_mode != kSessionMotionModeNrt) {
    view.can_accept_request = false;
    view.snapshot_stale_reason = "motion_mode_not_nrt";
  }
  return view;
}

bool MotionRequestCoordinator::readAuthorityJointState(std::array<double, 6> &pos,
                                                             std::array<double, 6> &vel,
                                                             std::array<double, 6> &tau) const {
  RobotSnapshot snapshot{};
  const bool live_backend = motion_runtime_.readAuthoritativeSnapshot(snapshot);
  pos = snapshot.joint_position;
  vel = snapshot.joint_velocity;
  tau = snapshot.joint_torque;
  return live_backend;
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

SubmissionResult MotionRequestCoordinator::queueMoveAppend(
    const rokae_xmate3_ros2::action::MoveAppend::Goal &goal,
    const std::array<double, 6> &joint_position,
    double trajectory_dt,
    const std::string &request_id) {
  SubmissionResult result;
  result.request_id = request_id;

  const auto gate = currentView();
  if (!gate.can_accept_request) {
    result.message = gate.snapshot_stale_reason.empty() ? std::string{"runtime request gate rejected"}
                                                        : gate.snapshot_stale_reason;
    return result;
  }

  MotionRequest request;
  std::string request_error;
  const auto context = buildContext(request_id, joint_position, trajectory_dt);
  if (!build_motion_request(goal, context, request, request_error)) {
    result.message = request_error;
    return result;
  }

  std::string queue_message;
  if (!motion_runtime_.queue(request, queue_message)) {
    result.message = queue_message.empty() ? "failed to queue runtime request" : queue_message;
    return result;
  }

  result.success = true;
  result.message = "queued awaiting moveStart";
  return result;
}

SubmissionResult MotionRequestCoordinator::submitReplayPath(
    const ReplayPathAsset &replay_asset,
    double rate,
    const std::array<double, 6> &joint_position,
    double trajectory_dt,
    const std::string &request_id) {
  SubmissionResult result;
  result.request_id = request_id;

  const auto gate = currentView();
  if (!gate.can_accept_request) {
    result.message = gate.snapshot_stale_reason.empty() ? std::string{"runtime request gate rejected"}
                                                        : gate.snapshot_stale_reason;
    return result;
  }

  MotionRequest request;
  std::string request_error;
  const auto context = buildContext(request_id, joint_position, trajectory_dt);
  if (!build_replay_request(replay_asset, rate, context, request, request_error)) {
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

SubmissionResult MotionRequestCoordinator::startQueuedRequest() {
  SubmissionResult result;
  const auto view = currentView();
  result.request_id = view.status.request_id;
  if (!view.connected) {
    result.message = "session_disconnected";
    return result;
  }
  if (view.motion_mode != kSessionMotionModeNrt) {
    result.message = "motion_mode_not_nrt";
    return result;
  }
  if (!view.power_on) {
    result.message = "power_off";
    return result;
  }
  if (view.drag_mode) {
    result.message = "drag_mode_active";
    return result;
  }

  std::string start_message;
  if (!motion_runtime_.commitQueuedRequest(start_message)) {
    result.message = start_message.empty() ? std::string{"moveStart rejected"} : start_message;
    return result;
  }

  result.success = true;
  result.message = start_message.empty() ? std::string{"move start committed"} : start_message;
  return result;
}

RuntimeStatus MotionRequestCoordinator::waitForUpdate(const std::string &request_id,
                                                      std::uint64_t last_revision,
                                                      std::chrono::milliseconds timeout) const {
  return motion_runtime_.waitForUpdate(request_id, last_revision, timeout);
}

void MotionRequestCoordinator::pause(const std::array<double, 6> &joint_position, const std::string &message) {
  RobotSnapshot snapshot;
  snapshot.joint_position = joint_position;
  snapshot.power_on = session_state_.powerOn();
  snapshot.drag_mode = session_state_.dragMode();
  motion_runtime_.pause(snapshot, message);
}

void MotionRequestCoordinator::abort(const std::string &message) {
  motion_runtime_.stop(message);
}

void MotionRequestCoordinator::stop(const std::string &message) {
  abort(message);
}

void MotionRequestCoordinator::reset() {
  motion_runtime_.reset();
}

}  // namespace rokae_xmate3_ros2::runtime
