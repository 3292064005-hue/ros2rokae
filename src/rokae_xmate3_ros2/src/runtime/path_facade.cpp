#include "runtime/service_facade.hpp"

namespace rokae_xmate3_ros2::runtime {

namespace {

constexpr int kMotionModeNrt = kSessionMotionModeNrt;

bool reject_if_not_connected(const SessionState &session_state, std::string &message) {
  if (!session_state.connected()) {
    message = "Robot not connected";
    return true;
  }
  return false;
}

}  // namespace

PathFacade::PathFacade(SessionState &session_state,
                       ProgramState &program_state,
                       ToolingState &tooling_state,
                       MotionRequestCoordinator *request_coordinator,
                       JointStateFetcher joint_state_fetcher,
                       TrajectoryDtProvider trajectory_dt_provider,
                       RequestIdGenerator request_id_generator)
    : session_state_(session_state),
      program_state_(program_state),
      tooling_state_(tooling_state),
      request_coordinator_(request_coordinator),
      joint_state_fetcher_(std::move(joint_state_fetcher)),
      trajectory_dt_provider_(std::move(trajectory_dt_provider)),
      request_id_generator_(std::move(request_id_generator)) {}

void PathFacade::handleStartRecordPath(const rokae_xmate3_ros2::srv::StartRecordPath::Request &req,
                                       rokae_xmate3_ros2::srv::StartRecordPath::Response &res) const {
  if (reject_if_not_connected(session_state_, res.message)) {
    res.success = false;
    return;
  }
  if (req.duration <= 0) {
    res.success = false;
    res.message = "record duration must be positive";
    return;
  }
  if (program_state_.isRecordingPath()) {
    res.success = false;
    res.message = "path recording already active";
    return;
  }
  if (request_coordinator_ != nullptr && !request_coordinator_->canAcceptRequest()) {
    res.success = false;
    res.message = "Runtime is busy";
    return;
  }
  program_state_.startRecordingPath(tooling_state_.toolset(), "sdk_record");
  res.success = true;
  res.message = "path recording started";
}

void PathFacade::handleStopRecordPath(const rokae_xmate3_ros2::srv::StopRecordPath::Request &req,
                                      rokae_xmate3_ros2::srv::StopRecordPath::Response &res) const {
  (void)req;
  if (reject_if_not_connected(session_state_, res.message)) {
    res.success = false;
    return;
  }
  if (!program_state_.isRecordingPath()) {
    res.success = false;
    res.message = "no active path recording";
    return;
  }
  program_state_.stopRecordingPath();
  res.success = true;
  res.message = "path recording stopped";
}

void PathFacade::handleCancelRecordPath(
    const rokae_xmate3_ros2::srv::CancelRecordPath::Request &req,
    rokae_xmate3_ros2::srv::CancelRecordPath::Response &res) const {
  (void)req;
  if (reject_if_not_connected(session_state_, res.message)) {
    res.success = false;
    return;
  }
  if (!program_state_.isRecordingPath()) {
    res.success = false;
    res.message = "no active path recording";
    return;
  }
  program_state_.cancelRecordingPath();
  res.success = true;
  res.message = "path recording cancelled";
}

void PathFacade::handleSaveRecordPath(const rokae_xmate3_ros2::srv::SaveRecordPath::Request &req,
                                      rokae_xmate3_ros2::srv::SaveRecordPath::Response &res) const {
  if (reject_if_not_connected(session_state_, res.message)) {
    res.success = false;
    return;
  }
  if (req.name.empty()) {
    res.success = false;
    res.message = "path name must not be empty";
    return;
  }
  if (program_state_.isRecordingPath()) {
    res.success = false;
    res.message = "stop path recording before saving";
    return;
  }
  if (program_state_.hasRecordedPathData()) {
    const auto target_name = req.save_as.empty() ? req.name : req.save_as;
    if (target_name.empty()) {
      res.success = false;
      res.message = "path target name must not be empty";
      return;
    }
    program_state_.saveRecordedPath(target_name);
    res.success = true;
    res.message = "path saved";
    return;
  }
  if (req.save_as.empty()) {
    res.success = false;
    res.message = "no recorded path available to save";
    return;
  }
  if (!program_state_.renameSavedPath(req.name, req.save_as)) {
    res.success = false;
    res.message = "path rename source not found";
    return;
  }
  res.success = true;
  res.message = "path renamed";
}

void PathFacade::handleReplayPath(const rokae_xmate3_ros2::srv::ReplayPath::Request &req,
                                  rokae_xmate3_ros2::srv::ReplayPath::Response &res) const {
  if (reject_if_not_connected(session_state_, res.message)) {
    res.success = false;
    return;
  }
  if (session_state_.motionMode() != kMotionModeNrt) {
    res.success = false;
    res.message = "path replay requires NrtCommand-compatible motion mode";
    return;
  }
  if (!session_state_.powerOn()) {
    res.success = false;
    res.message = "Robot not powered on";
    return;
  }
  if (program_state_.isRecordingPath()) {
    res.success = false;
    res.message = "stop path recording before replay";
    return;
  }
  if (req.name.empty()) {
    res.success = false;
    res.message = "path name must not be empty";
    return;
  }
  if (req.rate <= 0.0 || req.rate >= 3.0) {
    res.success = false;
    res.message = "replay rate must be within (0.0, 3.0)";
    return;
  }

  ReplayPathAsset replay_asset;
  if (!program_state_.getReplayAsset(req.name, replay_asset)) {
    res.success = false;
    res.message = "Path not found";
    return;
  }
  if (replay_asset.samples.empty()) {
    res.success = false;
    res.message = "Path is empty";
    return;
  }
  if (request_coordinator_ == nullptr) {
    res.success = false;
    res.message = "Runtime is not initialized";
    return;
  }
  if (!request_coordinator_->canAcceptRequest()) {
    res.success = false;
    res.message = "Runtime is busy";
    return;
  }
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  if (!tooling_state_.isCompatibleWith(replay_asset.toolset)) {
    res.success = false;
    res.message = "recorded path tooling context does not match the active toolset";
    return;
  }
  const auto submission = request_coordinator_->submitReplayPath(
      replay_asset,
      req.rate,
      pos,
      trajectory_dt_provider_(),
      request_id_generator_(std::string("replay_") + req.name));
  res.success = submission.success;
  res.message = submission.message;
}

void PathFacade::handleRemovePath(const rokae_xmate3_ros2::srv::RemovePath::Request &req,
                                  rokae_xmate3_ros2::srv::RemovePath::Response &res) const {
  if (reject_if_not_connected(session_state_, res.message)) {
    res.success = false;
    return;
  }
  if (!req.remove_all && req.name.empty()) {
    res.success = false;
    res.message = "path name must not be empty";
    return;
  }
  program_state_.removeSavedPath(req.name, req.remove_all);
  res.success = true;
  res.message = "path removed";
}

void PathFacade::handleQueryPathLists(const rokae_xmate3_ros2::srv::QueryPathLists::Request &req,
                                      rokae_xmate3_ros2::srv::QueryPathLists::Response &res) const {
  (void)req;
  if (reject_if_not_connected(session_state_, res.message)) {
    res.success = false;
    return;
  }
  res.path_names = program_state_.querySavedPaths();
  res.success = true;
  res.message = "path list queried";
}



}  // namespace rokae_xmate3_ros2::runtime
