#include "runtime/service_facade.hpp"

namespace rokae_xmate3_ros2::runtime {

PathFacade::PathFacade(ProgramState &program_state,
                       ToolingState &tooling_state,
                       MotionRequestCoordinator *request_coordinator,
                       JointStateFetcher joint_state_fetcher,
                       TrajectoryDtProvider trajectory_dt_provider,
                       RequestIdGenerator request_id_generator)
    : program_state_(program_state),
      tooling_state_(tooling_state),
      request_coordinator_(request_coordinator),
      joint_state_fetcher_(std::move(joint_state_fetcher)),
      trajectory_dt_provider_(std::move(trajectory_dt_provider)),
      request_id_generator_(std::move(request_id_generator)) {}

void PathFacade::handleStartRecordPath(const rokae_xmate3_ros2::srv::StartRecordPath::Request &req,
                                       rokae_xmate3_ros2::srv::StartRecordPath::Response &res) const {
  (void)req;
  program_state_.startRecordingPath(tooling_state_.toolset(), "sdk_record");
  res.success = true;
}

void PathFacade::handleStopRecordPath(const rokae_xmate3_ros2::srv::StopRecordPath::Request &req,
                                      rokae_xmate3_ros2::srv::StopRecordPath::Response &res) const {
  (void)req;
  program_state_.stopRecordingPath();
  res.success = true;
}

void PathFacade::handleCancelRecordPath(
    const rokae_xmate3_ros2::srv::CancelRecordPath::Request &req,
    rokae_xmate3_ros2::srv::CancelRecordPath::Response &res) const {
  (void)req;
  program_state_.cancelRecordingPath();
  res.success = true;
}

void PathFacade::handleSaveRecordPath(const rokae_xmate3_ros2::srv::SaveRecordPath::Request &req,
                                      rokae_xmate3_ros2::srv::SaveRecordPath::Response &res) const {
  program_state_.saveRecordedPath(req.name);
  res.success = true;
}

void PathFacade::handleReplayPath(const rokae_xmate3_ros2::srv::ReplayPath::Request &req,
                                  rokae_xmate3_ros2::srv::ReplayPath::Response &res) const {
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
  if (!replay_asset.toolset.tool_name.empty() || !replay_asset.toolset.wobj_name.empty()) {
    tooling_state_.setToolset(replay_asset.toolset.tool_name,
                              replay_asset.toolset.wobj_name,
                              replay_asset.toolset.tool_pose,
                              replay_asset.toolset.wobj_pose);
    tooling_state_.setBaseFrame(replay_asset.toolset.base_pose);
    tooling_state_.setToolDynamics(replay_asset.toolset.tool_name,
                                   replay_asset.toolset.tool_mass,
                                   replay_asset.toolset.tool_com);
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
  program_state_.removeSavedPath(req.name, req.remove_all);
  res.success = true;
}

void PathFacade::handleQueryPathLists(const rokae_xmate3_ros2::srv::QueryPathLists::Request &req,
                                      rokae_xmate3_ros2::srv::QueryPathLists::Response &res) const {
  (void)req;
  res.path_names = program_state_.querySavedPaths();
  res.success = true;
}



}  // namespace rokae_xmate3_ros2::runtime
