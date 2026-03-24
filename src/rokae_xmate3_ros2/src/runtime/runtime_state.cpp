#include "runtime/runtime_state.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "runtime/pose_utils.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

std::vector<double> normalize_runtime_pose(const std::vector<double> &pose) {
  return pose_utils::sanitizePose(pose);
}

double clamp_record_timestamp(double timestamp_sec, double fallback) {
  if (!std::isfinite(timestamp_sec)) {
    return fallback;
  }
  return timestamp_sec;
}

}  // namespace

void SessionState::connect(const std::string &remote_ip) {
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = true;
  remote_ip_ = remote_ip;
}

void SessionState::disconnect() {
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = false;
  power_on_ = false;
  drag_mode_ = false;
  rt_control_mode_ = -1;
  motion_mode_ = 0;
}

bool SessionState::connected() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return connected_;
}

std::string SessionState::remoteIp() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return remote_ip_;
}

void SessionState::setPowerOn(bool on) {
  std::lock_guard<std::mutex> lock(mutex_);
  power_on_ = on;
  if (!power_on_) {
    rt_control_mode_ = -1;
  }
}

bool SessionState::powerOn() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return power_on_;
}

void SessionState::setDragMode(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  drag_mode_ = enabled;
}

bool SessionState::dragMode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return drag_mode_;
}

void SessionState::setSimulationMode(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  simulation_mode_ = enabled;
}

bool SessionState::simulationMode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return simulation_mode_;
}

void SessionState::setCollisionDetectionEnabled(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  collision_detection_enabled_ = enabled;
}

bool SessionState::collisionDetectionEnabled() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return collision_detection_enabled_;
}

void SessionState::setCollisionDetectionConfig(const std::array<double, 6> &sensitivity,
                                               std::uint8_t behaviour,
                                               double fallback) {
  std::lock_guard<std::mutex> lock(mutex_);
  collision_sensitivity_ = sensitivity;
  collision_behaviour_ = behaviour;
  collision_fallback_ = fallback;
}

CollisionDetectionSnapshot SessionState::collisionDetection() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return CollisionDetectionSnapshot{
      collision_detection_enabled_,
      collision_sensitivity_,
      collision_behaviour_,
      collision_fallback_,
  };
}

void SessionState::setMotionMode(int mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  motion_mode_ = mode;
}

int SessionState::motionMode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return motion_mode_;
}

void SessionState::setOperateMode(uint8_t mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  operate_mode_.mode = mode;
}

rokae_xmate3_ros2::msg::OperateMode SessionState::operateMode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return operate_mode_;
}

void SessionState::setRtControlMode(int mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  rt_control_mode_ = mode;
}

int SessionState::rtControlMode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rt_control_mode_;
}

OperationStateContext SessionState::makeOperationStateContext(bool rl_project_running) const {
  std::lock_guard<std::mutex> lock(mutex_);
  OperationStateContext context;
  context.connected = connected_;
  context.power_on = power_on_;
  context.drag_mode = drag_mode_;
  context.rl_project_running = rl_project_running;
  context.rt_control_mode = rt_control_mode_;
  return context;
}

void MotionOptionsState::setDefaultSpeed(double speed) {
  std::lock_guard<std::mutex> lock(mutex_);
  default_speed_ = speed;
}

double MotionOptionsState::defaultSpeed() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return default_speed_;
}

void MotionOptionsState::setDefaultZone(int zone) {
  std::lock_guard<std::mutex> lock(mutex_);
  default_zone_ = zone;
}

int MotionOptionsState::defaultZone() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return default_zone_;
}

void MotionOptionsState::setZoneValidRange(int min_zone, int max_zone) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (max_zone < min_zone) {
    std::swap(min_zone, max_zone);
  }
  zone_valid_min_ = min_zone;
  zone_valid_max_ = max_zone;
  default_zone_ = std::clamp(default_zone_, zone_valid_min_, zone_valid_max_);
}

std::array<int, 2> MotionOptionsState::zoneValidRange() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return {zone_valid_min_, zone_valid_max_};
}

void MotionOptionsState::setSpeedScale(double scale) {
  std::lock_guard<std::mutex> lock(mutex_);
  speed_scale_ = scale;
}

double MotionOptionsState::speedScale() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return speed_scale_;
}

void MotionOptionsState::setDefaultConfOpt(bool forced) {
  std::lock_guard<std::mutex> lock(mutex_);
  default_conf_opt_forced_ = forced;
}

bool MotionOptionsState::defaultConfOptForced() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return default_conf_opt_forced_;
}

void MotionOptionsState::setAvoidSingularity(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  avoid_singularity_enabled_ = enabled;
}

bool MotionOptionsState::avoidSingularityEnabled() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return avoid_singularity_enabled_;
}

void MotionOptionsState::setSoftLimit(bool enabled,
                                      const std::array<std::array<double, 2>, 6> &limits) {
  std::lock_guard<std::mutex> lock(mutex_);
  soft_limit_enabled_ = enabled;
  soft_limits_ = limits;
}

SoftLimitSnapshot MotionOptionsState::softLimit() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return SoftLimitSnapshot{soft_limit_enabled_, soft_limits_};
}

MotionRequestContext MotionOptionsState::makeMotionRequestContext(const std::string &request_id,
                                                                  const std::vector<double> &start_joints,
                                                                  double trajectory_dt) const {
  std::lock_guard<std::mutex> lock(mutex_);
  MotionRequestContext context;
  context.request_id = request_id;
  context.start_joints = start_joints;
  context.default_speed = default_speed_;
  context.default_zone = default_zone_;
  context.strict_conf = default_conf_opt_forced_;
  context.avoid_singularity = avoid_singularity_enabled_;
  context.soft_limit_enabled = soft_limit_enabled_;
  context.soft_limits = soft_limits_;
  context.speed_scale = speed_scale_;
  context.trajectory_dt = trajectory_dt;
  return context;
}

void ToolingState::setToolset(const std::string &tool_name,
                              const std::string &wobj_name,
                              const std::vector<double> &tool_pose,
                              const std::vector<double> &wobj_pose) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_tool_name_ = tool_name.empty() ? "tool0" : tool_name;
  current_wobj_name_ = wobj_name.empty() ? "wobj0" : wobj_name;
  current_tool_pose_ = normalize_runtime_pose(tool_pose);
  current_wobj_pose_ = normalize_runtime_pose(wobj_pose);
  tool_registry_[current_tool_name_] = current_tool_pose_;
  wobj_registry_[current_wobj_name_] = current_wobj_pose_;
  if (tool_mass_registry_.find(current_tool_name_) == tool_mass_registry_.end()) {
    tool_mass_registry_[current_tool_name_] = 0.0;
  }
  if (tool_com_registry_.find(current_tool_name_) == tool_com_registry_.end()) {
    tool_com_registry_[current_tool_name_] = {0.0, 0.0, 0.0};
  }
  current_tool_mass_ = tool_mass_registry_[current_tool_name_];
  current_tool_com_ = tool_com_registry_[current_tool_name_];
  base_pose_ = current_wobj_pose_;
}

bool ToolingState::setToolsetByName(const std::string &tool_name, const std::string &wobj_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto resolved_tool_name = tool_name.empty() ? current_tool_name_ : tool_name;
  const auto resolved_wobj_name = wobj_name.empty() ? current_wobj_name_ : wobj_name;
  const auto tool_it = tool_registry_.find(resolved_tool_name);
  const auto wobj_it = wobj_registry_.find(resolved_wobj_name);
  if (tool_it == tool_registry_.end() || wobj_it == wobj_registry_.end()) {
    return false;
  }
  current_tool_name_ = resolved_tool_name;
  current_wobj_name_ = resolved_wobj_name;
  current_tool_pose_ = tool_it->second;
  current_wobj_pose_ = wobj_it->second;
  current_tool_mass_ = tool_mass_registry_[current_tool_name_];
  current_tool_com_ = tool_com_registry_[current_tool_name_];
  base_pose_ = current_wobj_pose_;
  return true;
}

void ToolingState::setToolDynamics(const std::string &tool_name,
                                   double mass,
                                   const std::array<double, 3> &com) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string resolved_tool_name = tool_name.empty() ? current_tool_name_ : tool_name;
  tool_mass_registry_[resolved_tool_name] = std::max(0.0, mass);
  tool_com_registry_[resolved_tool_name] = com;
  if (resolved_tool_name == current_tool_name_) {
    current_tool_mass_ = tool_mass_registry_[resolved_tool_name];
    current_tool_com_ = tool_com_registry_[resolved_tool_name];
  }
}

ToolsetSnapshot ToolingState::toolset() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ToolsetSnapshot{
      current_tool_name_,
      current_wobj_name_,
      current_tool_pose_,
      current_wobj_pose_,
      base_pose_,
      current_tool_mass_,
      current_tool_com_,
  };
}

void ToolingState::setBaseFrame(const std::vector<double> &base_pose) {
  std::lock_guard<std::mutex> lock(mutex_);
  base_pose_ = normalize_runtime_pose(base_pose);
}

std::vector<double> ToolingState::baseFrame() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return base_pose_;
}

void DataStoreState::appendLog(const rokae_xmate3_ros2::msg::LogInfo &log) {
  std::lock_guard<std::mutex> lock(mutex_);
  log_buffer_.push_back(log);
  constexpr std::size_t kMaxLogBuffer = 128;
  if (log_buffer_.size() > kMaxLogBuffer) {
    log_buffer_.erase(log_buffer_.begin(), log_buffer_.begin() + (log_buffer_.size() - kMaxLogBuffer));
  }
}

std::vector<rokae_xmate3_ros2::msg::LogInfo> DataStoreState::queryLogs(unsigned int count) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto bounded = std::min<std::size_t>(count, log_buffer_.size());
  return std::vector<rokae_xmate3_ros2::msg::LogInfo>(log_buffer_.begin(), log_buffer_.begin() + bounded);
}

void DataStoreState::setRegister(const std::string &key, const std::string &value) {
  std::lock_guard<std::mutex> lock(mutex_);
  register_bank_[key] = value;
}

std::string DataStoreState::registerValue(const std::string &key) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = register_bank_.find(key);
  return it == register_bank_.end() ? std::string() : it->second;
}

void DataStoreState::setCustomData(const std::string &topic, const std::string &value) {
  std::lock_guard<std::mutex> lock(mutex_);
  custom_data_bank_[topic] = value;
  if (topic.rfind("register/", 0) == 0) {
    register_bank_[topic.substr(9)] = value;
  }
}

std::string DataStoreState::customData(const std::string &topic) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = custom_data_bank_.find(topic);
  return it == custom_data_bank_.end() ? std::string() : it->second;
}

bool DataStoreState::hasCallback(const std::string &callback_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return callback_registry_.count(callback_id) != 0;
}

void DataStoreState::registerCallback(const std::string &callback_id, const std::string &topic) {
  std::lock_guard<std::mutex> lock(mutex_);
  callback_registry_[callback_id] = topic;
}

bool DataStoreState::getDI(unsigned int board, unsigned int port) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = di_state_.find({board, port});
  return it == di_state_.end() ? false : it->second;
}

bool DataStoreState::getDO(unsigned int board, unsigned int port) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = do_state_.find({board, port});
  return it == do_state_.end() ? false : it->second;
}

void DataStoreState::setDI(unsigned int board, unsigned int port, bool state) {
  std::lock_guard<std::mutex> lock(mutex_);
  di_state_[{board, port}] = state;
}

void DataStoreState::setDO(unsigned int board, unsigned int port, bool state) {
  std::lock_guard<std::mutex> lock(mutex_);
  do_state_[{board, port}] = state;
}

double DataStoreState::getAI(unsigned int board, unsigned int port) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = ai_state_.find({board, port});
  return it == ai_state_.end() ? 0.0 : it->second;
}

double DataStoreState::getAO(unsigned int board, unsigned int port) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = ao_state_.find({board, port});
  return it == ao_state_.end() ? 0.0 : it->second;
}

void DataStoreState::setAI(unsigned int board, unsigned int port, double value) {
  std::lock_guard<std::mutex> lock(mutex_);
  ai_state_[{board, port}] = value;
}

void DataStoreState::setAO(unsigned int board, unsigned int port, double value) {
  std::lock_guard<std::mutex> lock(mutex_);
  ao_state_[{board, port}] = value;
}

void ProgramState::loadRlProject(const std::string &project_path, const std::string &project_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  loaded_rl_project_path_ = project_path;
  loaded_rl_project_name_ = project_name;
  rl_project_loaded_ = true;
  rl_project_running_ = false;
  rl_current_episode_ = 0;
}

bool ProgramState::rlProjectLoaded() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rl_project_loaded_;
}

bool ProgramState::rlProjectRunning() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rl_project_running_;
}

std::string ProgramState::loadedRlProjectName() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return loaded_rl_project_name_;
}

std::string ProgramState::loadedRlProjectPath() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return loaded_rl_project_path_;
}

int ProgramState::rlCurrentEpisode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rl_current_episode_;
}

void ProgramState::setRlProjectRunning(bool running, int current_episode) {
  std::lock_guard<std::mutex> lock(mutex_);
  rl_project_running_ = running;
  rl_current_episode_ = current_episode;
}

void ProgramState::startRecordingPath() {
  std::lock_guard<std::mutex> lock(mutex_);
  is_recording_path_ = true;
  recorded_path_.clear();
  record_time_origin_initialized_ = false;
  record_time_origin_sec_ = 0.0;
  record_created_at_sec_ = 0.0;
  recorded_toolset_ = ToolsetSnapshot{};
  record_source_ = "sdk_record";
}

void ProgramState::startRecordingPath(const ToolsetSnapshot &toolset, const std::string &source) {
  std::lock_guard<std::mutex> lock(mutex_);
  is_recording_path_ = true;
  recorded_path_.clear();
  record_time_origin_initialized_ = false;
  record_time_origin_sec_ = 0.0;
  record_created_at_sec_ = 0.0;
  recorded_toolset_ = toolset;
  record_source_ = source.empty() ? std::string("sdk_record") : source;
}

void ProgramState::stopRecordingPath() {
  std::lock_guard<std::mutex> lock(mutex_);
  is_recording_path_ = false;
}

void ProgramState::cancelRecordingPath() {
  std::lock_guard<std::mutex> lock(mutex_);
  is_recording_path_ = false;
  recorded_path_.clear();
  record_time_origin_initialized_ = false;
  record_created_at_sec_ = 0.0;
}

bool ProgramState::isRecordingPath() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return is_recording_path_;
}

void ProgramState::recordPathSample(double timestamp_sec,
                                    const std::array<double, 6> &joint_position,
                                    const std::array<double, 6> &joint_velocity) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_recording_path_) {
    return;
  }

  const double fallback_time = recorded_path_.empty() ? 0.0 : recorded_path_.back().time_from_start_sec + 0.01;
  const double absolute_timestamp = clamp_record_timestamp(timestamp_sec, fallback_time);
  if (!record_time_origin_initialized_) {
    record_time_origin_sec_ = absolute_timestamp;
    record_created_at_sec_ = absolute_timestamp;
    record_time_origin_initialized_ = true;
  }

  double relative_time = std::max(0.0, absolute_timestamp - record_time_origin_sec_);
  if (!recorded_path_.empty() && relative_time <= recorded_path_.back().time_from_start_sec) {
    relative_time = recorded_path_.back().time_from_start_sec + 0.01;
  }

  RecordedPathSample sample;
  sample.time_from_start_sec = relative_time;
  sample.joint_position = joint_position;
  sample.joint_velocity = joint_velocity;
  recorded_path_.push_back(sample);
}

void ProgramState::recordPathSample(const std::array<double, 6> &joint_position) {
  recordPathSample(std::numeric_limits<double>::quiet_NaN(), joint_position, std::array<double, 6>{});
}

void ProgramState::saveRecordedPath(const std::string &name) {
  std::lock_guard<std::mutex> lock(mutex_);
  ReplayPathAsset asset;
  asset.metadata.version = "v1";
  asset.metadata.robot = "xMate3";
  asset.metadata.source = record_source_;
  asset.metadata.created_at_sec = record_created_at_sec_;
  asset.samples = recorded_path_;
  asset.toolset = recorded_toolset_;
  asset.source = record_source_;
  saved_paths_[name] = std::move(asset);
}

bool ProgramState::getSavedPath(const std::string &name,
                                std::vector<std::vector<double>> &path) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = saved_paths_.find(name);
  if (it == saved_paths_.end()) {
    return false;
  }
  path.clear();
  path.reserve(it->second.samples.size());
  for (const auto &sample : it->second.samples) {
    path.emplace_back(sample.joint_position.begin(), sample.joint_position.end());
  }
  return true;
}

bool ProgramState::getReplayAsset(const std::string &name, ReplayPathAsset &asset) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = saved_paths_.find(name);
  if (it == saved_paths_.end()) {
    return false;
  }
  asset = it->second;
  return true;
}

void ProgramState::removeSavedPath(const std::string &name, bool remove_all) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (remove_all) {
    saved_paths_.clear();
    return;
  }
  saved_paths_.erase(name);
}

std::vector<std::string> ProgramState::querySavedPaths() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> names;
  names.reserve(saved_paths_.size());
  for (const auto &entry : saved_paths_) {
    names.push_back(entry.first);
  }
  return names;
}

ProgramSnapshot ProgramState::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  ProgramSnapshot snapshot;
  snapshot.rl_project_loaded = rl_project_loaded_;
  snapshot.rl_project_running = rl_project_running_;
  snapshot.rl_current_episode = rl_current_episode_;
  snapshot.recording_path = is_recording_path_;
  snapshot.loaded_rl_project_name = loaded_rl_project_name_;
  snapshot.loaded_rl_project_path = loaded_rl_project_path_;
  return snapshot;
}

}  // namespace rokae_xmate3_ros2::runtime
