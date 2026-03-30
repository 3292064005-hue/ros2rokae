#include "runtime/runtime_state.hpp"

#include <algorithm>
#include <limits>

#include "runtime/runtime_state_utils.hpp"

namespace rokae_xmate3_ros2::runtime {

void ProgramState::loadRlProject(const std::string &project_path, const std::string &project_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  loaded_rl_project_path_ = project_path;
  loaded_rl_project_name_ = project_name;
  rl_project_loaded_ = true;
  rl_project_running_ = false;
  rl_current_episode_ = 0;
  rl_run_rate_ = 1.0;
  rl_loop_mode_ = false;
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

void ProgramState::setRlProjectRunningOptions(double rate, bool loop_mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  rl_run_rate_ = std::max(0.0, rate);
  rl_loop_mode_ = loop_mode;
}

double ProgramState::rlRunRate() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rl_run_rate_;
}

bool ProgramState::rlLoopMode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rl_loop_mode_;
}

std::vector<rokae::RLProjectInfo> ProgramState::rlProjectCatalog() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rokae::RLProjectInfo> projects;
  if (!loaded_rl_project_name_.empty()) {
    rokae::RLProjectInfo info;
    info.name = loaded_rl_project_name_;
    info.is_running = rl_project_running_;
    info.run_rate = rl_run_rate_;
    info.loop_mode = rl_loop_mode_;
    projects.push_back(info);
  }
  return projects;
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
  const double absolute_timestamp = detail::clamp_record_timestamp(timestamp_sec, fallback_time);
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
