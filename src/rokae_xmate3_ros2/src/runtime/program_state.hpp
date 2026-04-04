#ifndef ROKAE_XMATE3_ROS2_RUNTIME_PROGRAM_STATE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_PROGRAM_STATE_HPP

#include <array>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "runtime/runtime_snapshots.hpp"
#include "rokae_xmate3_ros2/types.hpp"

namespace rokae_xmate3_ros2::runtime {

class ProgramState {
 public:
  void loadRlProject(const std::string &project_path, const std::string &project_name);
  [[nodiscard]] bool rlProjectLoaded() const;
  [[nodiscard]] bool rlProjectRunning() const;
  [[nodiscard]] std::string loadedRlProjectName() const;
  [[nodiscard]] std::string loadedRlProjectPath() const;
  [[nodiscard]] int rlCurrentEpisode() const;
  void setRlProjectRunning(bool running, int current_episode);
  void setRlProjectRunningOptions(double rate, bool loop_mode);
  [[nodiscard]] double rlRunRate() const;
  [[nodiscard]] bool rlLoopMode() const;
  [[nodiscard]] std::vector<rokae::RLProjectInfo> rlProjectCatalog() const;

  void startRecordingPath();
  void startRecordingPath(const ToolsetSnapshot &toolset, const std::string &source);
  void stopRecordingPath();
  void cancelRecordingPath();
  [[nodiscard]] bool isRecordingPath() const;
  void recordPathSample(double timestamp_sec,
                        const std::array<double, 6> &joint_position,
                        const std::array<double, 6> &joint_velocity);
  void recordPathSample(const std::array<double, 6> &joint_position);
  void saveRecordedPath(const std::string &name);
  [[nodiscard]] bool hasRecordedPathData() const;
  [[nodiscard]] bool renameSavedPath(const std::string &name, const std::string &save_as);
  [[nodiscard]] bool getSavedPath(const std::string &name,
                                  std::vector<std::vector<double>> &path) const;
  [[nodiscard]] bool getReplayAsset(const std::string &name, ReplayPathAsset &asset) const;
  void removeSavedPath(const std::string &name, bool remove_all);
  [[nodiscard]] std::vector<std::string> querySavedPaths() const;
  [[nodiscard]] ProgramSnapshot snapshot() const;

 private:
  mutable std::mutex mutex_;
  bool rl_project_loaded_ = false;
  bool rl_project_running_ = false;
  int rl_current_episode_ = 0;
  double rl_run_rate_ = 1.0;
  bool rl_loop_mode_ = false;
  std::string loaded_rl_project_name_;
  std::string loaded_rl_project_path_;
  bool is_recording_path_ = false;
  bool record_time_origin_initialized_ = false;
  double record_time_origin_sec_ = 0.0;
  double record_created_at_sec_ = 0.0;
  ToolsetSnapshot recorded_toolset_;
  std::string record_source_{"sdk_record"};
  std::vector<RecordedPathSample> recorded_path_;
  std::map<std::string, ReplayPathAsset> saved_paths_;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
