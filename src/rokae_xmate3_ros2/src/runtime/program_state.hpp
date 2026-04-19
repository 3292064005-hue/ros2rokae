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
  /**
   * @brief Load the currently selected RL project metadata into the runtime cache.
   * @param project_path Filesystem path recorded as the current project origin.
   * @param project_name Human-readable project name exposed through query services.
   * @note Public xMate6 lane does not expose RL as a supported contract, but the runtime still
   *       tracks this state for internal compatibility paths.
   */
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

  /**
   * @brief Begin capturing a replayable xMate6 recorded-path asset using default tooling context.
   * @note Captured samples are reset on every start; timestamps are re-originated from the first
   *       accepted sample.
   */
  void startRecordingPath();
  /**
   * @brief Begin capturing a replayable recorded-path asset with explicit tooling provenance.
   * @param toolset Tool/work-object/base-frame snapshot stored into the replay asset metadata.
   * @param source Stable producer label used for audit and replay diagnostics. Empty values fall
   *        back to `sdk_record`.
   */
  void startRecordingPath(const ToolsetSnapshot &toolset, const std::string &source);
  /// Stop accepting new samples while keeping the captured buffer available for save/replay.
  void stopRecordingPath();
  /// Abort recording and discard all pending captured samples.
  void cancelRecordingPath();
  [[nodiscard]] bool isRecordingPath() const;
  /**
   * @brief Append one recorded-path sample.
   * @param timestamp_sec Absolute capture timestamp in seconds. NaN or non-monotonic values are
   *        sanitized into a strictly increasing local timeline using kRecordedPathMonotonicStepSec.
   * @param joint_position Six-axis joint position sample in radians.
   * @param joint_velocity Six-axis joint velocity sample in radians per second.
   * @note Boundary behavior: samples are ignored when recording is not active.
   */
  void recordPathSample(double timestamp_sec,
                        const std::array<double, 6> &joint_position,
                        const std::array<double, 6> &joint_velocity);
  /// Convenience overload that records positions with zero velocity when explicit velocity is unavailable.
  void recordPathSample(const std::array<double, 6> &joint_position);
  /**
   * @brief Seal the current capture buffer into a named replay asset.
   * @param name Stable asset name used by replay/query/remove operations.
   * @param error_message Optional validation error sink. Populated when the save is rejected.
   * @return true when the capture buffer was normalized, validated, and stored successfully.
   * @note Saved assets carry the public xMate6 compatibility identity while preserving the
   *       underlying simulated xMate3 model provenance. Empty names, empty buffers, or assets
   *       that fail replay contract validation are rejected. Analysis/report readiness is computed separately from the saved asset coverage report.
   */
  [[nodiscard]] bool saveRecordedPath(const std::string &name, std::string *error_message = nullptr);
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
