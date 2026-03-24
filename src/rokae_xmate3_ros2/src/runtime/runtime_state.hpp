#ifndef ROKAE_XMATE3_ROS2_RUNTIME_STATE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_STATE_HPP

#include <array>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "runtime/operation_state_adapter.hpp"
#include "runtime/request_adapter.hpp"
#include "rokae_xmate3_ros2/msg/log_info.hpp"
#include "rokae_xmate3_ros2/msg/operate_mode.hpp"

namespace rokae_xmate3_ros2::runtime {

struct ToolsetSnapshot {
  std::string tool_name;
  std::string wobj_name;
  std::vector<double> tool_pose;
  std::vector<double> wobj_pose;
  std::vector<double> base_pose;
  double tool_mass = 0.0;
  std::array<double, 3> tool_com{{0.0, 0.0, 0.0}};
};

struct CollisionDetectionSnapshot {
  bool enabled = false;
  std::array<double, 6> sensitivity{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  std::uint8_t behaviour = 1;
  double fallback = 0.0;
};

struct SoftLimitSnapshot {
  bool enabled = false;
  std::array<std::array<double, 2>, 6> limits{{
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
  }};
};

struct ProgramSnapshot {
  bool rl_project_loaded = false;
  bool rl_project_running = false;
  int rl_current_episode = 0;
  bool recording_path = false;
  std::string loaded_rl_project_name;
  std::string loaded_rl_project_path;
};

struct RecordedPathSample {
  double time_from_start_sec = 0.0;
  std::array<double, 6> joint_position{};
  std::array<double, 6> joint_velocity{};
};

struct ReplayPathAssetMetadata {
  std::string version{"v1"};
  std::string robot{"xMate3"};
  std::string source{"sdk_record"};
  double created_at_sec = 0.0;
};

struct ReplayPathAsset {
  ReplayPathAssetMetadata metadata;
  std::vector<RecordedPathSample> samples;
  ToolsetSnapshot toolset;
  std::string source{"sdk_record"};
};

class SessionState {
 public:
  void connect(const std::string &remote_ip);
  void disconnect();

  [[nodiscard]] bool connected() const;
  [[nodiscard]] std::string remoteIp() const;

  void setPowerOn(bool on);
  [[nodiscard]] bool powerOn() const;

  void setDragMode(bool enabled);
  [[nodiscard]] bool dragMode() const;

  void setSimulationMode(bool enabled);
  [[nodiscard]] bool simulationMode() const;

  void setCollisionDetectionEnabled(bool enabled);
  [[nodiscard]] bool collisionDetectionEnabled() const;
  void setCollisionDetectionConfig(const std::array<double, 6> &sensitivity,
                                   std::uint8_t behaviour,
                                   double fallback);
  [[nodiscard]] CollisionDetectionSnapshot collisionDetection() const;

  void setMotionMode(int mode);
  [[nodiscard]] int motionMode() const;

  void setOperateMode(uint8_t mode);
  [[nodiscard]] rokae_xmate3_ros2::msg::OperateMode operateMode() const;

  void setRtControlMode(int mode);
  [[nodiscard]] int rtControlMode() const;

  [[nodiscard]] OperationStateContext makeOperationStateContext(bool rl_project_running) const;

 private:
  mutable std::mutex mutex_;
  bool connected_ = false;
  bool power_on_ = false;
  bool drag_mode_ = false;
  bool simulation_mode_ = true;
  bool collision_detection_enabled_ = false;
  std::array<double, 6> collision_sensitivity_{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  std::uint8_t collision_behaviour_ = 1;
  double collision_fallback_ = 0.0;
  int motion_mode_ = 0;
  rokae_xmate3_ros2::msg::OperateMode operate_mode_{};
  int rt_control_mode_ = -1;
  std::string remote_ip_;
};

class MotionOptionsState {
 public:
  void setDefaultSpeed(double speed);
  [[nodiscard]] double defaultSpeed() const;

  void setDefaultZone(int zone);
  [[nodiscard]] int defaultZone() const;
  void setZoneValidRange(int min_zone, int max_zone);
  [[nodiscard]] std::array<int, 2> zoneValidRange() const;

  void setSpeedScale(double scale);
  [[nodiscard]] double speedScale() const;

  void setDefaultConfOpt(bool forced);
  [[nodiscard]] bool defaultConfOptForced() const;

  void setAvoidSingularity(bool enabled);
  [[nodiscard]] bool avoidSingularityEnabled() const;

  void setSoftLimit(bool enabled, const std::array<std::array<double, 2>, 6> &limits);
  [[nodiscard]] SoftLimitSnapshot softLimit() const;

  [[nodiscard]] MotionRequestContext makeMotionRequestContext(const std::string &request_id,
                                                              const std::vector<double> &start_joints,
                                                              double trajectory_dt) const;

 private:
  mutable std::mutex mutex_;
  double default_speed_ = 50.0;
  int default_zone_ = 0;
  int zone_valid_min_ = 0;
  int zone_valid_max_ = 200;
  double speed_scale_ = 1.0;
  bool default_conf_opt_forced_ = false;
  bool avoid_singularity_enabled_ = true;
  bool soft_limit_enabled_ = false;
  std::array<std::array<double, 2>, 6> soft_limits_{{
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
  }};
};

class ToolingState {
 public:
  void setToolset(const std::string &tool_name,
                  const std::string &wobj_name,
                  const std::vector<double> &tool_pose,
                  const std::vector<double> &wobj_pose);
  [[nodiscard]] bool setToolsetByName(const std::string &tool_name, const std::string &wobj_name);
  void setToolDynamics(const std::string &tool_name, double mass, const std::array<double, 3> &com);
  [[nodiscard]] ToolsetSnapshot toolset() const;
  void setBaseFrame(const std::vector<double> &base_pose);
  [[nodiscard]] std::vector<double> baseFrame() const;

 private:
  mutable std::mutex mutex_;
  std::string current_tool_name_{"tool0"};
  std::string current_wobj_name_{"wobj0"};
  std::vector<double> current_tool_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> current_wobj_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> base_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double current_tool_mass_ = 0.0;
  std::array<double, 3> current_tool_com_{{0.0, 0.0, 0.0}};
  std::map<std::string, std::vector<double>> tool_registry_{{"tool0", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}};
  std::map<std::string, std::vector<double>> wobj_registry_{{"wobj0", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}};
  std::map<std::string, double> tool_mass_registry_{{"tool0", 0.0}};
  std::map<std::string, std::array<double, 3>> tool_com_registry_{{"tool0", {{0.0, 0.0, 0.0}}}};
};

class DataStoreState {
 public:
  using IoKey = std::pair<unsigned int, unsigned int>;

  void appendLog(const rokae_xmate3_ros2::msg::LogInfo &log);
  [[nodiscard]] std::vector<rokae_xmate3_ros2::msg::LogInfo> queryLogs(unsigned int count) const;

  void setRegister(const std::string &key, const std::string &value);
  [[nodiscard]] std::string registerValue(const std::string &key) const;

  void setCustomData(const std::string &topic, const std::string &value);
  [[nodiscard]] std::string customData(const std::string &topic) const;

  [[nodiscard]] bool hasCallback(const std::string &callback_id) const;
  void registerCallback(const std::string &callback_id, const std::string &topic);

  [[nodiscard]] bool getDI(unsigned int board, unsigned int port) const;
  [[nodiscard]] bool getDO(unsigned int board, unsigned int port) const;
  void setDI(unsigned int board, unsigned int port, bool state);
  void setDO(unsigned int board, unsigned int port, bool state);

  [[nodiscard]] double getAI(unsigned int board, unsigned int port) const;
  [[nodiscard]] double getAO(unsigned int board, unsigned int port) const;
  void setAI(unsigned int board, unsigned int port, double value);
  void setAO(unsigned int board, unsigned int port, double value);

 private:
  mutable std::mutex mutex_;
  std::map<std::string, std::string> register_bank_;
  std::map<std::string, std::string> custom_data_bank_;
  std::map<std::string, std::string> callback_registry_;
  std::map<IoKey, bool> di_state_;
  std::map<IoKey, bool> do_state_;
  std::map<IoKey, double> ai_state_;
  std::map<IoKey, double> ao_state_;
  std::vector<rokae_xmate3_ros2::msg::LogInfo> log_buffer_;
};

class ProgramState {
 public:
  void loadRlProject(const std::string &project_path, const std::string &project_name);
  [[nodiscard]] bool rlProjectLoaded() const;
  [[nodiscard]] bool rlProjectRunning() const;
  [[nodiscard]] std::string loadedRlProjectName() const;
  [[nodiscard]] std::string loadedRlProjectPath() const;
  [[nodiscard]] int rlCurrentEpisode() const;
  void setRlProjectRunning(bool running, int current_episode);

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
