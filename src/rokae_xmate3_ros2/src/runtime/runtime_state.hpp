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
  int motion_mode_ = 0;
  rokae_xmate3_ros2::msg::OperateMode operate_mode_{};
  int rt_control_mode_ = -1;
  std::string remote_ip_;
};

class MotionOptionsState {
 public:
  void setDefaultSpeed(int speed);
  [[nodiscard]] int defaultSpeed() const;

  void setDefaultZone(int zone);
  [[nodiscard]] int defaultZone() const;

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
  int default_speed_ = 50;
  int default_zone_ = 0;
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
  [[nodiscard]] ToolsetSnapshot toolset() const;

 private:
  mutable std::mutex mutex_;
  std::string current_tool_name_;
  std::string current_wobj_name_;
  std::vector<double> current_tool_pose_;
  std::vector<double> current_wobj_pose_;
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
  void stopRecordingPath();
  void cancelRecordingPath();
  [[nodiscard]] bool isRecordingPath() const;
  void recordPathSample(const std::array<double, 6> &joint_position);
  void saveRecordedPath(const std::string &name);
  [[nodiscard]] bool getSavedPath(const std::string &name,
                                  std::vector<std::vector<double>> &path) const;
  void removeSavedPath(const std::string &name, bool remove_all);
  [[nodiscard]] std::vector<std::string> querySavedPaths() const;

 private:
  mutable std::mutex mutex_;
  bool rl_project_loaded_ = false;
  bool rl_project_running_ = false;
  int rl_current_episode_ = 0;
  std::string loaded_rl_project_name_;
  std::string loaded_rl_project_path_;
  bool is_recording_path_ = false;
  std::vector<std::vector<double>> recorded_path_;
  std::map<std::string, std::vector<std::vector<double>>> saved_paths_;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
