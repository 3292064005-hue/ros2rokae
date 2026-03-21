#ifndef ROKAE_XMATE3_ROS2_RUNTIME_CONTROLLER_STATE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_CONTROLLER_STATE_HPP

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "runtime/runtime_state.hpp"

namespace rokae_xmate3_ros2::runtime {

class ControllerState {
 public:
  ControllerState();
  ControllerState(std::shared_ptr<SessionState> session_state,
                  std::shared_ptr<MotionOptionsState> motion_options_state,
                  std::shared_ptr<ToolingState> tooling_state,
                  std::shared_ptr<DataStoreState> data_store_state,
                  std::shared_ptr<ProgramState> program_state);

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

  void setDefaultSpeed(double speed);
  [[nodiscard]] double defaultSpeed() const;

  void setDefaultZone(int zone);
  [[nodiscard]] int defaultZone() const;

  void setSpeedScale(double scale);
  [[nodiscard]] double speedScale() const;

  void setDefaultConfOpt(bool forced);
  [[nodiscard]] bool defaultConfOptForced() const;

  void setAvoidSingularity(bool enabled);
  [[nodiscard]] bool avoidSingularityEnabled() const;

  void setRtControlMode(int mode);
  [[nodiscard]] int rtControlMode() const;

  void setSoftLimit(bool enabled, const std::array<std::array<double, 2>, 6> &limits);
  [[nodiscard]] SoftLimitSnapshot softLimit() const;

  void setToolset(const std::string &tool_name,
                  const std::string &wobj_name,
                  const std::vector<double> &tool_pose,
                  const std::vector<double> &wobj_pose);
  [[nodiscard]] ToolsetSnapshot toolset() const;

  void appendLog(const rokae_xmate3_ros2::msg::LogInfo &log);
  [[nodiscard]] std::vector<rokae_xmate3_ros2::msg::LogInfo> queryLogs(unsigned int count) const;

  void setRegister(const std::string &key, const std::string &value);
  [[nodiscard]] std::string registerValue(const std::string &key) const;
  void setCustomData(const std::string &topic, const std::string &value);
  [[nodiscard]] std::string customData(const std::string &topic) const;
  [[nodiscard]] bool hasCallback(const std::string &callback_id) const;
  void registerCallback(const std::string &callback_id, const std::string &topic);

  void loadRlProject(const std::string &project_path, const std::string &project_name);
  [[nodiscard]] bool rlProjectLoaded() const;
  [[nodiscard]] bool rlProjectRunning() const;
  [[nodiscard]] std::string loadedRlProjectName() const;
  [[nodiscard]] std::string loadedRlProjectPath() const;
  [[nodiscard]] int rlCurrentEpisode() const;
  void setRlProjectRunning(bool running, int current_episode);

  [[nodiscard]] bool getDI(unsigned int board, unsigned int port) const;
  [[nodiscard]] bool getDO(unsigned int board, unsigned int port) const;
  void setDI(unsigned int board, unsigned int port, bool state);
  void setDO(unsigned int board, unsigned int port, bool state);
  [[nodiscard]] double getAI(unsigned int board, unsigned int port) const;
  [[nodiscard]] double getAO(unsigned int board, unsigned int port) const;
  void setAI(unsigned int board, unsigned int port, double value);
  void setAO(unsigned int board, unsigned int port, double value);

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

  [[nodiscard]] MotionRequestContext makeMotionRequestContext(const std::string &request_id,
                                                              const std::vector<double> &start_joints,
                                                              double trajectory_dt) const;
  [[nodiscard]] OperationStateContext makeOperationStateContext() const;

  [[nodiscard]] SessionState &sessionState() const;
  [[nodiscard]] MotionOptionsState &motionOptionsState() const;
  [[nodiscard]] ToolingState &toolingState() const;
  [[nodiscard]] DataStoreState &dataStoreState() const;
  [[nodiscard]] ProgramState &programState() const;

 private:
  std::shared_ptr<SessionState> session_state_;
  std::shared_ptr<MotionOptionsState> motion_options_state_;
  std::shared_ptr<ToolingState> tooling_state_;
  std::shared_ptr<DataStoreState> data_store_state_;
  std::shared_ptr<ProgramState> program_state_;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
