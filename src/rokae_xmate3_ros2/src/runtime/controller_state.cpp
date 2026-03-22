#include "runtime/controller_state.hpp"

namespace rokae_xmate3_ros2::runtime {

ControllerState::ControllerState()
    : ControllerState(std::make_shared<SessionState>(),
                      std::make_shared<MotionOptionsState>(),
                      std::make_shared<ToolingState>(),
                      std::make_shared<DataStoreState>(),
                      std::make_shared<ProgramState>()) {}

ControllerState::ControllerState(std::shared_ptr<SessionState> session_state,
                                 std::shared_ptr<MotionOptionsState> motion_options_state,
                                 std::shared_ptr<ToolingState> tooling_state,
                                 std::shared_ptr<DataStoreState> data_store_state,
                                 std::shared_ptr<ProgramState> program_state)
    : session_state_(std::move(session_state)),
      motion_options_state_(std::move(motion_options_state)),
      tooling_state_(std::move(tooling_state)),
      data_store_state_(std::move(data_store_state)),
      program_state_(std::move(program_state)) {}

void ControllerState::connect(const std::string &remote_ip) { session_state_->connect(remote_ip); }
void ControllerState::disconnect() { session_state_->disconnect(); }

bool ControllerState::connected() const { return session_state_->connected(); }
std::string ControllerState::remoteIp() const { return session_state_->remoteIp(); }

void ControllerState::setPowerOn(bool on) { session_state_->setPowerOn(on); }
bool ControllerState::powerOn() const { return session_state_->powerOn(); }

void ControllerState::setDragMode(bool enabled) { session_state_->setDragMode(enabled); }
bool ControllerState::dragMode() const { return session_state_->dragMode(); }

void ControllerState::setSimulationMode(bool enabled) { session_state_->setSimulationMode(enabled); }
bool ControllerState::simulationMode() const { return session_state_->simulationMode(); }

void ControllerState::setCollisionDetectionEnabled(bool enabled) {
  session_state_->setCollisionDetectionEnabled(enabled);
}

bool ControllerState::collisionDetectionEnabled() const {
  return session_state_->collisionDetectionEnabled();
}

void ControllerState::setCollisionDetectionConfig(const std::array<double, 6> &sensitivity,
                                                  std::uint8_t behaviour,
                                                  double fallback) {
  session_state_->setCollisionDetectionConfig(sensitivity, behaviour, fallback);
}

CollisionDetectionSnapshot ControllerState::collisionDetection() const {
  return session_state_->collisionDetection();
}

void ControllerState::setMotionMode(int mode) { session_state_->setMotionMode(mode); }
int ControllerState::motionMode() const { return session_state_->motionMode(); }

void ControllerState::setOperateMode(uint8_t mode) { session_state_->setOperateMode(mode); }
rokae_xmate3_ros2::msg::OperateMode ControllerState::operateMode() const {
  return session_state_->operateMode();
}

void ControllerState::setDefaultSpeed(double speed) { motion_options_state_->setDefaultSpeed(speed); }
double ControllerState::defaultSpeed() const { return motion_options_state_->defaultSpeed(); }

void ControllerState::setDefaultZone(int zone) { motion_options_state_->setDefaultZone(zone); }
int ControllerState::defaultZone() const { return motion_options_state_->defaultZone(); }

void ControllerState::setSpeedScale(double scale) { motion_options_state_->setSpeedScale(scale); }
double ControllerState::speedScale() const { return motion_options_state_->speedScale(); }

void ControllerState::setDefaultConfOpt(bool forced) {
  motion_options_state_->setDefaultConfOpt(forced);
}

bool ControllerState::defaultConfOptForced() const {
  return motion_options_state_->defaultConfOptForced();
}

void ControllerState::setAvoidSingularity(bool enabled) {
  motion_options_state_->setAvoidSingularity(enabled);
}

bool ControllerState::avoidSingularityEnabled() const {
  return motion_options_state_->avoidSingularityEnabled();
}

void ControllerState::setRtControlMode(int mode) { session_state_->setRtControlMode(mode); }
int ControllerState::rtControlMode() const { return session_state_->rtControlMode(); }

void ControllerState::setSoftLimit(bool enabled, const std::array<std::array<double, 2>, 6> &limits) {
  motion_options_state_->setSoftLimit(enabled, limits);
}

SoftLimitSnapshot ControllerState::softLimit() const { return motion_options_state_->softLimit(); }

void ControllerState::setToolset(const std::string &tool_name,
                                 const std::string &wobj_name,
                                 const std::vector<double> &tool_pose,
                                 const std::vector<double> &wobj_pose) {
  tooling_state_->setToolset(tool_name, wobj_name, tool_pose, wobj_pose);
}

void ControllerState::setToolDynamics(const std::string &tool_name,
                                      double mass,
                                      const std::array<double, 3> &com) {
  tooling_state_->setToolDynamics(tool_name, mass, com);
}

ToolsetSnapshot ControllerState::toolset() const { return tooling_state_->toolset(); }

void ControllerState::appendLog(const rokae_xmate3_ros2::msg::LogInfo &log) {
  data_store_state_->appendLog(log);
}

std::vector<rokae_xmate3_ros2::msg::LogInfo> ControllerState::queryLogs(unsigned int count) const {
  return data_store_state_->queryLogs(count);
}

void ControllerState::setRegister(const std::string &key, const std::string &value) {
  data_store_state_->setRegister(key, value);
}

std::string ControllerState::registerValue(const std::string &key) const {
  return data_store_state_->registerValue(key);
}

void ControllerState::setCustomData(const std::string &topic, const std::string &value) {
  data_store_state_->setCustomData(topic, value);
}

std::string ControllerState::customData(const std::string &topic) const {
  return data_store_state_->customData(topic);
}

bool ControllerState::hasCallback(const std::string &callback_id) const {
  return data_store_state_->hasCallback(callback_id);
}

void ControllerState::registerCallback(const std::string &callback_id, const std::string &topic) {
  data_store_state_->registerCallback(callback_id, topic);
}

void ControllerState::loadRlProject(const std::string &project_path, const std::string &project_name) {
  program_state_->loadRlProject(project_path, project_name);
}

bool ControllerState::rlProjectLoaded() const { return program_state_->rlProjectLoaded(); }
bool ControllerState::rlProjectRunning() const { return program_state_->rlProjectRunning(); }
std::string ControllerState::loadedRlProjectName() const { return program_state_->loadedRlProjectName(); }
std::string ControllerState::loadedRlProjectPath() const { return program_state_->loadedRlProjectPath(); }
int ControllerState::rlCurrentEpisode() const { return program_state_->rlCurrentEpisode(); }
void ControllerState::setRlProjectRunning(bool running, int current_episode) {
  program_state_->setRlProjectRunning(running, current_episode);
}

bool ControllerState::getDI(unsigned int board, unsigned int port) const {
  return data_store_state_->getDI(board, port);
}

bool ControllerState::getDO(unsigned int board, unsigned int port) const {
  return data_store_state_->getDO(board, port);
}

void ControllerState::setDI(unsigned int board, unsigned int port, bool state) {
  data_store_state_->setDI(board, port, state);
}

void ControllerState::setDO(unsigned int board, unsigned int port, bool state) {
  data_store_state_->setDO(board, port, state);
}

double ControllerState::getAI(unsigned int board, unsigned int port) const {
  return data_store_state_->getAI(board, port);
}

double ControllerState::getAO(unsigned int board, unsigned int port) const {
  return data_store_state_->getAO(board, port);
}

void ControllerState::setAI(unsigned int board, unsigned int port, double value) {
  data_store_state_->setAI(board, port, value);
}

void ControllerState::setAO(unsigned int board, unsigned int port, double value) {
  data_store_state_->setAO(board, port, value);
}

void ControllerState::startRecordingPath() { program_state_->startRecordingPath(); }
void ControllerState::startRecordingPath(const ToolsetSnapshot &toolset, const std::string &source) {
  program_state_->startRecordingPath(toolset, source);
}
void ControllerState::stopRecordingPath() { program_state_->stopRecordingPath(); }
void ControllerState::cancelRecordingPath() { program_state_->cancelRecordingPath(); }
bool ControllerState::isRecordingPath() const { return program_state_->isRecordingPath(); }
void ControllerState::recordPathSample(double timestamp_sec,
                                       const std::array<double, 6> &joint_position,
                                       const std::array<double, 6> &joint_velocity) {
  program_state_->recordPathSample(timestamp_sec, joint_position, joint_velocity);
}
void ControllerState::recordPathSample(const std::array<double, 6> &joint_position) {
  program_state_->recordPathSample(joint_position);
}
void ControllerState::saveRecordedPath(const std::string &name) { program_state_->saveRecordedPath(name); }
bool ControllerState::getSavedPath(const std::string &name,
                                   std::vector<std::vector<double>> &path) const {
  return program_state_->getSavedPath(name, path);
}
bool ControllerState::getReplayAsset(const std::string &name, ReplayPathAsset &asset) const {
  return program_state_->getReplayAsset(name, asset);
}
void ControllerState::removeSavedPath(const std::string &name, bool remove_all) {
  program_state_->removeSavedPath(name, remove_all);
}
std::vector<std::string> ControllerState::querySavedPaths() const {
  return program_state_->querySavedPaths();
}

MotionRequestContext ControllerState::makeMotionRequestContext(const std::string &request_id,
                                                               const std::vector<double> &start_joints,
                                                               double trajectory_dt) const {
  return motion_options_state_->makeMotionRequestContext(request_id, start_joints, trajectory_dt);
}

OperationStateContext ControllerState::makeOperationStateContext() const {
  return session_state_->makeOperationStateContext(program_state_->rlProjectRunning());
}

SessionState &ControllerState::sessionState() const { return *session_state_; }
MotionOptionsState &ControllerState::motionOptionsState() const { return *motion_options_state_; }
ToolingState &ControllerState::toolingState() const { return *tooling_state_; }
DataStoreState &ControllerState::dataStoreState() const { return *data_store_state_; }
ProgramState &ControllerState::programState() const { return *program_state_; }

}  // namespace rokae_xmate3_ros2::runtime
