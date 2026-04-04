#include "rokae/robot.h"
#include "compat/internal/compat_shared.hpp"

#include <algorithm>
#include <sstream>
#include <utility>

#include "rokae_xmate3_ros2/runtime/rt_semantic_topics.hpp"

namespace rokae {
namespace {

std::shared_ptr<detail::CompatRobotHandle> make_handle() {
  return std::make_shared<detail::CompatRobotHandle>();
}

std::shared_ptr<detail::CompatRobotHandle> make_handle(const std::string &remote,
                                                       const std::string &local) {
  return std::make_shared<detail::CompatRobotHandle>(remote, local);
}

std::string serialize_bool(bool value) {
  return value ? "1" : "0";
}

std::string serialize_numeric_array(const std::array<double, 6> &values) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      oss << ',';
    }
    oss << values[i];
  }
  return oss.str();
}

std::string serialize_matrix16(const std::array<double, 16> &values) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      oss << ',';
    }
    oss << values[i];
  }
  return oss.str();
}

std::string serialize_load(const Load &load) {
  std::ostringstream oss;
  oss << "mass=" << load.mass << ";cog=" << load.cog[0] << ',' << load.cog[1] << ',' << load.cog[2]
      << ";inertia=" << load.inertia[0] << ',' << load.inertia[1] << ',' << load.inertia[2];
  return oss.str();
}

bool has_remote_endpoint(const std::shared_ptr<detail::CompatRobotHandle> &handle) {
  if (!handle) {
    return false;
  }
  std::lock_guard<std::mutex> lock(handle->mutex);
  return !handle->remote_ip.empty();
}


}  // namespace

BaseRobot::BaseRobot()
    : handle_(make_handle()) {}

BaseRobot::BaseRobot(const std::string &remoteIP, const std::string &localIP)
    : handle_(make_handle(remoteIP, localIP)) {}

BaseRobot::BaseRobot(std::shared_ptr<detail::CompatRobotHandle> handle)
    : handle_(std::move(handle)) {}

BaseRobot::BaseRobot(BaseRobot &&other) noexcept
    : handle_(std::move(other.handle_)) {
  if (!handle_) {
    handle_ = make_handle();
  }
  other.handle_ = make_handle();
}

BaseRobot &BaseRobot::operator=(BaseRobot &&other) noexcept {
  if (this != &other) {
    handle_ = std::move(other.handle_);
    if (!handle_) {
      handle_ = make_handle();
    }
    other.handle_ = make_handle();
  }
  return *this;
}

BaseRobot::~BaseRobot() = default;
BaseCobot::~BaseCobot() = default;
Cobot<6>::~Cobot() = default;
Robot_T<WorkType::collaborative, 6>::~Robot_T() = default;
xMateRobot::~xMateRobot() = default;

Cobot<6>::Cobot()
    : BaseCobot(make_handle()) {}

Cobot<6>::Cobot(const std::string &remoteIP, const std::string &localIP)
    : BaseCobot(make_handle(remoteIP, localIP)) {}

Robot_T<WorkType::collaborative, 6>::Robot_T()
    : Cobot<6>() {}

Robot_T<WorkType::collaborative, 6>::Robot_T(const std::string &remoteIP, const std::string &localIP)
    : Cobot<6>(remoteIP, localIP) {
  error_code ec;
  connectToRobot(ec);
  throw_if_error<ExecutionException>(ec, "Robot_T::Robot_T(connectToRobot)");
}

xMateRobot::xMateRobot() = default;

xMateRobot::xMateRobot(const std::string &remoteIP, const std::string &localIP)
    : Robot_T<WorkType::collaborative, 6>(remoteIP, localIP) {}

std::string BaseRobot::sdkVersion() {
  return rokae::ros2::xMateRobot::sdkVersion();
}

void BaseRobot::connectToRobot(error_code &ec) noexcept {
  if (!handle_ || !handle_->backend) {
    ec = std::make_error_code(std::errc::not_connected);
    return;
  }
  if (!has_remote_endpoint(handle_)) {
    ec = std::make_error_code(std::errc::invalid_argument);
    handle_->backend->rememberCompatError(ec);
    return;
  }
  handle_->backend->connectToRobot(ec);
}

void BaseRobot::connectToRobot(const std::string &remoteIP,
                               const std::string &localIP,
                               error_code &ec) noexcept {
  if (remoteIP.empty()) {
    ec = std::make_error_code(std::errc::invalid_argument);
    if (handle_ && handle_->backend) {
      handle_->backend->rememberCompatError(ec);
    }
    return;
  }
  if (!handle_) {
    handle_ = make_handle(remoteIP, localIP);
  } else {
    handle_->resetBackend(remoteIP, localIP);
  }
  handle_->backend->connectToRobot(ec);
}

void BaseRobot::connectToRobot(const std::string &remoteIP, const std::string &localIP) {
  error_code ec;
  connectToRobot(remoteIP, localIP, ec);
  throw_if_error<ExecutionException>(ec, "connectToRobot");
}

void BaseRobot::disconnectFromRobot(error_code &ec) noexcept {
  if (!handle_ || !handle_->backend) {
    ec = std::make_error_code(std::errc::not_connected);
    return;
  }
  handle_->backend->disconnectFromRobot(ec);
}

Info BaseRobot::robotInfo(error_code &ec) const noexcept { return handle_->backend->robotInfo(ec); }
PowerState BaseRobot::powerState(error_code &ec) const noexcept { return handle_->backend->powerState(ec); }
void BaseRobot::setPowerState(bool on, error_code &ec) noexcept { handle_->backend->setPowerState(on, ec); }
OperateMode BaseRobot::operateMode(error_code &ec) const noexcept { return handle_->backend->operateMode(ec); }
void BaseRobot::setOperateMode(OperateMode mode, error_code &ec) noexcept { handle_->backend->setOperateMode(mode, ec); }
OperationState BaseRobot::operationState(error_code &ec) const noexcept { return handle_->backend->operationState(ec); }
std::array<double, 6> BaseRobot::jointPos(error_code &ec) const noexcept { return handle_->backend->jointPos(ec); }
std::array<double, 6> BaseRobot::jointVel(error_code &ec) const noexcept { return handle_->backend->jointVel(ec); }
std::array<double, 6> BaseRobot::jointTorque(error_code &ec) const noexcept { return handle_->backend->jointTorques(ec); }
std::array<double, 6> BaseRobot::jointTorques(error_code &ec) const noexcept { return jointTorque(ec); }
std::array<double, 6> BaseRobot::posture(CoordinateType ct, error_code &ec) const noexcept { return handle_->backend->posture(ct, ec); }
CartesianPosition BaseRobot::cartPosture(CoordinateType ct, error_code &ec) const noexcept { return handle_->backend->cartPosture(ct, ec); }
std::array<double, 6> BaseRobot::flangePos(error_code &ec) const noexcept { return posture(CoordinateType::flangeInBase, ec); }
std::array<double, 6> BaseRobot::baseFrame(error_code &ec) const noexcept { return handle_->backend->baseFrame(ec); }

Toolset BaseRobot::toolset(error_code &ec) const noexcept {
  auto value = handle_->backend->toolset(ec);
  if (!ec) {
    std::lock_guard<std::mutex> lock(handle_->mutex);
    handle_->toolset_cache = value;
    handle_->model_load_cache = value.load;
    handle_->model_f_t_ee = value.end.pos;
  }
  return value;
}

void BaseRobot::setToolset(const Toolset &toolset_value, error_code &ec) noexcept {
  handle_->backend->setToolset(toolset_value, ec);
  if (!ec) {
    std::lock_guard<std::mutex> lock(handle_->mutex);
    handle_->toolset_cache = toolset_value;
    handle_->model_load_cache = toolset_value.load;
    handle_->model_f_t_ee = toolset_value.end.pos;
  }
}

Toolset BaseRobot::setToolset(const std::string &toolName, const std::string &wobjName, error_code &ec) noexcept {
  handle_->backend->setToolset(toolName, wobjName, ec);
  if (!ec) {
    return toolset(ec);
  }
  return {};
}

void BaseRobot::clearServoAlarm(error_code &ec) noexcept { handle_->backend->clearServoAlarm(ec); }

std::vector<LogInfo> BaseRobot::queryControllerLog(unsigned count,
                                                   const std::set<LogInfo::Level> &level,
                                                   error_code &ec) const noexcept {
  auto logs = handle_->backend->queryControllerLog(count, ec);
  if (ec || level.empty()) {
    return logs;
  }
  std::vector<LogInfo> filtered;
  filtered.reserve(logs.size());
  for (const auto &entry : logs) {
    const auto lvl = static_cast<LogInfo::Level>(entry.level);
    if (level.find(lvl) != level.end()) {
      filtered.push_back(entry);
    }
  }
  return filtered;
}

void BaseRobot::setMotionControlMode(MotionControlMode mode, error_code &ec) noexcept { handle_->backend->setMotionControlMode(mode, ec); }
void BaseRobot::moveReset(error_code &ec) noexcept { handle_->backend->moveReset(ec); }
void BaseRobot::stop(error_code &ec) noexcept { handle_->backend->stop(ec); }
void BaseRobot::moveStart(error_code &ec) noexcept { handle_->backend->moveStart(ec); }
void BaseRobot::setDefaultSpeed(int speed, error_code &ec) noexcept { handle_->backend->setDefaultSpeed(speed, ec); }
void BaseRobot::setDefaultZone(int zone, error_code &ec) noexcept { handle_->backend->setDefaultZone(zone, ec); }
void BaseRobot::setDefaultConfOpt(bool forced, error_code &ec) noexcept { handle_->backend->setDefaultConfOpt(forced, ec); }
void BaseRobot::adjustSpeedOnline(double scale, error_code &ec) noexcept { handle_->backend->adjustSpeedOnline(scale, ec); }
void BaseRobot::setEventWatcher(Event eventType, const EventCallback &callback, error_code &ec) noexcept { handle_->backend->setEventWatcher(eventType, callback, ec); }
EventInfo BaseRobot::queryEventInfo(Event eventType, error_code &ec) noexcept { return handle_->backend->queryEventInfo(eventType, ec); }
void BaseRobot::setMaxCacheSize(int number, error_code &ec) noexcept { handle_->backend->setMaxCacheSize(number, ec); }
std::error_code BaseRobot::lastErrorCode() noexcept { return handle_->backend->lastErrorCode(); }

void BaseRobot::moveAppend(const std::vector<MoveAbsJCommand> &cmds, std::string &cmdID, error_code &ec) noexcept { handle_->backend->moveAppend(cmds, cmdID, ec); }
void BaseRobot::moveAppend(const std::vector<MoveJCommand> &cmds, std::string &cmdID, error_code &ec) noexcept { handle_->backend->moveAppend(cmds, cmdID, ec); }
void BaseRobot::moveAppend(const std::vector<MoveLCommand> &cmds, std::string &cmdID, error_code &ec) noexcept { handle_->backend->moveAppend(cmds, cmdID, ec); }
void BaseRobot::moveAppend(const std::vector<MoveCCommand> &cmds, std::string &cmdID, error_code &ec) noexcept { handle_->backend->moveAppend(cmds, cmdID, ec); }
void BaseRobot::moveAppend(const std::vector<MoveCFCommand> &cmds, std::string &cmdID, error_code &ec) noexcept { handle_->backend->moveAppend(cmds, cmdID, ec); }
void BaseRobot::moveAppend(const std::vector<MoveSPCommand> &cmds, std::string &cmdID, error_code &ec) noexcept { handle_->backend->moveAppend(cmds, cmdID, ec); }
void BaseRobot::executeCommand(const std::vector<MoveAbsJCommand> &cmds, error_code &ec) noexcept { handle_->backend->executeCommand(cmds, ec); }
void BaseRobot::executeCommand(const std::vector<MoveJCommand> &cmds, error_code &ec) noexcept { handle_->backend->executeCommand(cmds, ec); }
void BaseRobot::executeCommand(const std::vector<MoveLCommand> &cmds, error_code &ec) noexcept { handle_->backend->executeCommand(cmds, ec); }
void BaseRobot::executeCommand(const std::vector<MoveCCommand> &cmds, error_code &ec) noexcept { handle_->backend->executeCommand(cmds, ec); }

/**
 * @brief Public-lane IO/query entry points remain for source compatibility only.
 * @details The install-facing xMate6 compatibility lane intentionally excludes IO, registers, RL, calibration,
 *          and singularity-avoid workflows. These wrappers therefore return deterministic `not_implemented`
 *          errors instead of silently dispatching to backend-only services.
 */
bool BaseRobot::getDI(unsigned board, unsigned port, error_code &ec) noexcept { (void)board; (void)port; detail::setPublicLaneUnsupported(handle_, ec, "io"); return false; }
bool BaseRobot::getDO(unsigned board, unsigned port, error_code &ec) noexcept { (void)board; (void)port; detail::setPublicLaneUnsupported(handle_, ec, "io"); return false; }
void BaseRobot::setDI(unsigned board, unsigned port, bool state, error_code &ec) noexcept { (void)board; (void)port; (void)state; detail::setPublicLaneUnsupported(handle_, ec, "io"); }
void BaseRobot::setDO(unsigned board, unsigned port, bool state, error_code &ec) noexcept { (void)board; (void)port; (void)state; detail::setPublicLaneUnsupported(handle_, ec, "io"); }
double BaseRobot::getAI(unsigned board, unsigned port, error_code &ec) noexcept { (void)board; (void)port; detail::setPublicLaneUnsupported(handle_, ec, "io"); return 0.0; }
void BaseRobot::setAO(unsigned board, unsigned port, double value, error_code &ec) noexcept { (void)board; (void)port; (void)value; detail::setPublicLaneUnsupported(handle_, ec, "io"); }
void BaseRobot::setSimulationMode(bool state, error_code &ec) noexcept { (void)state; detail::setPublicLaneUnsupported(handle_, ec, "io"); }
void BaseRobot::setxPanelVout(xPanelOpt::Vout opt, error_code &ec) noexcept { (void)opt; detail::setPublicLaneUnsupported(handle_, ec, "io"); }

std::string BaseRobot::readRegisterRaw(const std::string &name, unsigned index, error_code &ec) noexcept {
  (void)name; (void)index; detail::setPublicLaneUnsupported(handle_, ec, "io/register"); return {};
}

void BaseRobot::writeRegisterRaw(const std::string &name, unsigned index, const std::string &value, error_code &ec) noexcept {
  (void)name; (void)index; (void)value; detail::setPublicLaneUnsupported(handle_, ec, "io/register");
}

std::vector<RLProjectInfo> BaseRobot::projectsInfo(error_code &ec) noexcept {
  detail::setPublicLaneUnsupported(handle_, ec, "rl");
  return {};
}

void BaseRobot::loadProject(const std::string &name, const std::vector<std::string> &tasks, error_code &ec) noexcept { (void)name; (void)tasks; detail::setPublicLaneUnsupported(handle_, ec, "rl"); }

void BaseRobot::ppToMain(error_code &ec) noexcept { detail::setPublicLaneUnsupported(handle_, ec, "rl"); }
void BaseRobot::runProject(error_code &ec) noexcept { detail::setPublicLaneUnsupported(handle_, ec, "rl"); }
void BaseRobot::pauseProject(error_code &ec) noexcept { detail::setPublicLaneUnsupported(handle_, ec, "rl"); }
void BaseRobot::setProjectRunningOpt(double rate, bool loop, error_code &ec) noexcept { (void)rate; (void)loop; detail::setPublicLaneUnsupported(handle_, ec, "rl"); }
std::vector<WorkToolInfo> BaseRobot::toolsInfo(error_code &ec) noexcept { detail::setPublicLaneUnsupported(handle_, ec, "rl"); return {}; }
std::vector<WorkToolInfo> BaseRobot::wobjsInfo(error_code &ec) noexcept { detail::setPublicLaneUnsupported(handle_, ec, "rl"); return {}; }

void BaseRobot::startReceiveRobotState(std::chrono::steady_clock::duration interval,
                                       const std::vector<std::string> &fields) {
  handle_->backend->startReceiveRobotState(interval, fields);
}

void BaseRobot::stopReceiveRobotState() noexcept { handle_->backend->stopReceiveRobotState(); }
unsigned BaseRobot::updateRobotState(std::chrono::steady_clock::duration timeout) { return handle_->backend->updateRobotState(timeout); }
int BaseRobot::getStateDataArray6(const std::string &fieldName, std::array<double, 6> &data) noexcept { return handle_->backend->getStateDataArray6(fieldName, data); }
int BaseRobot::getStateDataArray3(const std::string &fieldName, std::array<double, 3> &data) noexcept { return handle_->backend->getStateDataArray3(fieldName, data); }
int BaseRobot::getStateDataMatrix16(const std::string &fieldName, std::array<double, 16> &data) noexcept { return handle_->backend->getStateDataMatrix16(fieldName, data); }
int BaseRobot::getStateDataScalarDouble(const std::string &fieldName, double &data) noexcept { return handle_->backend->getStateDataScalarDouble(fieldName, data); }
int BaseRobot::getStateDataBool(const std::string &fieldName, bool &data) noexcept { return handle_->backend->getStateDataBool(fieldName, data); }
bool BaseRobot::getSoftLimit(std::array<double[2], 6> &limits, error_code &ec) noexcept { return handle_->backend->getSoftLimit(limits, ec); }
void BaseRobot::setSoftLimit(bool enable, error_code &ec, const std::array<double[2], 6> &limits) noexcept { handle_->backend->setSoftLimit(enable, ec, limits); }
bool BaseRobot::getSoftLimit(std::array<std::array<double, 2>, 6> &limits, error_code &ec) noexcept { return handle_->backend->getSoftLimit(limits, ec); }
void BaseRobot::setSoftLimit(bool enable, error_code &ec, const std::array<std::array<double, 2>, 6> &limits) noexcept { handle_->backend->setSoftLimit(enable, ec, limits); }
FrameCalibrationResult BaseRobot::calibrateFrame(FrameType type, const std::vector<std::array<double, 6>> &points, bool is_held, error_code &ec, const std::array<double, 3> &base_aux) noexcept { (void)type; (void)points; (void)is_held; (void)base_aux; detail::setPublicLaneUnsupported(handle_, ec, "calibration"); return {}; }

void BaseCobot::enableCollisionDetection(const std::array<double, 6> &sensitivity,
                                         StopLevel behaviour,
                                         double fallback_compliance,
                                         error_code &ec) noexcept {
  handle_->backend->enableCollisionDetection(sensitivity, behaviour, fallback_compliance, ec);
}

void BaseCobot::disableCollisionDetection(error_code &ec) noexcept { handle_->backend->disableCollisionDetection(ec); }
void BaseCobot::enableDrag(DragParameter::Space space, DragParameter::Type type, error_code &ec) noexcept { handle_->backend->enableDrag(space, type, ec); }
void BaseCobot::disableDrag(error_code &ec) noexcept { handle_->backend->disableDrag(ec); }
void BaseCobot::startJog(JogOpt::Space space, double rate, double step, unsigned index, bool direction, error_code &ec) noexcept { handle_->backend->startJog(space, rate, step, index, direction, ec); }
void BaseCobot::setAvoidSingularity(bool enable, error_code &ec) noexcept { (void)enable; detail::setPublicLaneUnsupported(handle_, ec, "avoid_singularity"); }
bool BaseCobot::getAvoidSingularity(error_code &ec) noexcept { detail::setPublicLaneUnsupported(handle_, ec, "avoid_singularity"); return false; }
void BaseCobot::getEndTorque(FrameType ref_type, std::array<double, 6> &joint_torque_measured, std::array<double, 6> &external_torque_measured, std::array<double, 3> &cart_torque, std::array<double, 3> &cart_force, error_code &ec) noexcept { handle_->backend->getEndTorque(ref_type, joint_torque_measured, external_torque_measured, cart_torque, cart_force, ec); }
void BaseCobot::startRecordPath(std::chrono::seconds duration, error_code &ec) noexcept { handle_->backend->startRecordPath(duration, ec); }
void BaseCobot::startRecordPath(int duration_seconds, error_code &ec) noexcept {
  if (duration_seconds < 0) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return;
  }
  startRecordPath(std::chrono::seconds(duration_seconds), ec);
}
void BaseCobot::stopRecordPath(error_code &ec) noexcept { handle_->backend->stopRecordPath(ec); }
void BaseCobot::cancelRecordPath(error_code &ec) noexcept { handle_->backend->cancelRecordPath(ec); }
void BaseCobot::saveRecordPath(const std::string &name, error_code &ec, const std::string &saveAs) noexcept { handle_->backend->saveRecordPath(name, ec, saveAs); }
void BaseCobot::replayPath(const std::string &name, double rate, error_code &ec) noexcept { handle_->backend->replayPath(name, rate, ec); }
void BaseCobot::removePath(const std::string &name, error_code &ec, bool removeAll) noexcept { handle_->backend->removePath(name, ec, removeAll); }
std::vector<std::string> BaseCobot::queryPathLists(error_code &ec) noexcept { return handle_->backend->queryPathLists(ec); }

xMateModel<6> BaseCobot::model() const { return xMateModel<6>(handle_); }

void BaseCobot::setRtNetworkTolerance(unsigned percent, error_code &ec) noexcept {
  if (percent > 100u) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return;
  }
  handle_->backend->sendCustomData(rokae_xmate3_ros2::runtime::rt_topics::kConfigRtNetworkTolerance,
                                   std::to_string(percent), ec);
}

void BaseCobot::useRciClient(bool use, error_code &ec) noexcept {
  handle_->backend->sendCustomData(rokae_xmate3_ros2::runtime::rt_topics::kConfigUseRciClient,
                                   serialize_bool(use), ec);
}

}  // namespace rokae
