#include "runtime/service_facade.hpp"

#include <algorithm>

#include "runtime/service_facade_utils.hpp"

namespace rokae_xmate3_ros2::runtime {

namespace {

constexpr std::uint32_t kIoBoardMax = 1;
constexpr std::uint32_t kIoDigitalPortMax = 15;
constexpr std::uint32_t kIoAnalogPortMax = 7;
constexpr double kAoValueMin = 0.0;
constexpr double kAoValueMax = 10.0;

bool validateIoAddress(std::uint32_t board,
                       std::uint32_t port,
                       std::uint32_t max_port,
                       std::string &message) {
  if (board > kIoBoardMax) {
    message = "IO board index is out of range";
    return false;
  }
  if (port > max_port) {
    message = "IO port index is out of range";
    return false;
  }
  return true;
}

}  // namespace

IoProgramFacade::IoProgramFacade(SessionState &session_state,
                               DataStoreState &data_store_state,
                               ProgramState &program_state,
                               ToolingState &tooling_state,
                               TimeProvider time_provider)
    : session_state_(session_state),
      data_store_state_(data_store_state),
      program_state_(program_state),
      tooling_state_(tooling_state),
      time_provider_(std::move(time_provider)) {}

void IoProgramFacade::handleSendCustomData(
    const rokae_xmate3_ros2::srv::SendCustomData::Request &req,
    rokae_xmate3_ros2::srv::SendCustomData::Response &res) const {
  if (req.data_topic.empty()) {
    res.success = false;
    res.error_code = 12000;
    res.error_msg = "data_topic must not be empty";
    return;
  }
  if (!session_state_.connected()) {
    res.success = false;
    res.error_code = 12006;
    res.error_msg = "robot not connected";
    return;
  }
  data_store_state_.setCustomData(req.data_topic, req.custom_data);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.response_data = std::string("ACK:") + req.data_topic;
  res.send_time = detail::ToBuiltinTime(time_provider_());
}

void IoProgramFacade::handleRegisterDataCallback(
    const rokae_xmate3_ros2::srv::RegisterDataCallback::Request &req,
    rokae_xmate3_ros2::srv::RegisterDataCallback::Response &res) const {
  if (req.data_topic.empty() || req.callback_id.empty()) {
    res.success = false;
    res.error_code = 12001;
    res.error_msg = "data_topic and callback_id must not be empty";
    return;
  }
  if (!session_state_.connected()) {
    res.success = false;
    res.error_code = 12006;
    res.error_msg = "robot not connected";
    return;
  }
  if (data_store_state_.hasCallback(req.callback_id)) {
    res.success = false;
    res.error_code = 12004;
    res.error_msg = "callback_id already exists";
    return;
  }
  data_store_state_.registerCallback(req.callback_id, req.data_topic);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.register_time = detail::ToBuiltinTime(time_provider_());
}

void IoProgramFacade::handleReadRegister(const rokae_xmate3_ros2::srv::ReadRegister::Request &req,
                                         rokae_xmate3_ros2::srv::ReadRegister::Response &res) const {
  if (req.key.empty()) {
    res.success = false;
    res.error_code = 12002;
    res.error_msg = "register key must not be empty";
    return;
  }
  if (!session_state_.connected()) {
    res.success = false;
    res.error_code = 12006;
    res.error_msg = "robot not connected";
    return;
  }
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.value = data_store_state_.registerValue(req.key);
}

void IoProgramFacade::handleWriteRegister(const rokae_xmate3_ros2::srv::WriteRegister::Request &req,
                                          rokae_xmate3_ros2::srv::WriteRegister::Response &res) const {
  if (req.key.empty()) {
    res.success = false;
    res.error_code = 12002;
    res.error_msg = "register key must not be empty";
    return;
  }
  if (!session_state_.connected()) {
    res.success = false;
    res.error_code = 12006;
    res.error_msg = "robot not connected";
    return;
  }
  data_store_state_.setRegister(req.key, req.value);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void IoProgramFacade::handleReadRegisterEx(const rokae_xmate3_ros2::srv::ReadRegisterEx::Request &req,
                                           rokae_xmate3_ros2::srv::ReadRegisterEx::Response &res) const {
  if (req.name.empty()) {
    res.success = false;
    res.error_code = 12003;
    res.error_msg = "register name must not be empty";
    return;
  }
  if (req.index < 0) {
    res.success = false;
    res.error_code = 12007;
    res.error_msg = "register index must be non-negative";
    return;
  }
  if (!session_state_.connected()) {
    res.success = false;
    res.error_code = 12006;
    res.error_msg = "robot not connected";
    return;
  }
  res.key = req.name + "[" + std::to_string(req.index) + "]";
  res.value = data_store_state_.registerValue(res.key);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void IoProgramFacade::handleWriteRegisterEx(const rokae_xmate3_ros2::srv::WriteRegisterEx::Request &req,
                                            rokae_xmate3_ros2::srv::WriteRegisterEx::Response &res) const {
  if (req.name.empty()) {
    res.success = false;
    res.error_code = 12003;
    res.error_msg = "register name must not be empty";
    return;
  }
  if (req.index < 0) {
    res.success = false;
    res.error_code = 12007;
    res.error_msg = "register index must be non-negative";
    return;
  }
  if (!session_state_.connected()) {
    res.success = false;
    res.error_code = 12006;
    res.error_msg = "robot not connected";
    return;
  }
  res.key = req.name + "[" + std::to_string(req.index) + "]";
  data_store_state_.setRegister(res.key, req.value);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void IoProgramFacade::handleSetXPanelVout(const rokae_xmate3_ros2::srv::SetXPanelVout::Request &req,
                                          rokae_xmate3_ros2::srv::SetXPanelVout::Response &res) const {
  if (req.mode < 0 || req.mode > 3u) {
    res.success = false;
    res.error_code = 12005;
    res.error_msg = "xpanel mode must be one of off/reserve/supply12v/supply24v";
    return;
  }
  if (!session_state_.connected()) {
    res.success = false;
    res.error_code = 12006;
    res.error_msg = "robot not connected";
    return;
  }
  data_store_state_.setRegister("xpanel_vout", std::to_string(req.mode));
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.applied_mode = req.mode;
}

void IoProgramFacade::handleLoadRlProject(const rokae_xmate3_ros2::srv::LoadRLProject::Request &req,
                                          rokae_xmate3_ros2::srv::LoadRLProject::Response &res) const {
  const auto effective_project_name = detail::basename_without_extension(req.project_path);
  program_state_.loadRlProject(req.project_path, effective_project_name);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.project_name = program_state_.loadedRlProjectName();
  res.load_time = 0.0;
}

void IoProgramFacade::handleStartRlProject(const rokae_xmate3_ros2::srv::StartRLProject::Request &req,
                                           rokae_xmate3_ros2::srv::StartRLProject::Response &res) const {
  if (!program_state_.rlProjectLoaded() || req.project_id != program_state_.loadedRlProjectName()) {
    res.success = false;
    res.error_code = 7001;
    res.error_msg = "RL project is not loaded";
    return;
  }
  if (program_state_.rlProjectRunning()) {
    res.success = false;
    res.error_code = 7002;
    res.error_msg = "RL project is already running";
    return;
  }
  program_state_.setRlProjectRunning(true, 1);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.current_episode = program_state_.rlCurrentEpisode();
  res.start_time = detail::ToBuiltinTime(time_provider_());
}

void IoProgramFacade::handleStopRlProject(const rokae_xmate3_ros2::srv::StopRLProject::Request &req,
                                          rokae_xmate3_ros2::srv::StopRLProject::Response &res) const {
  if (!program_state_.rlProjectRunning() || req.project_id != program_state_.loadedRlProjectName()) {
    res.success = false;
    res.error_code = 8001;
    res.error_msg = "RL project is not running";
    return;
  }
  program_state_.setRlProjectRunning(false, program_state_.rlCurrentEpisode());
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.finished_episode = std::max(program_state_.rlCurrentEpisode(), 1);
  res.stop_time = detail::ToBuiltinTime(time_provider_());
}

void IoProgramFacade::handlePauseRlProject(const rokae_xmate3_ros2::srv::PauseRLProject::Request &req,
                                           rokae_xmate3_ros2::srv::PauseRLProject::Response &res) const {
  if (!program_state_.rlProjectRunning() || req.project_id != program_state_.loadedRlProjectName()) {
    res.success = false;
    res.error_code = 8001;
    res.error_msg = "RL project is not running";
    return;
  }
  program_state_.setRlProjectRunning(false, program_state_.rlCurrentEpisode());
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.current_episode = program_state_.rlCurrentEpisode();
}

void IoProgramFacade::handleSetProjectRunningOpt(const rokae_xmate3_ros2::srv::SetProjectRunningOpt::Request &req,
                                                 rokae_xmate3_ros2::srv::SetProjectRunningOpt::Response &res) const {
  program_state_.setRlProjectRunningOptions(req.rate, req.loop);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.applied_rate = program_state_.rlRunRate();
  res.applied_loop = program_state_.rlLoopMode();
}

void IoProgramFacade::handleGetDI(const rokae_xmate3_ros2::srv::GetDI::Request &req,
                                  rokae_xmate3_ros2::srv::GetDI::Response &res) const {
  std::string validation_message;
  if (!validateIoAddress(req.board, req.port, kIoDigitalPortMax, validation_message)) {
    res.success = false;
    res.message = validation_message;
    return;
  }
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
  res.state = data_store_state_.getDI(req.board, req.port);
  res.success = true;
}

void IoProgramFacade::handleGetDO(const rokae_xmate3_ros2::srv::GetDO::Request &req,
                                  rokae_xmate3_ros2::srv::GetDO::Response &res) const {
  std::string validation_message;
  if (!validateIoAddress(req.board, req.port, kIoDigitalPortMax, validation_message)) {
    res.success = false;
    res.message = validation_message;
    return;
  }
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
  res.state = data_store_state_.getDO(req.board, req.port);
  res.success = true;
}

void IoProgramFacade::handleSetDI(const rokae_xmate3_ros2::srv::SetDI::Request &req,
                                  rokae_xmate3_ros2::srv::SetDI::Response &res) const {
  std::string validation_message;
  if (!validateIoAddress(req.board, req.port, kIoDigitalPortMax, validation_message)) {
    res.success = false;
    res.message = validation_message;
    return;
  }
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
  if (!session_state_.simulationMode()) {
    res.success = false;
    res.message = "DI writes require simulation mode";
    return;
  }
  data_store_state_.setDI(req.board, req.port, req.state);
  res.success = true;
}

void IoProgramFacade::handleSetDO(const rokae_xmate3_ros2::srv::SetDO::Request &req,
                                  rokae_xmate3_ros2::srv::SetDO::Response &res) const {
  std::string validation_message;
  if (!validateIoAddress(req.board, req.port, kIoDigitalPortMax, validation_message)) {
    res.success = false;
    res.message = validation_message;
    return;
  }
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
  data_store_state_.setDO(req.board, req.port, req.state);
  res.success = true;
}

void IoProgramFacade::handleGetAI(const rokae_xmate3_ros2::srv::GetAI::Request &req,
                                  rokae_xmate3_ros2::srv::GetAI::Response &res) const {
  std::string validation_message;
  if (!validateIoAddress(req.board, req.port, kIoAnalogPortMax, validation_message)) {
    res.success = false;
    res.message = validation_message;
    return;
  }
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
  res.value = data_store_state_.getAI(req.board, req.port);
  res.success = true;
}

void IoProgramFacade::handleSetAO(const rokae_xmate3_ros2::srv::SetAO::Request &req,
                                  rokae_xmate3_ros2::srv::SetAO::Response &res) const {
  std::string validation_message;
  if (!validateIoAddress(req.board, req.port, kIoAnalogPortMax, validation_message)) {
    res.success = false;
    res.message = validation_message;
    return;
  }
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
  if (req.value < kAoValueMin || req.value > kAoValueMax) {
    res.success = false;
    res.message = "AO value must be within [0.0, 10.0]";
    return;
  }
  data_store_state_.setAO(req.board, req.port, req.value);
  res.success = true;
}


}  // namespace rokae_xmate3_ros2::runtime
