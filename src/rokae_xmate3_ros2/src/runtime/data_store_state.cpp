#include "runtime/runtime_state.hpp"

#include <algorithm>
#include <cctype>
#include <optional>
#include <sstream>

#include "rokae_xmate3_ros2/runtime/rt_semantic_topics.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr std::size_t kMaxLogBuffer = 128;
constexpr double kDefaultRtCommandTimeoutSec = 0.25;
constexpr double kMinRtCommandTimeoutSec = 0.05;
constexpr double kMaxRtCommandTimeoutSec = 2.0;

std::vector<std::string> splitText(const std::string &text, char delim) {
  std::vector<std::string> parts;
  std::string current;
  for (char ch : text) {
    if (ch == delim) {
      parts.push_back(current);
      current.clear();
      continue;
    }
    current.push_back(ch);
  }
  parts.push_back(current);
  return parts;
}

std::string trimCopy(std::string value) {
  const auto not_space = [](unsigned char c) { return !std::isspace(c); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::optional<std::string> findField(const std::string &payload, const std::string &key) {
  for (const auto &part : splitText(payload, ';')) {
    const auto pos = part.find('=');
    if (pos == std::string::npos) {
      continue;
    }
    if (trimCopy(part.substr(0, pos)) == key) {
      return trimCopy(part.substr(pos + 1));
    }
  }
  return std::nullopt;
}

bool parseArrayCsv(const std::string &csv, std::array<double, 6> &out) {
  const auto parts = splitText(csv, ',');
  if (parts.size() != out.size()) {
    return false;
  }
  for (std::size_t i = 0; i < out.size(); ++i) {
    try {
      out[i] = std::stod(trimCopy(parts[i]));
    } catch (...) {
      return false;
    }
  }
  return true;
}

bool parseArrayCsv(const std::string &csv, std::array<double, 3> &out) {
  const auto parts = splitText(csv, ',');
  if (parts.size() != out.size()) {
    return false;
  }
  for (std::size_t i = 0; i < out.size(); ++i) {
    try {
      out[i] = std::stod(trimCopy(parts[i]));
    } catch (...) {
      return false;
    }
  }
  return true;
}

bool parseArrayCsv(const std::string &csv, std::array<double, 16> &out) {
  const auto parts = splitText(csv, ',');
  if (parts.size() != out.size()) {
    return false;
  }
  for (std::size_t i = 0; i < out.size(); ++i) {
    try {
      out[i] = std::stod(trimCopy(parts[i]));
    } catch (...) {
      return false;
    }
  }
  return true;
}

bool parseBool(const std::string &raw, bool fallback = false) {
  const auto lowered = trimCopy(raw);
  if (lowered == "1" || lowered == "true" || lowered == "on") {
    return true;
  }
  if (lowered == "0" || lowered == "false" || lowered == "off") {
    return false;
  }
  return fallback;
}

void updateDirectCommandSnapshot(const DataStoreState::CustomDataEntry &entry,
                                 DataStoreState::RtDirectCommandSnapshot &command) {
  command = {};
  command.present = entry.valid;
  command.updated_at = entry.updated_at;
  if (!entry.valid || entry.value.empty()) {
    return;
  }

  if (const auto raw_values = findField(entry.value, "values");
      raw_values.has_value() && parseArrayCsv(*raw_values, command.values)) {
    command.valid = true;
  }
  if (const auto raw_finished = findField(entry.value, "finished"); raw_finished.has_value()) {
    command.finished = parseBool(*raw_finished, false);
  }
  if (const auto raw_seq = findField(entry.value, "seq"); raw_seq.has_value()) {
    try {
      command.sequence = static_cast<std::uint64_t>(std::stoull(*raw_seq));
    } catch (...) {
      command.sequence = 0;
    }
  }
}

void updateArray6Topic(const std::string &value,
                       bool &configured,
                       std::array<double, 6> &target,
                       const std::optional<std::string> &field_name = std::nullopt) {
  configured = false;
  std::array<double, 6> parsed{};
  if (parseArrayCsv(value, parsed)) {
    target = parsed;
    configured = true;
    return;
  }
  if (field_name.has_value()) {
    const auto raw = findField(value, *field_name);
    if (raw.has_value() && parseArrayCsv(*raw, parsed)) {
      target = parsed;
      configured = true;
    }
  }
}

void updateEndEffectorFrame(const std::string &value, DataStoreState::RtControlSnapshot &snapshot) {
  std::array<double, 16> parsed{};
  if (parseArrayCsv(value, parsed)) {
    snapshot.end_effector_frame = parsed;
    snapshot.end_effector_frame_configured = true;
    return;
  }
  if (const auto raw_values = findField(value, "values"); raw_values.has_value() && parseArrayCsv(*raw_values, parsed)) {
    snapshot.end_effector_frame = parsed;
    snapshot.end_effector_frame_configured = true;
    return;
  }
  snapshot.end_effector_frame_configured = false;
}

void updateCartesianLimit(const std::string &value, DataStoreState::RtCartesianLimitSnapshot &limit) {
  limit.configured = false;
  limit.enabled = false;
  if (value.empty()) {
    return;
  }
  const auto lengths_text = findField(value, "lengths");
  const auto frame_text = findField(value, "frame");
  if (lengths_text.has_value() && parseArrayCsv(*lengths_text, limit.lengths)) {
    limit.enabled = std::any_of(limit.lengths.begin(), limit.lengths.end(), [](double axis) { return axis > 1e-9; });
    limit.configured = true;
  } else {
    const auto prefix = value.find(';');
    const auto legacy_lengths = value.substr(0, prefix == std::string::npos ? std::string::npos : prefix);
    if (parseArrayCsv(legacy_lengths, limit.lengths)) {
      limit.enabled = std::any_of(limit.lengths.begin(), limit.lengths.end(), [](double axis) { return axis > 1e-9; });
      limit.configured = true;
    }
  }
  if (frame_text.has_value()) {
    std::array<double, 16> parsed{};
    if (parseArrayCsv(*frame_text, parsed)) {
      limit.frame = parsed;
      limit.configured = true;
    }
  }
}

void updateForceControlFrame(const std::string &value,
                             DataStoreState::RtForceControlFrameSnapshot &config) {
  config = {};
  if (value.empty()) {
    return;
  }
  if (const auto raw_type = findField(value, "type"); raw_type.has_value()) {
    try {
      config.type = static_cast<rokae::FrameType>(std::stoi(*raw_type));
    } catch (...) {
      config.type = rokae::FrameType::world;
    }
  }
  if (const auto raw_values = findField(value, "values"); raw_values.has_value()) {
    std::array<double, 16> parsed{};
    if (parseArrayCsv(*raw_values, parsed)) {
      config.frame = parsed;
      config.configured = true;
      return;
    }
  }
  const auto prefix = value.find(';');
  const auto legacy_frame = value.substr(0, prefix == std::string::npos ? std::string::npos : prefix);
  std::array<double, 16> parsed{};
  if (parseArrayCsv(legacy_frame, parsed)) {
    config.frame = parsed;
    config.configured = true;
  }
}

void updateFilterLimit(const std::string &value, DataStoreState::RtControlSnapshot &snapshot) {
  snapshot.filter_limit_configured = false;
  snapshot.filter_limit_enabled = true;
  snapshot.filter_limit_cutoff_frequency = 80.0;
  if (value.empty()) {
    return;
  }
  if (const auto raw_enabled = findField(value, "limit_rate"); raw_enabled.has_value()) {
    snapshot.filter_limit_enabled = parseBool(*raw_enabled, true);
    snapshot.filter_limit_configured = true;
  }
  if (const auto raw_enabled = findField(value, "enabled"); raw_enabled.has_value()) {
    snapshot.filter_limit_enabled = parseBool(*raw_enabled, snapshot.filter_limit_enabled);
    snapshot.filter_limit_configured = true;
  }
  if (const auto raw_cutoff = findField(value, "cutoff_frequency"); raw_cutoff.has_value()) {
    try {
      snapshot.filter_limit_cutoff_frequency = std::stod(*raw_cutoff);
      snapshot.filter_limit_configured = true;
    } catch (...) {
      snapshot.filter_limit_cutoff_frequency = 80.0;
    }
  }
}

void updateLoadContext(const std::string &value, DataStoreState::RtLoadSnapshot &load) {
  load = {};
  if (value.empty()) {
    return;
  }
  if (const auto raw_mass = findField(value, "mass"); raw_mass.has_value()) {
    try {
      load.mass = std::stod(*raw_mass);
      load.configured = true;
    } catch (...) {
      load.mass = 0.0;
    }
  }
  if (const auto raw_cog = findField(value, "cog"); raw_cog.has_value()) {
    std::array<double, 3> parsed{};
    if (parseArrayCsv(*raw_cog, parsed)) {
      load.cog = parsed;
      load.configured = true;
    }
  }
  if (const auto raw_inertia = findField(value, "inertia"); raw_inertia.has_value()) {
    std::array<double, 3> parsed{};
    if (parseArrayCsv(*raw_inertia, parsed)) {
      load.inertia = parsed;
      load.configured = true;
    }
  }
}

void updateRtNetworkTolerance(const std::string &value, DataStoreState::RtControlSnapshot &snapshot) {
  snapshot.rt_network_tolerance_configured = false;
  snapshot.rt_command_timeout_sec = kDefaultRtCommandTimeoutSec;
  if (value.empty()) {
    return;
  }
  try {
    const double percent = std::clamp(std::stod(value), 0.0, 100.0);
    const double normalized = percent / 100.0;
    snapshot.rt_command_timeout_sec = std::clamp(
        kMinRtCommandTimeoutSec + normalized * (kMaxRtCommandTimeoutSec - kMinRtCommandTimeoutSec),
        kMinRtCommandTimeoutSec,
        kMaxRtCommandTimeoutSec);
    snapshot.rt_network_tolerance_configured = true;
  } catch (...) {
    snapshot.rt_command_timeout_sec = kDefaultRtCommandTimeoutSec;
  }
}

void updateScalarTopic(const std::string &value, bool &configured, double &target) {
  configured = false;
  try {
    target = std::stod(value);
    configured = true;
    return;
  } catch (...) {
  }
  if (const auto raw = findField(value, "value"); raw.has_value()) {
    try {
      target = std::stod(*raw);
      configured = true;
    } catch (...) {
      configured = false;
    }
  }
}

void updateSemanticSnapshotForTopic(const std::string &topic,
                                  const DataStoreState::CustomDataEntry &entry,
                                  DataStoreState::RtSemanticSnapshot &snapshot) {
  if (topic == rt_topics::kControlSurface) {
    snapshot.control_surface = !entry.value.empty() ? entry.value : std::string{"sdk_shim"};
    return;
  }
  if (topic == rt_topics::kControlDispatchMode) {
    snapshot.dispatch_mode = !entry.value.empty() ? entry.value : std::string{"idle"};
    return;
  }
  if (topic == rt_topics::kCatalogProvenance) {
    snapshot.catalog_provenance = !entry.value.empty() ? entry.value : std::string{"runtime_authoritative"};
    return;
  }
}

void updateRtSnapshotForTopic(const std::string &topic,
                              const DataStoreState::CustomDataEntry &entry,
                              DataStoreState::RtControlSnapshot &snapshot) {
  if (topic == rt_topics::kConfigUseRciClient) {
    snapshot.use_rci_client = parseBool(entry.value, false);
    snapshot.use_rci_client_configured = entry.valid;
    return;
  }
  if (topic == rt_topics::kControlStop) {
    snapshot.stop_requested = parseBool(entry.value, false);
    return;
  }
  if (topic == rt_topics::kConfigRtNetworkTolerance) {
    updateRtNetworkTolerance(entry.value, snapshot);
    return;
  }
  if (topic == rt_topics::kConfigJointImpedance) {
    updateArray6Topic(entry.value, snapshot.joint_impedance_configured, snapshot.joint_impedance, std::string{"values"});
    return;
  }
  if (topic == rt_topics::kConfigFilterFrequency) {
    updateArray6Topic(entry.value, snapshot.filter_frequency_configured, snapshot.filter_frequency, std::string{"values"});
    return;
  }
  if (topic == rt_topics::kConfigFilterLimit) {
    updateFilterLimit(entry.value, snapshot);
    return;
  }
  if (topic == rt_topics::kConfigCartesianImpedance) {
    updateArray6Topic(entry.value, snapshot.cartesian_impedance_configured, snapshot.cartesian_impedance, std::string{"values"});
    return;
  }
  if (topic == rt_topics::kConfigCartesianDesiredWrench) {
    updateArray6Topic(entry.value, snapshot.cartesian_desired_wrench_configured, snapshot.cartesian_desired_wrench);
    return;
  }
  if (topic == rt_topics::kConfigCollisionBehaviourThresholds) {
    updateArray6Topic(entry.value, snapshot.collision_thresholds_configured, snapshot.collision_thresholds, std::string{"values"});
    return;
  }
  if (topic == rt_topics::kConfigTorqueCutoffFrequency) {
    updateScalarTopic(entry.value, snapshot.torque_cutoff_frequency_configured, snapshot.torque_cutoff_frequency);
    return;
  }
  if (topic == rt_topics::kConfigEndEffectorFrame) {
    updateEndEffectorFrame(entry.value, snapshot);
    return;
  }
  if (topic == rt_topics::kConfigCartesianLimit) {
    updateCartesianLimit(entry.value, snapshot.cartesian_limit);
    return;
  }
  if (topic == rt_topics::kConfigForceControlFrame) {
    updateForceControlFrame(entry.value, snapshot.force_control_frame);
    return;
  }
  if (topic == rt_topics::kConfigLoad) {
    updateLoadContext(entry.value, snapshot.load);
    return;
  }
  if (topic == rt_topics::kControlJointPosition) {
    updateDirectCommandSnapshot(entry, snapshot.joint_position_command);
    return;
  }
  if (topic == rt_topics::kControlCartesianPosition) {
    updateDirectCommandSnapshot(entry, snapshot.cartesian_position_command);
    return;
  }
  if (topic == rt_topics::kControlTorque) {
    updateDirectCommandSnapshot(entry, snapshot.torque_command);
    return;
  }
}

}  // namespace

void DataStoreState::appendLog(const rokae_xmate3_ros2::msg::LogInfo &log) {
  std::lock_guard<std::mutex> lock(mutex_);
  log_buffer_.push_back(log);
  if (log_buffer_.size() > kMaxLogBuffer) {
    log_buffer_.erase(log_buffer_.begin(), log_buffer_.begin() + (log_buffer_.size() - kMaxLogBuffer));
  }
}

std::vector<rokae_xmate3_ros2::msg::LogInfo> DataStoreState::queryLogs(
    unsigned int count,
    std::optional<std::uint8_t> level) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rokae_xmate3_ros2::msg::LogInfo> result;
  const std::size_t bounded_count = std::min<std::size_t>(std::max<unsigned int>(count, 0u), log_buffer_.size());
  result.reserve(bounded_count);
  for (auto it = log_buffer_.rbegin(); it != log_buffer_.rend() && result.size() < bounded_count; ++it) {
    if (level.has_value() && *level != 0u && it->level != *level) {
      continue;
    }
    result.push_back(*it);
  }
  return result;
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

std::vector<std::string> DataStoreState::registerKeys() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> keys;
  keys.reserve(register_bank_.size());
  for (const auto &entry : register_bank_) {
    keys.push_back(entry.first);
  }
  return keys;
}

void DataStoreState::setCustomData(const std::string &topic, const std::string &value) {
  std::lock_guard<std::mutex> lock(mutex_);
  CustomDataEntry entry;
  entry.value = value;
  entry.updated_at = std::chrono::steady_clock::now();
  entry.valid = true;
  custom_data_bank_[topic] = entry;
  if (topic.rfind("register/", 0) == 0) {
    register_bank_[topic.substr(9)] = value;
  }
  updateRtSnapshotForTopic(topic, entry, rt_control_snapshot_);
  updateSemanticSnapshotForTopic(topic, entry, rt_semantic_snapshot_);
}

std::string DataStoreState::customData(const std::string &topic) const {
  return customDataEntry(topic).value;
}

DataStoreState::CustomDataEntry DataStoreState::customDataEntry(const std::string &topic) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = custom_data_bank_.find(topic);
  return it == custom_data_bank_.end() ? CustomDataEntry{} : it->second;
}

DataStoreState::RtControlSnapshot DataStoreState::rtControlSnapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rt_control_snapshot_;
}

DataStoreState::RtSemanticSnapshot DataStoreState::rtSemanticSnapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rt_semantic_snapshot_;
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

}  // namespace rokae_xmate3_ros2::runtime
