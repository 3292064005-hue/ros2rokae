#include "runtime/runtime_state.hpp"

#include <algorithm>

namespace rokae_xmate3_ros2::runtime {

void DataStoreState::appendLog(const rokae_xmate3_ros2::msg::LogInfo &log) {
  std::lock_guard<std::mutex> lock(mutex_);
  log_buffer_.push_back(log);
  constexpr std::size_t kMaxLogBuffer = 128;
  if (log_buffer_.size() > kMaxLogBuffer) {
    log_buffer_.erase(log_buffer_.begin(), log_buffer_.begin() + (log_buffer_.size() - kMaxLogBuffer));
  }
}

std::vector<rokae_xmate3_ros2::msg::LogInfo> DataStoreState::queryLogs(unsigned int count) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto bounded = std::min<std::size_t>(count, log_buffer_.size());
  return std::vector<rokae_xmate3_ros2::msg::LogInfo>(log_buffer_.begin(), log_buffer_.begin() + bounded);
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
  custom_data_bank_[topic] = value;
  if (topic.rfind("register/", 0) == 0) {
    register_bank_[topic.substr(9)] = value;
  }
}

std::string DataStoreState::customData(const std::string &topic) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = custom_data_bank_.find(topic);
  return it == custom_data_bank_.end() ? std::string() : it->second;
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
