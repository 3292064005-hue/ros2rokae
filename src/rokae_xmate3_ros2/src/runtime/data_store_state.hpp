#ifndef ROKAE_XMATE3_ROS2_RUNTIME_DATA_STORE_STATE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_DATA_STORE_STATE_HPP

#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rokae_xmate3_ros2/msg/log_info.hpp"

namespace rokae_xmate3_ros2::runtime {

class DataStoreState {
 public:
  using IoKey = std::pair<unsigned int, unsigned int>;

  void appendLog(const rokae_xmate3_ros2::msg::LogInfo &log);
  [[nodiscard]] std::vector<rokae_xmate3_ros2::msg::LogInfo> queryLogs(unsigned int count) const;

  void setRegister(const std::string &key, const std::string &value);
  [[nodiscard]] std::string registerValue(const std::string &key) const;
  [[nodiscard]] std::vector<std::string> registerKeys() const;

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

}  // namespace rokae_xmate3_ros2::runtime

#endif
