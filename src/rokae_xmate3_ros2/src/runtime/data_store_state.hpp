#ifndef ROKAE_XMATE3_ROS2_RUNTIME_DATA_STORE_STATE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_DATA_STORE_STATE_HPP

#include <array>
#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rokae/data_types.h"
#include "rokae_xmate3_ros2/msg/log_info.hpp"

namespace rokae_xmate3_ros2::runtime {

class DataStoreState {
 public:
  using IoKey = std::pair<unsigned int, unsigned int>;

  struct CustomDataEntry {
    std::string value;
    std::chrono::steady_clock::time_point updated_at{};
    bool valid = false;
  };

  struct RtDirectCommandSnapshot {
    bool present = false;
    bool valid = false;
    bool finished = false;
    std::uint64_t sequence = 0;
    std::array<double, 6> values{};
    std::chrono::steady_clock::time_point updated_at{};
  };

  struct RtCartesianLimitSnapshot {
    bool configured = false;
    bool enabled = false;
    std::array<double, 3> lengths{};
    std::array<double, 16> frame{1.0, 0.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0, 0.0,
                                 0.0, 0.0, 1.0, 0.0,
                                 0.0, 0.0, 0.0, 1.0};
  };

  struct RtForceControlFrameSnapshot {
    bool configured = false;
    rokae::FrameType type = rokae::FrameType::world;
    std::array<double, 16> frame{1.0, 0.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0, 0.0,
                                 0.0, 0.0, 1.0, 0.0,
                                 0.0, 0.0, 0.0, 1.0};
  };

  struct RtLoadSnapshot {
    bool configured = false;
    double mass = 0.0;
    std::array<double, 3> cog{{0.0, 0.0, 0.0}};
    std::array<double, 3> inertia{{0.0, 0.0, 0.0}};
  };

  struct RtSemanticSnapshot {
    std::string control_surface{"sdk_shim"};
    std::string dispatch_mode{"idle"};
    std::string catalog_provenance{"runtime_authoritative"};
  };

  struct RtControlSnapshot {
    bool use_rci_client = false;
    bool use_rci_client_configured = false;
    bool stop_requested = false;
    bool rt_network_tolerance_configured = false;
    double rt_command_timeout_sec = 0.25;
    std::array<double, 6> joint_impedance{};
    bool joint_impedance_configured = false;
    std::array<double, 6> filter_frequency{};
    bool filter_frequency_configured = false;
    bool filter_limit_enabled = true;
    bool filter_limit_configured = false;
    double filter_limit_cutoff_frequency = 80.0;
    std::array<double, 6> cartesian_impedance{};
    bool cartesian_impedance_configured = false;
    std::array<double, 6> cartesian_desired_wrench{};
    bool cartesian_desired_wrench_configured = false;
    std::array<double, 6> collision_thresholds{};
    bool collision_thresholds_configured = false;
    double torque_cutoff_frequency = 80.0;
    bool torque_cutoff_frequency_configured = false;
    std::array<double, 16> end_effector_frame{1.0, 0.0, 0.0, 0.0,
                                              0.0, 1.0, 0.0, 0.0,
                                              0.0, 0.0, 1.0, 0.0,
                                              0.0, 0.0, 0.0, 1.0};
    bool end_effector_frame_configured = false;
    RtCartesianLimitSnapshot cartesian_limit{};
    RtForceControlFrameSnapshot force_control_frame{};
    RtLoadSnapshot load{};
    RtDirectCommandSnapshot joint_position_command{};
    RtDirectCommandSnapshot cartesian_position_command{};
    RtDirectCommandSnapshot torque_command{};
  };

  void appendLog(const rokae_xmate3_ros2::msg::LogInfo &log);
  [[nodiscard]] std::vector<rokae_xmate3_ros2::msg::LogInfo> queryLogs(
      unsigned int count,
      std::optional<std::uint8_t> level = std::nullopt) const;

  void setRegister(const std::string &key, const std::string &value);
  [[nodiscard]] std::string registerValue(const std::string &key) const;
  [[nodiscard]] std::vector<std::string> registerKeys() const;

  /**
   * @brief Persist raw custom data and refresh the pre-parsed realtime control snapshot.
   *
   * @param topic Semantic topic name published by the SDK/runtime bridge.
   * @param value Raw payload string. Empty strings are preserved for compatibility.
   *
   * @note Parsing failures never throw. The raw payload remains available via customDataEntry(),
   *       while the typed realtime snapshot marks the corresponding field as not configured.
   */
  void setCustomData(const std::string &topic, const std::string &value);
  [[nodiscard]] std::string customData(const std::string &topic) const;
  [[nodiscard]] CustomDataEntry customDataEntry(const std::string &topic) const;

  /**
   * @brief Return the fully parsed realtime control snapshot used by RuntimeControlBridge.
   *
   * @return Copy of the latest typed snapshot. The returned structure is lock-free for callers.
   *
   * @note This snapshot is updated on setCustomData() so the servo loop never reparses strings.
   */
  [[nodiscard]] RtControlSnapshot rtControlSnapshot() const;
  [[nodiscard]] RtSemanticSnapshot rtSemanticSnapshot() const;

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
  std::map<std::string, CustomDataEntry> custom_data_bank_;
  std::map<std::string, std::string> callback_registry_;
  std::map<IoKey, bool> di_state_;
  std::map<IoKey, bool> do_state_;
  std::map<IoKey, double> ai_state_;
  std::map<IoKey, double> ao_state_;
  std::vector<rokae_xmate3_ros2::msg::LogInfo> log_buffer_;
  RtControlSnapshot rt_control_snapshot_{};
  RtSemanticSnapshot rt_semantic_snapshot_{};
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
