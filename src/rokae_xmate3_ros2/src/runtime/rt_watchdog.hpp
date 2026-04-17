#ifndef ROKAE_XMATE3_ROS2_RUNTIME_RT_WATCHDOG_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_RT_WATCHDOG_HPP

#include <cstdint>
#include <string>

namespace rokae_xmate3_ros2::runtime {

struct RtWatchdogSnapshot {
  std::uint32_t late_cycle_count = 0;
  double max_gap_ms = 0.0;
  double avg_gap_ms = 0.0;
  std::uint32_t consecutive_late_cycles = 0;
  std::uint32_t stale_state_count = 0;
  std::uint32_t command_starvation_windows = 0;
  bool stale_state = false;
  bool command_starvation = false;
  std::string last_trigger_reason{"nominal"};
  std::string summary{"nominal"};
};

class RtWatchdog {
 public:
  explicit RtWatchdog(double late_cycle_threshold_sec = 0.004,
                      double stale_state_threshold_sec = 0.050);

  void observeCycle(double dt_sec, bool state_updated, bool command_sent);
  void reset();
  [[nodiscard]] RtWatchdogSnapshot snapshot() const;

 private:
  double late_cycle_threshold_sec_;
  double stale_state_threshold_sec_;
  std::uint64_t observed_cycles_ = 0;
  double accumulated_gap_ms_ = 0.0;
  RtWatchdogSnapshot snapshot_;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
