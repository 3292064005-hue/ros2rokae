#ifndef ROKAE_XMATE3_ROS2_RUNTIME_RT_WATCHDOG_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_RT_WATCHDOG_HPP

#include <cstdint>
#include <string>

namespace rokae_xmate3_ros2::runtime {

struct RtWatchdogSnapshot {
  std::uint32_t late_cycle_count = 0;
  double max_gap_ms = 0.0;
  bool stale_state = false;
  bool command_starvation = false;
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
  RtWatchdogSnapshot snapshot_;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
