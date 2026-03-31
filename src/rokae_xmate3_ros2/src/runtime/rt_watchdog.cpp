#include "runtime/rt_watchdog.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace rokae_xmate3_ros2::runtime {

RtWatchdog::RtWatchdog(double late_cycle_threshold_sec, double stale_state_threshold_sec)
    : late_cycle_threshold_sec_(std::max(late_cycle_threshold_sec, 1e-6)),
      stale_state_threshold_sec_(std::max(stale_state_threshold_sec, 1e-6)) {}

void RtWatchdog::observeCycle(double dt_sec, bool state_updated, bool command_sent) {
  if (!std::isfinite(dt_sec) || dt_sec < 0.0) {
    return;
  }
  snapshot_.max_gap_ms = std::max(snapshot_.max_gap_ms, dt_sec * 1000.0);
  if (dt_sec > late_cycle_threshold_sec_) {
    ++snapshot_.late_cycle_count;
  }
  snapshot_.stale_state = !state_updated || dt_sec > stale_state_threshold_sec_;
  snapshot_.command_starvation = !command_sent;

  std::ostringstream stream;
  stream << "late_cycles=" << snapshot_.late_cycle_count
         << ",max_gap_ms=" << snapshot_.max_gap_ms;
  if (snapshot_.stale_state) {
    stream << ",stale_state=true";
  }
  if (snapshot_.command_starvation) {
    stream << ",command_starvation=true";
  }
  snapshot_.summary = stream.str();
}

void RtWatchdog::reset() {
  snapshot_ = RtWatchdogSnapshot{};
}

RtWatchdogSnapshot RtWatchdog::snapshot() const {
  return snapshot_;
}

}  // namespace rokae_xmate3_ros2::runtime
