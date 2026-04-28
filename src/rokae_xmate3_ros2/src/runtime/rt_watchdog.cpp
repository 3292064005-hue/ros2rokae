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

  const double gap_ms = dt_sec * 1000.0;
  ++observed_cycles_;
  accumulated_gap_ms_ += gap_ms;
  snapshot_.avg_gap_ms = observed_cycles_ > 0 ? (accumulated_gap_ms_ / static_cast<double>(observed_cycles_)) : 0.0;
  snapshot_.max_gap_ms = std::max(snapshot_.max_gap_ms, gap_ms);

  if (dt_sec > late_cycle_threshold_sec_) {
    ++snapshot_.late_cycle_count;
    ++snapshot_.consecutive_late_cycles;
    snapshot_.last_trigger_reason = "late_cycle";
  } else {
    snapshot_.consecutive_late_cycles = 0;
  }

  snapshot_.stale_state = !state_updated || dt_sec > stale_state_threshold_sec_;
  if (snapshot_.stale_state) {
    ++snapshot_.stale_state_count;
    snapshot_.last_trigger_reason = !state_updated ? "state_not_updated" : "stale_state";
  }

  snapshot_.command_starvation = !command_sent;
  if (snapshot_.command_starvation) {
    ++snapshot_.command_starvation_windows;
    snapshot_.last_trigger_reason = "command_starvation";
  }

  if (!snapshot_.stale_state && !snapshot_.command_starvation && dt_sec <= late_cycle_threshold_sec_) {
    snapshot_.last_trigger_reason = "nominal";
  }

  std::ostringstream stream;
  stream << "late_cycles=" << snapshot_.late_cycle_count << ",max_gap_ms=" << snapshot_.max_gap_ms
         << ",avg_gap_ms=" << snapshot_.avg_gap_ms
         << ",consecutive_late_cycles=" << snapshot_.consecutive_late_cycles
         << ",stale_state_count=" << snapshot_.stale_state_count
         << ",command_starvation_windows=" << snapshot_.command_starvation_windows
         << ",last_trigger_reason=" << snapshot_.last_trigger_reason;
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
  observed_cycles_ = 0;
  accumulated_gap_ms_ = 0.0;
}

RtWatchdogSnapshot RtWatchdog::snapshot() const {
  return snapshot_;
}

}  // namespace rokae_xmate3_ros2::runtime
