#ifndef ROKAE_XMATE3_ROS2_RUNTIME_RT_SCHEDULER_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_RT_SCHEDULER_HPP

#include <string>

#include <rclcpp/logger.hpp>

namespace rokae_xmate3_ros2::runtime {

struct RtSchedulerRequest {
  bool enable = true;
  std::string policy{"fifo"};
  int priority = 80;
  std::string cpu_affinity;
  bool lock_all_memory = true;
  bool require_strict_success = false;
};

struct RtSchedulerResult {
  std::string state{"disabled"};
  bool enabled = false;
  bool active = false;
  bool degraded = false;
  bool hard_failure = false;
};

/**
 * @brief Apply realtime scheduler settings to the current thread.
 *
 * This helper is intentionally thread-scoped: the caller decides which thread is the
 * authoritative servo owner, and this function binds the scheduler contract to that thread.
 *
 * @param logger Logger used for operator-visible diagnostics.
 * @param request Requested scheduler policy, priority, affinity and memory-lock contract.
 * @return Effective scheduler result including degraded/fail-fast state.
 */
[[nodiscard]] RtSchedulerResult applyRtScheduler(const rclcpp::Logger &logger,
                                                 const RtSchedulerRequest &request);

}  // namespace rokae_xmate3_ros2::runtime

#endif
