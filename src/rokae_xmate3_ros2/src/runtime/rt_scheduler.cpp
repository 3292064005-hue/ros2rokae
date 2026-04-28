#include "runtime/rt_scheduler.hpp"

#include <algorithm>
#include <string>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <rclcpp/rclcpp.hpp>

namespace rokae_xmate3_ros2::runtime {

RtSchedulerResult applyRtScheduler(const rclcpp::Logger &logger,
                                   const RtSchedulerRequest &request) {
  RtSchedulerResult result;
  result.enabled = request.enable;
  if (!request.enable) {
    result.state = "disabled";
    RCLCPP_INFO(logger, "rt scheduler disabled by configuration");
    return result;
  }

  result.state = "active";
  result.active = true;

  if (request.lock_all_memory && mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    result.state = "degraded_best_effort(memory_lock_failed)";
    result.degraded = true;
  }

  int sched_policy = SCHED_FIFO;
  if (request.policy == "rr") {
    sched_policy = SCHED_RR;
  } else if (request.policy != "fifo") {
    sched_policy = SCHED_OTHER;
  }
  sched_param sched{};
  sched.sched_priority = std::clamp(request.priority, 1, 95);
  if (sched_setscheduler(0, sched_policy, &sched) != 0) {
    result.state = "degraded_best_effort(scheduler_failed)";
    result.degraded = true;
  }

  if (!request.cpu_affinity.empty()) {
    cpu_set_t set;
    CPU_ZERO(&set);
    bool any = false;
    std::size_t start = 0;
    while (start <= request.cpu_affinity.size()) {
      const auto end = request.cpu_affinity.find(',', start);
      const std::string token = request.cpu_affinity.substr(
          start,
          end == std::string::npos ? std::string::npos : end - start);
      if (!token.empty()) {
        try {
          const int cpu = std::stoi(token);
          if (cpu >= 0 && cpu < CPU_SETSIZE) {
            CPU_SET(cpu, &set);
            any = true;
          }
        } catch (...) {
        }
      }
      if (end == std::string::npos) {
        break;
      }
      start = end + 1;
    }
    if (any && pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
      result.state = "degraded_best_effort(affinity_failed)";
      result.degraded = true;
    }
  }

  if (request.require_strict_success && result.degraded) {
    result.hard_failure = true;
  }

  RCLCPP_INFO(
      logger,
      "rt scheduler setup: enable=%s policy=%s priority=%d cpu_affinity=\"%s\" lock_all=%s state=%s hard_failure=%s",
      request.enable ? "true" : "false",
      request.policy.c_str(),
      request.priority,
      request.cpu_affinity.c_str(),
      request.lock_all_memory ? "true" : "false",
      result.state.c_str(),
      result.hard_failure ? "true" : "false");

  return result;
}

}  // namespace rokae_xmate3_ros2::runtime
