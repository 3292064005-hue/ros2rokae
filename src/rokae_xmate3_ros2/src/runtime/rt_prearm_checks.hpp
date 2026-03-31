#ifndef ROKAE_XMATE3_ROS2_RUNTIME_RT_PREARM_CHECKS_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_RT_PREARM_CHECKS_HPP

#include <string>
#include <vector>

#include "runtime/rt_subscription_plan.hpp"

namespace rokae_xmate3_ros2::runtime {

struct RtPrearmCheckInput {
  int motion_mode = 0;
  int rt_mode = -1;
  bool power_on = false;
  bool network_tolerance_configured = true;
  std::string active_profile{"unknown"};
  RtSubscriptionPlan subscription_plan;
};

struct RtPrearmCheckReport {
  bool ok = false;
  std::string status{"not_ready"};
  std::vector<std::string> notes;

  [[nodiscard]] std::string summary() const;
};

[[nodiscard]] RtPrearmCheckReport evaluateRtPrearm(const RtPrearmCheckInput &input);

}  // namespace rokae_xmate3_ros2::runtime

#endif
