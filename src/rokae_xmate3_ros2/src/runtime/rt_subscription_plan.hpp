#ifndef ROKAE_XMATE3_ROS2_RUNTIME_RT_SUBSCRIPTION_PLAN_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_RT_SUBSCRIPTION_PLAN_HPP

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

namespace rokae_xmate3_ros2::runtime {

struct RtSubscriptionPlan {
  bool ok = false;
  std::string status{"inactive"};
  std::vector<std::string> accepted_fields;
  std::vector<std::string> rejected_fields;
  std::vector<std::string> notes;
  std::size_t total_bytes = 0;
  bool use_state_data_in_loop = false;
  double interval_ms = 0.0;

  [[nodiscard]] std::string summary() const;
};

[[nodiscard]] RtSubscriptionPlan buildRtSubscriptionPlan(
    const std::vector<std::string> &requested_fields,
    std::chrono::steady_clock::duration interval,
    bool use_state_data_in_loop);

}  // namespace rokae_xmate3_ros2::runtime

#endif
