#ifndef ROKAE_XMATE3_ROS2_RUNTIME_PLANNER_PREFLIGHT_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_PLANNER_PREFLIGHT_HPP

#include <string>
#include <vector>

#include "runtime/runtime_types.hpp"

namespace rokae_xmate3_ros2::runtime {

struct PlannerPreflightReport {
  bool ok = false;
  std::string request_id;
  std::string reject_reason;
  std::string detail;
  std::string request_profile{"simulation-grade"};
  std::string primary_backend{"kdl"};
  std::string auxiliary_backend{"improved_dh"};
  std::string retimer_family{"unified"};
  std::string branch_policy{"nearest_seed"};
  std::string dominant_motion_kind{"none"};
  bool strict_conf = false;
  bool avoid_singularity = true;
  bool soft_limit_enabled = false;
  bool fallback_permitted = true;
  std::size_t command_count = 0;
  std::vector<std::string> notes;
};

[[nodiscard]] PlannerPreflightReport runPlannerPreflight(const MotionRequest &request);

}  // namespace rokae_xmate3_ros2::runtime

#endif
