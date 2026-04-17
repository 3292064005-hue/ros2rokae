#ifndef ROKAE_XMATE3_ROS2_RUNTIME_RUNTIME_PROFILE_SERVICE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_RUNTIME_PROFILE_SERVICE_HPP

#include <string>
#include <vector>

namespace rokae_xmate3_ros2::runtime {

struct RuntimeProfileDescriptor {
  std::string name;
  std::string owner_rule;
  std::string required_controller;
  std::string preferred_contract;
  std::string diagnostics_expectation;
  std::vector<std::string> allowed_motion_families;
  bool rt_capable = false;
  bool sim_approx = true;
  bool experimental = false;
  bool active = false;
  std::string source{"runtime_profile_matrix"};
};

[[nodiscard]] std::vector<RuntimeProfileDescriptor> buildRuntimeProfileCatalog(
    const std::string &backend_mode,
    const std::string &active_profile,
    const std::vector<std::string> &capability_flags);

[[nodiscard]] std::string summarizeRuntimeProfileCatalog(
    const std::vector<RuntimeProfileDescriptor> &profiles);

}  // namespace rokae_xmate3_ros2::runtime

#endif
