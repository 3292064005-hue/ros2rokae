#ifndef ROKAE_XMATE3_ROS2_RUNTIME_PLANNING_CAPABILITY_SERVICE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_PLANNING_CAPABILITY_SERVICE_HPP

#include <string>
#include <vector>

namespace rokae_xmate3_ros2::runtime {

struct PlanningCapabilityDescriptor {
  std::string name;
  std::string summary;
  bool active = false;
  bool experimental = false;
  std::string source{"planning_capability_matrix"};
};

[[nodiscard]] std::vector<PlanningCapabilityDescriptor> buildKinematicsBackendCatalog(
    const std::string &active_backend = "kdl");
[[nodiscard]] std::vector<PlanningCapabilityDescriptor> buildRetimerPolicyCatalog(
    const std::string &active_policy = "nominal");
[[nodiscard]] std::vector<PlanningCapabilityDescriptor> buildPlannerSelectionCatalog(
    const std::string &active_policy = "risk_weighted");

[[nodiscard]] std::string summarizePlanningCapabilityCatalog(
    const std::vector<PlanningCapabilityDescriptor> &kinematics_backends,
    const std::vector<PlanningCapabilityDescriptor> &retimer_policies,
    const std::vector<PlanningCapabilityDescriptor> &planner_policies);

}  // namespace rokae_xmate3_ros2::runtime

#endif
