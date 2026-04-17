#include "runtime/planning_capability_service.hpp"

#include <algorithm>
#include <sstream>

namespace rokae_xmate3_ros2::runtime {
namespace {

PlanningCapabilityDescriptor make_descriptor(std::string name,
                                             std::string summary,
                                             bool active = false,
                                             bool experimental = false) {
  PlanningCapabilityDescriptor descriptor;
  descriptor.name = std::move(name);
  descriptor.summary = std::move(summary);
  descriptor.active = active;
  descriptor.experimental = experimental;
  return descriptor;
}

template <typename Container>
void sort_active_first(Container &entries) {
  std::sort(entries.begin(), entries.end(), [](const auto &lhs, const auto &rhs) {
    if (lhs.active != rhs.active) {
      return lhs.active > rhs.active;
    }
    return lhs.name < rhs.name;
  });
}

std::string summarize_group(const std::string &group_name,
                            const std::vector<PlanningCapabilityDescriptor> &entries) {
  std::ostringstream stream;
  stream << group_name << '[';
  bool first = true;
  for (const auto &entry : entries) {
    if (!first) {
      stream << ',';
    }
    first = false;
    stream << entry.name;
    if (entry.active) {
      stream << "=active";
    }
    if (entry.experimental) {
      stream << ":experimental";
    }
  }
  stream << ']';
  return stream.str();
}

}  // namespace

std::vector<PlanningCapabilityDescriptor> buildKinematicsBackendCatalog(const std::string &active_backend) {
  std::vector<PlanningCapabilityDescriptor> entries;
  entries.reserve(4);
  entries.push_back(make_descriptor("kdl",
                                    "Primary production backend for FK/IK/Jacobian requests",
                                    active_backend == "kdl"));
  entries.push_back(make_descriptor("improved_dh",
                                    "Numerical fallback backend used for seed repair and regression checks",
                                    active_backend == "improved_dh"));
  entries.push_back(make_descriptor("official_dh_reference",
                                    "Reference-only backend for specification tracing and diagnostics",
                                    active_backend == "official_dh_reference",
                                    true));
  entries.push_back(make_descriptor("hybrid_seed",
                                    "Experimental hybrid seed backend for conservative branch selection",
                                    active_backend == "hybrid_seed",
                                    true));
  sort_active_first(entries);
  return entries;
}

std::vector<PlanningCapabilityDescriptor> buildRetimerPolicyCatalog(const std::string &active_policy) {
  std::vector<PlanningCapabilityDescriptor> entries;
  entries.reserve(4);
  entries.push_back(make_descriptor("nominal",
                                    "Balanced default retimer for NRT paths and conservative RT previews",
                                    active_policy == "nominal"));
  entries.push_back(make_descriptor("conservative",
                                    "Reduced velocity/acceleration envelope with stricter stop-point preference",
                                    active_policy == "conservative"));
  entries.push_back(make_descriptor("online_safe",
                                    "RT-oriented policy with safer dt assumptions and stronger clamp behavior",
                                    active_policy == "online_safe"));
  entries.push_back(make_descriptor("stop_point_enforced",
                                    "Experimental policy that minimizes blend carry-through and forces stop-points",
                                    active_policy == "stop_point_enforced",
                                    true));
  sort_active_first(entries);
  return entries;
}

std::vector<PlanningCapabilityDescriptor> buildPlannerSelectionCatalog(const std::string &active_policy) {
  std::vector<PlanningCapabilityDescriptor> entries;
  entries.reserve(4);
  entries.push_back(make_descriptor("risk_weighted",
                                    "Primary selection policy: continuity/singularity/soft-limit weighted score",
                                    active_policy == "risk_weighted"));
  entries.push_back(make_descriptor("nominal_preferred",
                                    "Prefers blended nominal plans when risk margins remain acceptable",
                                    active_policy == "nominal_preferred"));
  entries.push_back(make_descriptor("conservative_stop",
                                    "Prefers stop-point candidates for branch or singularity risk",
                                    active_policy == "conservative_stop"));
  entries.push_back(make_descriptor("cartesian_hold",
                                    "Experimental cartesian continuity bias for mixed-family lookahead runs",
                                    active_policy == "cartesian_hold",
                                    true));
  sort_active_first(entries);
  return entries;
}

std::string summarizePlanningCapabilityCatalog(
    const std::vector<PlanningCapabilityDescriptor> &kinematics_backends,
    const std::vector<PlanningCapabilityDescriptor> &retimer_policies,
    const std::vector<PlanningCapabilityDescriptor> &planner_policies) {
  return summarize_group("kinematics", kinematics_backends) + "; " +
         summarize_group("retimer", retimer_policies) + "; " +
         summarize_group("planner", planner_policies);
}

}  // namespace rokae_xmate3_ros2::runtime
