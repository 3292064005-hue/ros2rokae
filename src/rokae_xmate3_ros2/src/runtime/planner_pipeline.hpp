#ifndef ROKAE_XMATE3_ROS2_RUNTIME_PLANNER_PIPELINE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_PLANNER_PIPELINE_HPP

#include <string>
#include <vector>

#include "runtime/planner_preflight.hpp"
#include "runtime/runtime_types.hpp"

namespace rokae_xmate3_ros2::runtime {

struct PlannerExecutionCandidate {
  std::string name{"nominal"};
  std::string requested_stop_point{"segment_start"};
  double score = 0.0;
  double continuity_component = 0.0;
  double singularity_component = 0.0;
  double soft_limit_component = 0.0;
  double duration_component = 0.0;
  double degradation_penalty = 0.0;
  std::string rationale;
  bool selected = false;
};

struct PlannerSelectionPolicyDescriptor {
  std::string name{"risk_weighted"};
  std::string summary;
};

[[nodiscard]] std::vector<PlannerExecutionCandidate> buildPlannerExecutionCandidates(
    const MotionRequest &request,
    const PlannerPreflightReport &preflight);

[[nodiscard]] PlannerExecutionCandidate selectPlannerExecutionCandidate(
    std::vector<PlannerExecutionCandidate> &candidates);

[[nodiscard]] std::vector<std::string> summarizePlannerExecutionCandidates(
    const std::vector<PlannerExecutionCandidate> &candidates);
[[nodiscard]] PlannerSelectionPolicyDescriptor describePlannerSelectionPolicy();

}  // namespace rokae_xmate3_ros2::runtime

#endif
