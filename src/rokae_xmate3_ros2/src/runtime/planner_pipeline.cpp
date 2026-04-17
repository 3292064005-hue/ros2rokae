#include "runtime/planner_pipeline.hpp"

#include <algorithm>
#include <sstream>

namespace rokae_xmate3_ros2::runtime {
namespace {

bool request_has_zone(const MotionRequest &request) {
  for (const auto &command : request.commands) {
    if (command.zone > 0) {
      return true;
    }
  }
  return false;
}

bool request_has_cartesian(const MotionRequest &request) {
  for (const auto &command : request.commands) {
    switch (command.kind) {
      case MotionKind::move_j:
      case MotionKind::move_l:
      case MotionKind::move_c:
      case MotionKind::move_cf:
      case MotionKind::move_sp:
        return true;
      default:
        break;
    }
  }
  return false;
}

PlannerExecutionCandidate make_candidate(const std::string &name,
                                         const std::string &stop_point,
                                         double score,
                                         double continuity_component,
                                         double singularity_component,
                                         double soft_limit_component,
                                         double duration_component,
                                         double degradation_penalty,
                                         std::string rationale) {
  PlannerExecutionCandidate candidate;
  candidate.name = name;
  candidate.requested_stop_point = stop_point;
  candidate.score = score;
  candidate.continuity_component = continuity_component;
  candidate.singularity_component = singularity_component;
  candidate.soft_limit_component = soft_limit_component;
  candidate.duration_component = duration_component;
  candidate.degradation_penalty = degradation_penalty;
  candidate.rationale = std::move(rationale);
  return candidate;
}

}  // namespace

std::vector<PlannerExecutionCandidate> buildPlannerExecutionCandidates(
    const MotionRequest &request,
    const PlannerPreflightReport &preflight) {
  const bool has_zone = request_has_zone(request);
  const bool has_cartesian = request_has_cartesian(request);

  const double branch = std::clamp(preflight.branch_switch_risk, 0.0, 1.0);
  const double singularity = std::clamp(preflight.singularity_risk, 0.0, 1.0);
  const double continuity = std::clamp(preflight.continuity_risk, 0.0, 1.0);
  const double soft_limit = std::clamp(preflight.soft_limit_risk, 0.0, 1.0);
  const double risk_sum = 0.28 * branch + 0.26 * singularity + 0.28 * continuity + 0.18 * soft_limit;

  std::vector<PlannerExecutionCandidate> candidates;
  candidates.reserve(3);

  double nominal_bonus = 0.0;
  if (preflight.recommended_stop_point == "blended") {
    nominal_bonus += 0.12;
  }
  if (has_zone) {
    nominal_bonus += 0.05;
  }
  if (has_cartesian) {
    nominal_bonus += 0.03;
  }
  const double nominal_continuity = 0.40 * (1.0 - continuity);
  const double nominal_singularity = 0.25 * (1.0 - singularity);
  const double nominal_soft_limit = 0.20 * (1.0 - soft_limit);
  const double nominal_duration = has_zone ? 0.08 : 0.02;
  const double nominal_degradation = has_cartesian ? 0.03 * branch : 0.01 * branch;
  candidates.push_back(make_candidate(
      "nominal",
      preflight.recommended_stop_point,
      std::clamp(1.05 - risk_sum + nominal_bonus, 0.0, 2.0),
      nominal_continuity,
      nominal_singularity,
      nominal_soft_limit,
      nominal_duration,
      nominal_degradation,
      "honor preflight stop-point and preferred blending policy"));

  const std::string safer_stop_point = has_zone ? std::string{"junction_stop_point"} : std::string{"segment_end"};
  double conservative_bonus = (branch > 0.40 || continuity > 0.40 || singularity > 0.40) ? 0.18 : 0.06;
  if (preflight.recommended_stop_point != "blended") {
    conservative_bonus += 0.08;
  }
  const double conservative_continuity = 0.48 * (1.0 - continuity);
  const double conservative_singularity = 0.28 * (1.0 - singularity);
  const double conservative_soft_limit = 0.22 * (1.0 - soft_limit);
  const double conservative_duration = -0.05;
  const double conservative_degradation = 0.02 * branch;
  candidates.push_back(make_candidate(
      "conservative",
      safer_stop_point,
      std::clamp(0.96 - 0.45 * risk_sum + conservative_bonus, 0.0, 2.0),
      conservative_continuity,
      conservative_singularity,
      conservative_soft_limit,
      conservative_duration,
      conservative_degradation,
      "prefer stop-points to preserve branch continuity and reduce blended junction risk"));

  if (has_cartesian) {
    const bool high_risk = branch > 0.55 || singularity > 0.55 || continuity > 0.55;
    const double hold_continuity = 0.55 * (1.0 - continuity);
    const double hold_singularity = 0.25 * (1.0 - singularity);
    const double hold_soft_limit = 0.12 * (1.0 - soft_limit);
    const double hold_duration = has_zone ? -0.02 : 0.01;
    const double hold_degradation = 0.04 * branch;
    candidates.push_back(make_candidate(
        "cartesian_hold",
        high_risk ? std::string{"junction_stop_point"} : preflight.recommended_stop_point,
        std::clamp(0.90 - 0.35 * branch - 0.35 * singularity - 0.20 * continuity + (high_risk ? 0.16 : 0.04),
                   0.0,
                   2.0),
        hold_continuity,
        hold_singularity,
        hold_soft_limit,
        hold_duration,
        hold_degradation,
        "hold cartesian continuity and avoid aggressive mixed-family blending"));
  }

  return candidates;
}

PlannerExecutionCandidate selectPlannerExecutionCandidate(
    std::vector<PlannerExecutionCandidate> &candidates) {
  if (candidates.empty()) {
    return {};
  }
  auto best_it = std::max_element(
      candidates.begin(), candidates.end(), [](const auto &lhs, const auto &rhs) { return lhs.score < rhs.score; });
  best_it->selected = true;
  return *best_it;
}

std::vector<std::string> summarizePlannerExecutionCandidates(
    const std::vector<PlannerExecutionCandidate> &candidates) {
  std::vector<std::string> summaries;
  summaries.reserve(candidates.size());
  for (const auto &candidate : candidates) {
    std::ostringstream oss;
    oss << "plan_candidate[name=" << candidate.name << ", stop_point=" << candidate.requested_stop_point
        << ", score=" << candidate.score << ", continuity=" << candidate.continuity_component
        << ", singularity=" << candidate.singularity_component << ", soft_limit="
        << candidate.soft_limit_component << ", duration=" << candidate.duration_component
        << ", degradation_penalty=" << candidate.degradation_penalty
        << ", selected=" << (candidate.selected ? "true" : "false")
        << "] rationale=" << candidate.rationale;
    summaries.push_back(oss.str());
  }
  return summaries;
}

PlannerSelectionPolicyDescriptor describePlannerSelectionPolicy() {
  PlannerSelectionPolicyDescriptor descriptor;
  descriptor.name = "risk_weighted";
  descriptor.summary =
      "Weighted candidate selection: continuity + singularity + soft-limit margins with duration and degradation penalties";
  return descriptor;
}

}  // namespace rokae_xmate3_ros2::runtime
