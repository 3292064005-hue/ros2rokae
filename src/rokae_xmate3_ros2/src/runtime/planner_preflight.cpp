#include "runtime/planner_preflight.hpp"

#include <algorithm>
#include <cmath>

namespace rokae_xmate3_ros2::runtime {
namespace {

bool isFiniteVector(const std::vector<double> &values) {
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

bool isFiniteConf(const std::vector<int> &values) {
  return std::all_of(values.begin(), values.end(), [](int value) { return value > -1000000 && value < 1000000; });
}

const MotionCommandSpec *dominantCommand(const MotionRequest &request) {
  if (request.commands.empty()) {
    return nullptr;
  }
  return &request.commands.front();
}

}  // namespace

PlannerPreflightReport runPlannerPreflight(const MotionRequest &request) {
  PlannerPreflightReport report;
  report.request_id = request.request_id;
  report.command_count = request.commands.size();
  report.strict_conf = request.strict_conf;
  report.avoid_singularity = request.avoid_singularity;
  report.soft_limit_enabled = request.soft_limit_enabled;
  report.branch_policy = request.strict_conf ? "strict_conf" : "nearest_seed";
  report.fallback_permitted = !request.strict_conf;
  if (const auto *command = dominantCommand(request)) {
    report.dominant_motion_kind = to_string(command->kind);
  }

  if (request.commands.empty()) {
    report.reject_reason = "unreachable_pose";
    report.detail = "motion request is empty";
    return report;
  }
  if (request.start_joints.size() < 6) {
    report.reject_reason = "unreachable_pose";
    report.detail = "start_joints must contain 6 values";
    return report;
  }
  if (!isFiniteVector(request.start_joints)) {
    report.reject_reason = "unreachable_pose";
    report.detail = "start_joints must be finite";
    return report;
  }
  if (!std::isfinite(request.trajectory_dt) || request.trajectory_dt <= 0.0) {
    report.reject_reason = "unreachable_pose";
    report.detail = "trajectory_dt must be positive and finite";
    return report;
  }
  if (!std::isfinite(request.speed_scale) || request.speed_scale <= 0.0) {
    report.reject_reason = "unreachable_pose";
    report.detail = "speed_scale must be positive and finite";
    return report;
  }

  report.notes.push_back("planner_preflight=passed");
  report.notes.push_back("planner_profile=" + report.request_profile);
  report.notes.push_back("planner_primary_backend=" + report.primary_backend);
  report.notes.push_back("planner_aux_backend=" + report.auxiliary_backend);
  report.notes.push_back("planner_retimer_family=" + report.retimer_family);
  report.notes.push_back("planner_branch_policy=" + report.branch_policy);
  report.notes.push_back("planner_motion_kind=" + report.dominant_motion_kind);
  report.notes.push_back(std::string("planner_fallback_permitted=") + (report.fallback_permitted ? "true" : "false"));
  if (request.soft_limit_enabled) {
    report.notes.push_back("planner_preflight=soft_limit_enabled");
  }
  if (request.strict_conf) {
    report.notes.push_back("planner_preflight=strict_conf");
  }
  if (request.avoid_singularity) {
    report.notes.push_back("planner_preflight=avoid_singularity");
  }

  for (std::size_t index = 0; index < request.commands.size(); ++index) {
    const auto &command = request.commands[index];
    if (command.kind == MotionKind::none) {
      report.reject_reason = "unreachable_pose";
      report.detail = "command[" + std::to_string(index) + "] has unsupported motion kind";
      return report;
    }
    if (!command.target_joints.empty() && !isFiniteVector(command.target_joints)) {
      report.reject_reason = "unreachable_pose";
      report.detail = "command[" + std::to_string(index) + "] target_joints must be finite";
      return report;
    }
    if (!command.target_cartesian.empty() && !isFiniteVector(command.target_cartesian)) {
      report.reject_reason = "unreachable_pose";
      report.detail = "command[" + std::to_string(index) + "] target_cartesian must be finite";
      return report;
    }
    if (!command.aux_cartesian.empty() && !isFiniteVector(command.aux_cartesian)) {
      report.reject_reason = "unreachable_pose";
      report.detail = "command[" + std::to_string(index) + "] aux_cartesian must be finite";
      return report;
    }
    if (!command.requested_conf.empty() && !isFiniteConf(command.requested_conf)) {
      report.reject_reason = "unreachable_pose";
      report.detail = "command[" + std::to_string(index) + "] requested_conf is invalid";
      return report;
    }
    if (command.use_preplanned_trajectory && command.preplanned_trajectory.empty()) {
      report.reject_reason = "unreachable_pose";
      report.detail = "command[" + std::to_string(index) + "] preplanned trajectory is empty";
      return report;
    }
  }

  report.ok = true;
  return report;
}

}  // namespace rokae_xmate3_ros2::runtime
