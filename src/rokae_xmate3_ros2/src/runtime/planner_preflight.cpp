#include "runtime/planner_preflight.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>

#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

using rokae_xmate3_ros2::spec::xmate3::jointAccelerationLimit;
using rokae_xmate3_ros2::spec::xmate3::jointLimitMax;
using rokae_xmate3_ros2::spec::xmate3::jointLimitMin;
using rokae_xmate3_ros2::spec::xmate3::jointVelocityLimit;

bool isFiniteVector(const std::vector<double> &values) {
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

bool isFiniteConf(const std::vector<int> &values) {
  return std::all_of(values.begin(), values.end(), [](int value) { return value > -1000000 && value < 1000000; });
}

std::string formatDouble(double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4) << value;
  return stream.str();
}

const MotionCommandSpec *dominantCommand(const MotionRequest &request) {
  if (request.commands.empty()) {
    return nullptr;
  }
  return &request.commands.front();
}

bool isCartesianKind(MotionKind kind) {
  return kind == MotionKind::move_j || kind == MotionKind::move_l || kind == MotionKind::move_c ||
         kind == MotionKind::move_cf || kind == MotionKind::move_sp;
}

std::string motionRejectReason(MotionKind kind) {
  switch (kind) {
    case MotionKind::none:
      return "unsupported_motion_kind";
    case MotionKind::move_absj:
    case MotionKind::move_j:
    case MotionKind::move_l:
    case MotionKind::move_c:
    case MotionKind::move_cf:
    case MotionKind::move_sp:
      return {};
  }
  return "unsupported_motion_kind";
}

double estimateJointDuration(const std::vector<double> &from,
                             const std::vector<double> &to,
                             double speed_scale) {
  if (from.size() < 6 || to.size() < 6) {
    return 0.0;
  }
  const auto &vel_limits = jointVelocityLimit();
  const auto &acc_limits = jointAccelerationLimit();
  double duration = 0.0;
  for (std::size_t index = 0; index < 6; ++index) {
    const double delta = std::fabs(to[index] - from[index]);
    const double vmax = std::max(vel_limits[index] * std::max(speed_scale, 0.05), 1e-6);
    const double amax = std::max(acc_limits[index] * std::max(speed_scale, 0.05), 1e-6);
    const double accel_time = std::sqrt(std::max(delta / amax, 0.0));
    const double velocity_time = delta / vmax;
    duration = std::max(duration, std::max(accel_time, velocity_time));
  }
  return duration;
}

double estimateCartesianDuration(const std::vector<double> &from_pose,
                                 const std::vector<double> &to_pose,
                                 double default_speed_mm_s,
                                 double speed_scale) {
  if (from_pose.size() < 6 || to_pose.size() < 6) {
    return 0.0;
  }
  const double dx = to_pose[0] - from_pose[0];
  const double dy = to_pose[1] - from_pose[1];
  const double dz = to_pose[2] - from_pose[2];
  const double distance_m = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double angular_distance = std::sqrt(std::pow(to_pose[3] - from_pose[3], 2.0) +
                                            std::pow(to_pose[4] - from_pose[4], 2.0) +
                                            std::pow(to_pose[5] - from_pose[5], 2.0));
  const double speed_mps = std::max((std::max(default_speed_mm_s, 5.0) / 1000.0) * std::max(speed_scale, 0.05), 1e-6);
  const double linear_time = distance_m / speed_mps;
  const double angular_time = angular_distance / std::max(0.6 * std::max(speed_scale, 0.05), 1e-6);
  return std::max(linear_time, angular_time);
}

double computeSoftLimitRisk(const std::vector<double> &start_joints,
                            const std::array<std::array<double, 2>, 6> &soft_limits,
                            bool enabled) {
  if (!enabled || start_joints.size() < 6) {
    return 0.0;
  }
  double risk = 0.0;
  for (std::size_t index = 0; index < 6; ++index) {
    const double lower = soft_limits[index][0];
    const double upper = soft_limits[index][1];
    const double value = start_joints[index];
    if (value < lower || value > upper) {
      return 1.0;
    }
    const double range = std::max(upper - lower, 1e-6);
    const double margin = std::min(value - lower, upper - value) / range;
    risk = std::max(risk, 1.0 - std::clamp(2.0 * margin, 0.0, 1.0));
  }
  return std::clamp(risk, 0.0, 1.0);
}

double computeJointLimitRisk(const std::vector<double> &target_joints) {
  if (target_joints.size() < 6) {
    return 0.0;
  }
  double risk = 0.0;
  for (std::size_t index = 0; index < 6; ++index) {
    const double lower = jointLimitMin()[index];
    const double upper = jointLimitMax()[index];
    const double value = target_joints[index];
    if (value < lower || value > upper) {
      return 1.0;
    }
    const double range = std::max(upper - lower, 1e-6);
    const double margin = std::min(value - lower, upper - value) / range;
    risk = std::max(risk, 1.0 - std::clamp(2.0 * margin, 0.0, 1.0));
  }
  return std::clamp(risk, 0.0, 1.0);
}

double computeContinuityRisk(MotionKind previous_kind, MotionKind current_kind) {
  if (previous_kind == MotionKind::none || current_kind == MotionKind::none) {
    return 0.0;
  }
  if (previous_kind == current_kind) {
    return 0.1;
  }
  if (isCartesianKind(previous_kind) != isCartesianKind(current_kind)) {
    return 0.85;
  }
  if ((previous_kind == MotionKind::move_c || previous_kind == MotionKind::move_cf || previous_kind == MotionKind::move_sp) &&
      current_kind == MotionKind::move_l) {
    return 0.6;
  }
  return 0.35;
}

double computeBranchSwitchRisk(const MotionRequest &request) {
  double risk = 0.0;
  MotionKind previous_kind = MotionKind::none;
  for (const auto &command : request.commands) {
    risk = std::max(risk, computeContinuityRisk(previous_kind, command.kind));
    if (!command.requested_conf.empty()) {
      risk = std::max(risk, request.strict_conf ? 0.15 : 0.55);
    }
    previous_kind = command.kind;
  }
  return std::clamp(risk, 0.0, 1.0);
}

double computeSingularityRisk(const MotionRequest &request) {
  double risk = request.avoid_singularity ? 0.15 : 0.35;
  for (const auto &command : request.commands) {
    switch (command.kind) {
      case MotionKind::move_c:
      case MotionKind::move_cf:
      case MotionKind::move_sp:
        risk = std::max(risk, 0.55);
        break;
      case MotionKind::move_l:
      case MotionKind::move_j:
        risk = std::max(risk, 0.25);
        break;
      default:
        break;
    }
  }
  return std::clamp(risk, 0.0, 1.0);
}

std::string recommendedStopPointFromRisk(double continuity_risk, double branch_switch_risk) {
  const double risk = std::max(continuity_risk, branch_switch_risk);
  if (risk >= 0.8) {
    return "segment_start";
  }
  if (risk >= 0.45) {
    return "junction_stop_point";
  }
  return "blended";
}

void fillPolicyNotes(PlannerPreflightReport &report) {
  report.notes.push_back("planner_preflight=passed");
  report.notes.push_back("planner_profile=" + report.request_profile);
  report.notes.push_back("planner_primary_backend=" + report.primary_backend);
  report.notes.push_back("planner_aux_backend=" + report.auxiliary_backend);
  report.notes.push_back("planner_fallback_backend=" + report.fallback_backend);
  report.notes.push_back("planner_retimer_family=" + report.retimer_family);
  report.notes.push_back("planner_branch_policy=" + report.branch_policy);
  report.notes.push_back("planner_selected_branch=" + report.selected_branch);
  report.notes.push_back("planner_motion_kind=" + report.dominant_motion_kind);
  report.notes.push_back(std::string("planner_fallback_permitted=") + (report.fallback_permitted ? "true" : "false"));
  report.notes.push_back(std::string("planner_fallback_used=") + (report.fallback_used ? "true" : "false"));
  report.notes.push_back("planner_estimated_duration=" + formatDouble(report.estimated_duration));
  report.notes.push_back("planner_branch_switch_risk=" + formatDouble(report.branch_switch_risk));
  report.notes.push_back("planner_singularity_risk=" + formatDouble(report.singularity_risk));
  report.notes.push_back("planner_continuity_risk=" + formatDouble(report.continuity_risk));
  report.notes.push_back("planner_soft_limit_risk=" + formatDouble(report.soft_limit_risk));
  report.notes.push_back("planner_recommended_stop_point=" + report.recommended_stop_point);
  if (!report.fallback_reason.empty()) {
    report.notes.push_back("planner_fallback_reason=" + report.fallback_reason);
  }
  if (report.soft_limit_enabled) {
    report.notes.push_back("planner_preflight=soft_limit_enabled");
  }
  if (report.strict_conf) {
    report.notes.push_back("planner_preflight=strict_conf");
  }
  if (report.avoid_singularity) {
    report.notes.push_back("planner_preflight=avoid_singularity");
  }
}

void reject(PlannerPreflightReport &report, std::string reason, std::string detail) {
  report.ok = false;
  report.reject_reason = std::move(reason);
  report.detail = std::move(detail);
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
  report.selected_branch = report.branch_policy;
  report.fallback_permitted = !request.strict_conf;
  report.fallback_used = false;
  report.fallback_reason = report.fallback_permitted ? "policy_allows_aux_backend" : "strict_conf_disallows_relaxed_fallback";
  if (const auto *command = dominantCommand(request)) {
    report.dominant_motion_kind = to_string(command->kind);
  }

  if (request.commands.empty()) {
    reject(report, "empty_request", "motion request is empty");
    return report;
  }
  if (request.start_joints.size() != 6) {
    reject(report, "invalid_start_joint_count", "start_joints must contain exactly 6 values");
    return report;
  }
  if (!isFiniteVector(request.start_joints)) {
    reject(report, "nonfinite_start_joints", "start_joints must be finite");
    return report;
  }
  if (!std::isfinite(request.trajectory_dt) || request.trajectory_dt <= 0.0) {
    reject(report, "invalid_timing", "trajectory_dt must be positive and finite");
    return report;
  }
  if (!std::isfinite(request.speed_scale) || request.speed_scale <= 0.0) {
    reject(report, "invalid_speed_scale", "speed_scale must be positive and finite");
    return report;
  }

  report.soft_limit_risk = computeSoftLimitRisk(request.start_joints, request.soft_limits, request.soft_limit_enabled);
  if (report.soft_limit_risk >= 1.0) {
    reject(report, "soft_limit_violation", "start_joints are outside configured soft limits");
    return report;
  }

  std::vector<double> current_joints = request.start_joints;
  std::vector<double> current_pose;
  MotionKind previous_kind = MotionKind::none;
  for (std::size_t index = 0; index < request.commands.size(); ++index) {
    const auto &command = request.commands[index];
    if (const auto reason = motionRejectReason(command.kind); !reason.empty()) {
      reject(report, reason, "command[" + std::to_string(index) + "] has unsupported motion kind");
      return report;
    }
    if (!command.target_joints.empty() && !isFiniteVector(command.target_joints)) {
      reject(report, "nonfinite_target_joints", "command[" + std::to_string(index) + "] target_joints must be finite");
      return report;
    }
    if (!command.target_cartesian.empty() && !isFiniteVector(command.target_cartesian)) {
      reject(report, "nonfinite_target_pose", "command[" + std::to_string(index) + "] target_cartesian must be finite");
      return report;
    }
    if (!command.aux_cartesian.empty() && !isFiniteVector(command.aux_cartesian)) {
      reject(report, "nonfinite_aux_pose", "command[" + std::to_string(index) + "] aux_cartesian must be finite");
      return report;
    }
    if (!command.requested_conf.empty() && !isFiniteConf(command.requested_conf)) {
      reject(report, "invalid_conf_data", "command[" + std::to_string(index) + "] requested_conf is invalid");
      return report;
    }
    if (command.use_preplanned_trajectory && command.preplanned_trajectory.empty()) {
      reject(report, "preplanned_trajectory_empty", "command[" + std::to_string(index) + "] preplanned trajectory is empty");
      return report;
    }

    report.continuity_risk = std::max(report.continuity_risk, computeContinuityRisk(previous_kind, command.kind));
    previous_kind = command.kind;

    if (command.kind == MotionKind::move_absj) {
      if (command.target_joints.size() != 6) {
        reject(report, "invalid_target_joint_count", "command[" + std::to_string(index) + "] MoveAbsJ requires 6 joint values");
        return report;
      }
      const double target_joint_limit_risk = computeJointLimitRisk(command.target_joints);
      report.soft_limit_risk = std::max(report.soft_limit_risk, target_joint_limit_risk);
      if (target_joint_limit_risk >= 1.0) {
        reject(report, "joint_limit_violation", "command[" + std::to_string(index) + "] MoveAbsJ target exceeds hard joint limits");
        return report;
      }
      report.estimated_duration += estimateJointDuration(current_joints, command.target_joints, request.speed_scale);
      current_joints = command.target_joints;
      continue;
    }

    if (isCartesianKind(command.kind) && command.target_cartesian.size() != 6) {
      reject(report, "invalid_target_pose", "command[" + std::to_string(index) + "] Cartesian target must contain 6 values");
      return report;
    }
    if ((command.kind == MotionKind::move_c || command.kind == MotionKind::move_cf) && command.aux_cartesian.size() != 6) {
      reject(report, "invalid_aux_pose", "command[" + std::to_string(index) + "] circular motion requires 6-value auxiliary pose");
      return report;
    }
    if (command.kind == MotionKind::move_sp) {
      if (!std::isfinite(command.angle) || std::fabs(command.angle) < 1e-9) {
        reject(report, "invalid_spiral_angle", "command[" + std::to_string(index) + "] spiral angle must be finite and non-zero");
        return report;
      }
      if (!std::isfinite(command.radius) || command.radius < 0.0 || !std::isfinite(command.radius_step)) {
        reject(report, "invalid_spiral_radius", "command[" + std::to_string(index) + "] spiral radius parameters are invalid");
        return report;
      }
    }

    if (current_pose.empty()) {
      report.estimated_duration += std::max(
          0.25,
          0.5 * estimateCartesianDuration(std::vector<double>(6, 0.0), command.target_cartesian, request.default_speed, request.speed_scale));
    } else {
      report.estimated_duration += estimateCartesianDuration(current_pose, command.target_cartesian, request.default_speed, request.speed_scale);
    }
    current_pose = command.target_cartesian;
    if (!command.requested_conf.empty()) {
      report.branch_switch_risk = std::max(report.branch_switch_risk, request.strict_conf ? 0.15 : 0.55);
      report.selected_branch = request.strict_conf ? "strict_conf" : "conf_relaxed";
    }
  }

  report.branch_switch_risk = std::max(report.branch_switch_risk, computeBranchSwitchRisk(request));
  report.singularity_risk = computeSingularityRisk(request);
  report.recommended_stop_point = recommendedStopPointFromRisk(report.continuity_risk, report.branch_switch_risk);
  report.ok = true;
  fillPolicyNotes(report);
  return report;
}

}  // namespace rokae_xmate3_ros2::runtime
