#include "runtime/planning_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr double kConfAngleStrictToleranceDeg = 45.0;
constexpr double kJointBranchJumpThreshold = 0.75;

struct ConfScore {
  double penalty = 0.0;
  bool strict_match = true;
  bool has_request = false;
};

double normalize_angle(double value) {
  while (value > M_PI) {
    value -= 2.0 * M_PI;
  }
  while (value < -M_PI) {
    value += 2.0 * M_PI;
  }
  return value;
}

double normalize_degree(double value) {
  while (value > 180.0) {
    value -= 360.0;
  }
  while (value < -180.0) {
    value += 360.0;
  }
  return value;
}

double shortest_degree_distance(double lhs, double rhs) {
  return std::fabs(normalize_degree(lhs - rhs));
}

bool violates_soft_limit(const std::vector<double> &joints,
                         const std::array<std::array<double, 2>, 6> &soft_limits) {
  for (size_t i = 0; i < 6 && i < joints.size(); ++i) {
    if (joints[i] < soft_limits[i][0] || joints[i] > soft_limits[i][1]) {
      return true;
    }
  }
  return false;
}

ConfScore score_requested_conf(const std::vector<double> &candidate,
                               const std::vector<int> &requested_conf) {
  ConfScore score;
  for (size_t i = 0; i < 6 && i < candidate.size() && i < requested_conf.size(); ++i) {
    const int requested = requested_conf[i];
    if (requested == 0) {
      continue;
    }

    score.has_request = true;
    if (std::abs(requested) <= 2) {
      const int sign = std::fabs(candidate[i]) < 1e-6 ? 0 : (candidate[i] > 0.0 ? 1 : -1);
      if (sign != requested) {
        score.penalty += 180.0;
        score.strict_match = false;
      }
      continue;
    }

    const double candidate_deg = candidate[i] * 180.0 / M_PI;
    const double delta_deg = shortest_degree_distance(candidate_deg, static_cast<double>(requested));
    score.penalty += delta_deg;
    if (delta_deg > kConfAngleStrictToleranceDeg) {
      score.strict_match = false;
    }
  }
  return score;
}

double cartesian_error_score(const std::vector<double> &expected,
                             const std::vector<double> &actual) {
  const double pos_error = std::sqrt(
      std::pow(actual[0] - expected[0], 2) +
      std::pow(actual[1] - expected[1], 2) +
      std::pow(actual[2] - expected[2], 2));
  const double ori_error = std::fabs(normalize_angle(actual[3] - expected[3])) +
                           std::fabs(normalize_angle(actual[4] - expected[4])) +
                           std::fabs(normalize_angle(actual[5] - expected[5]));
  return pos_error * 1000.0 + ori_error * 100.0;
}

}  // namespace

double max_joint_step(const std::vector<double> &lhs, const std::vector<double> &rhs) {
  double max_step = 0.0;
  for (size_t i = 0; i < 6 && i < lhs.size() && i < rhs.size(); ++i) {
    max_step = std::max(max_step, std::fabs(lhs[i] - rhs[i]));
  }
  return max_step;
}

IkSelection select_ik_solution(
    ::gazebo::xMate3Kinematics &kinematics,
    const std::vector<std::vector<double>> &candidates,
    const std::vector<double> &target_pose,
    const std::vector<double> &seed_joints,
    const std::vector<int> &requested_conf,
    bool strict_conf,
    bool avoid_singularity,
    bool soft_limit_enabled,
    const std::array<std::array<double, 2>, 6> &soft_limits) {
  IkSelection best;
  IkSelection fallback_singular;
  double best_score = std::numeric_limits<double>::infinity();
  double fallback_singular_score = std::numeric_limits<double>::infinity();

  for (const auto &candidate : candidates) {
    if (candidate.size() < 6) {
      continue;
    }
    if (soft_limit_enabled && violates_soft_limit(candidate, soft_limits)) {
      continue;
    }

    const auto fk_pose = kinematics.forwardKinematicsRPY(candidate);
    const auto conf_score = score_requested_conf(candidate, requested_conf);
    if (conf_score.has_request && strict_conf && !conf_score.strict_match) {
      continue;
    }

    double joint_distance = 0.0;
    for (size_t i = 0; i < 6 && i < seed_joints.size(); ++i) {
      joint_distance += std::fabs(candidate[i] - seed_joints[i]);
    }
    const bool near_singularity = kinematics.isNearSingularity(candidate);
    const double singularity_measure = kinematics.computeSingularityMeasure(candidate);
    const double score = cartesian_error_score(target_pose, fk_pose) + joint_distance * 2.0 +
                         conf_score.penalty + singularity_measure * 40.0;

    if (avoid_singularity && near_singularity) {
      if (score < fallback_singular_score) {
        fallback_singular.success = true;
        fallback_singular.joints = candidate;
        fallback_singular_score = score;
      }
      continue;
    }

    if (score < best_score) {
      best.success = true;
      best.joints = candidate;
      best_score = score;
    }
  }

  if (!best.success && fallback_singular.success) {
    fallback_singular.message =
        "selected a near-singular IK branch because no better branch was available";
    return fallback_singular;
  }

  if (!best.success) {
    best.message = (!requested_conf.empty() && strict_conf)
                       ? "no IK solution matches requested confData"
                       : "no valid IK solution";
  }
  return best;
}

bool build_joint_trajectory_from_cartesian(
    ::gazebo::xMate3Kinematics &kinematics,
    const std::vector<std::vector<double>> &cartesian_trajectory,
    const std::vector<double> &initial_seed,
    const std::vector<int> &requested_conf,
    bool strict_conf,
    bool avoid_singularity,
    bool soft_limit_enabled,
    const std::array<std::array<double, 2>, 6> &soft_limits,
    std::vector<std::vector<double>> &joint_trajectory,
    std::vector<double> &last_joints,
    std::string &error_message) {
  joint_trajectory.clear();
  last_joints = initial_seed;

  for (const auto &pose : cartesian_trajectory) {
    std::vector<double> resolved_joints;

    const auto fast_solution = kinematics.inverseKinematicsSeededFast(pose, last_joints);
    if (!fast_solution.empty()) {
      const auto conf_score = score_requested_conf(fast_solution, requested_conf);
      const bool conf_ok = !conf_score.has_request || !strict_conf || conf_score.strict_match;
      const bool soft_limit_ok = !soft_limit_enabled || !violates_soft_limit(fast_solution, soft_limits);
      const bool singularity_ok = !avoid_singularity || !kinematics.isNearSingularity(fast_solution);
      const bool continuity_ok = joint_trajectory.empty() ||
                                 max_joint_step(last_joints, fast_solution) <= kJointBranchJumpThreshold;
      if (conf_ok && soft_limit_ok && singularity_ok && continuity_ok) {
        resolved_joints = fast_solution;
      }
    }

    if (resolved_joints.empty()) {
      const auto candidates = kinematics.inverseKinematicsMultiSolution(pose, last_joints);
      const auto selected = select_ik_solution(kinematics, candidates, pose, last_joints,
                                               requested_conf, strict_conf, avoid_singularity,
                                               soft_limit_enabled, soft_limits);
      if (!selected.success) {
        error_message = selected.message;
        return false;
      }
      resolved_joints = selected.joints;
    }

    if (!joint_trajectory.empty() &&
        max_joint_step(last_joints, resolved_joints) > kJointBranchJumpThreshold) {
      error_message = "IK branch changed discontinuously along Cartesian path";
      return false;
    }
    joint_trajectory.push_back(resolved_joints);
    last_joints = resolved_joints;
  }

  return !joint_trajectory.empty();
}

}  // namespace rokae_xmate3_ros2::runtime
