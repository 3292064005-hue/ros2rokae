#include "gazebo/kinematics_backend.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace gazebo::detail {
namespace {

using Matrix4d = Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

constexpr std::array<double, 6> kDhA{0.0, 0.0, 0.394, 0.0, 0.0, 0.0};
constexpr std::array<double, 6> kDhAlpha{
    0.0,
    -M_PI / 2.0,
    0.0,
    M_PI / 2.0,
    -M_PI / 2.0,
    M_PI / 2.0,
};
constexpr std::array<double, 6> kDhD{0.3415, 0.0, 0.0, 0.366, 0.0, 0.2503};
constexpr std::array<double, 6> kJointOffset{0.0, -M_PI / 2.0, M_PI / 2.0, 0.0, 0.0, 0.0};
constexpr std::array<double, 6> kJointLimitsMin{-3.0527, -2.0933, -2.0933, -3.0527, -2.0933, -6.1082};
constexpr std::array<double, 6> kJointLimitsMax{3.0527, 2.0933, 2.0933, 3.0527, 2.0933, 6.1082};
constexpr std::array<double, 6> kSmokeReferenceJoints{0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
constexpr const char *kPackageName = "rokae_xmate3_ros2";
constexpr const char *kModelRoot = "xMate3_base";
constexpr const char *kModelTip = "flange";
constexpr double kConfAngleStrictToleranceDeg = 45.0;

struct ConfScore {
  double penalty = 0.0;
  bool strict_match = true;
  bool has_request = false;
};

Matrix4d legacyDhTransform(std::size_t index, double theta) {
  Matrix4d transform = Matrix4d::Identity();
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const double ca = std::cos(kDhAlpha[index]);
  const double sa = std::sin(kDhAlpha[index]);

  transform << ct, -st, 0.0, kDhA[index], st * ca, ct * ca, -sa, -kDhD[index] * sa, st * sa, ct * sa, ca,
      kDhD[index] * ca, 0.0, 0.0, 0.0, 1.0;
  return transform;
}

std::vector<double> adjustedLegacyJoints(const std::vector<double> &joints) {
  std::vector<double> adjusted = joints;
  if (adjusted.size() >= 3) {
    adjusted[1] -= M_PI / 2.0;
    adjusted[2] += M_PI / 2.0;
  }
  return adjusted;
}

Matrix4d legacyForwardKinematics(const std::vector<double> &joints) {
  const auto adjusted = adjustedLegacyJoints(joints);
  Matrix4d transform = Matrix4d::Identity();
  for (std::size_t index = 0; index < 6 && index < adjusted.size(); ++index) {
    transform *= legacyDhTransform(index, adjusted[index]);
  }
  return transform;
}

std::vector<Matrix4d> legacyAllTransforms(const std::vector<double> &joints) {
  std::vector<Matrix4d> transforms(7, Matrix4d::Identity());
  const auto adjusted = adjustedLegacyJoints(joints);
  for (std::size_t index = 0; index < 6 && index < adjusted.size(); ++index) {
    transforms[index + 1] = transforms[index] * legacyDhTransform(index, adjusted[index]);
  }
  return transforms;
}

Vector6d poseError(const Matrix4d &target, const Matrix4d &current) {
  Vector6d error = Vector6d::Zero();
  error.head<3>() = target.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);

  const Eigen::Matrix3d rotation_delta = target.block<3, 3>(0, 0) * current.block<3, 3>(0, 0).transpose();
  const Eigen::AngleAxisd angle_axis(rotation_delta);
  Eigen::Vector3d orientation_error = angle_axis.angle() * angle_axis.axis();
  if (!std::isfinite(orientation_error.norm()) || angle_axis.angle() < 1e-6) {
    orientation_error << rotation_delta(2, 1) - rotation_delta(1, 2),
        rotation_delta(0, 2) - rotation_delta(2, 0),
        rotation_delta(1, 0) - rotation_delta(0, 1);
    orientation_error *= 0.5;
  }
  error.tail<3>() = orientation_error;
  return error;
}

double normalizeDegree(double value) {
  while (value > 180.0) {
    value -= 360.0;
  }
  while (value < -180.0) {
    value += 360.0;
  }
  return value;
}

double shortestDegreeDistance(double lhs, double rhs) {
  return std::fabs(normalizeDegree(lhs - rhs));
}

bool violatesSoftLimit(const std::vector<double> &joints,
                       const std::array<std::array<double, 2>, 6> &soft_limits) {
  for (std::size_t index = 0; index < 6 && index < joints.size(); ++index) {
    if (joints[index] < soft_limits[index][0] || joints[index] > soft_limits[index][1]) {
      return true;
    }
  }
  return false;
}

ConfScore scoreRequestedConf(const std::vector<double> &candidate, const std::vector<int> &requested_conf) {
  ConfScore score;
  for (std::size_t index = 0; index < 6 && index < candidate.size() && index < requested_conf.size(); ++index) {
    const int requested = requested_conf[index];
    if (requested == 0) {
      continue;
    }

    score.has_request = true;
    if (std::abs(requested) <= 2) {
      const int sign = std::fabs(candidate[index]) < 1e-6 ? 0 : (candidate[index] > 0.0 ? 1 : -1);
      if (sign != requested) {
        score.penalty += 180.0;
        score.strict_match = false;
      }
      continue;
    }

    const double candidate_deg = candidate[index] * 180.0 / M_PI;
    const double delta_deg = shortestDegreeDistance(candidate_deg, static_cast<double>(requested));
    score.penalty += delta_deg;
    if (delta_deg > kConfAngleStrictToleranceDeg) {
      score.strict_match = false;
    }
  }
  return score;
}

double cartesianErrorScore(const Matrix4d &target_transform, const Matrix4d &candidate_transform) {
  const Eigen::Vector3d position_error =
      candidate_transform.block<3, 1>(0, 3) - target_transform.block<3, 1>(0, 3);
  const Eigen::Quaterniond q_target(target_transform.block<3, 3>(0, 0));
  const Eigen::Quaterniond q_candidate(candidate_transform.block<3, 3>(0, 0));
  const double orientation_error = q_target.angularDistance(q_candidate);
  return position_error.norm() * 1000.0 + orientation_error * 100.0;
}

KinematicsBackend::VectorJ defaultJointLimitVector(const std::vector<double> &candidate,
                                                   const std::array<double, 6> &fallback) {
  if (candidate.size() >= 6) {
    return candidate;
  }
  return KinematicsBackend::VectorJ(fallback.begin(), fallback.end());
}

KinematicsBackend::SeededIkRequest makeCartesianSeededIkRequest(
    const Matrix4d &target_transform,
    const KinematicsBackend::VectorJ &seed_joints,
    const KinematicsBackend::CartesianIkOptions &options) {
  KinematicsBackend::SeededIkRequest request;
  request.target_transform = target_transform;
  request.seed_joints = seed_joints;
  request.joint_limits_min = defaultJointLimitVector(options.joint_limits_min, kJointLimitsMin);
  request.joint_limits_max = defaultJointLimitVector(options.joint_limits_max, kJointLimitsMax);
  request.max_iter = 16;
  request.position_tolerance = 1e-5;
  request.orientation_tolerance = 1e-3;
  request.orientation_weight = 0.8;
  request.max_joint_step = 0.05;
  request.min_lambda = 0.02;
  request.max_lambda = 0.5;
  request.solution_valid_threshold = 5e-3;
  return request;
}

double jointLimitPenalty(const std::vector<double> &joints,
                         const std::vector<double> &joint_limits_min,
                         const std::vector<double> &joint_limits_max) {
  double penalty = 0.0;
  for (std::size_t index = 0; index < 6 && index < joints.size() && index < joint_limits_min.size() &&
                              index < joint_limits_max.size();
       ++index) {
    const double lower_margin = joints[index] - joint_limits_min[index];
    const double upper_margin = joint_limits_max[index] - joints[index];
    const double range = std::max(joint_limits_max[index] - joint_limits_min[index], 1e-6);
    const double normalized_margin = std::clamp(std::min(lower_margin, upper_margin) / range, 0.0, 1.0);
    penalty += (1.0 - normalized_margin);
  }
  return penalty / 6.0;
}

std::vector<KinematicsBackend::VectorJ> makeMultiBranchSeeds(const KinematicsBackend::SeededIkRequest &request) {
  std::vector<KinematicsBackend::VectorJ> seeds;
  if (request.seed_joints.size() < 6 || request.joint_limits_min.size() < 6 || request.joint_limits_max.size() < 6) {
    return seeds;
  }

  auto clamp_seed = [&](KinematicsBackend::VectorJ seed) {
    for (std::size_t index = 0; index < 6; ++index) {
      seed[index] = std::clamp(seed[index], request.joint_limits_min[index], request.joint_limits_max[index]);
    }
    return seed;
  };

  seeds.push_back(clamp_seed(request.seed_joints));

  auto shoulder_left = request.seed_joints;
  shoulder_left[0] = 2.0;
  shoulder_left[4] = 0.3;
  seeds.push_back(clamp_seed(std::move(shoulder_left)));

  auto shoulder_right = request.seed_joints;
  shoulder_right[0] = -2.0;
  shoulder_right[4] = -0.3;
  seeds.push_back(clamp_seed(std::move(shoulder_right)));

  auto elbow_up = request.seed_joints;
  elbow_up[2] = 1.5;
  elbow_up[4] = 0.4;
  seeds.push_back(clamp_seed(std::move(elbow_up)));

  auto elbow_down = request.seed_joints;
  elbow_down[2] = -1.5;
  elbow_down[4] = -0.4;
  seeds.push_back(clamp_seed(std::move(elbow_down)));

  auto wrist_positive = request.seed_joints;
  wrist_positive[4] = 0.5;
  seeds.push_back(clamp_seed(std::move(wrist_positive)));

  auto wrist_negative = request.seed_joints;
  wrist_negative[4] = -0.5;
  seeds.push_back(clamp_seed(std::move(wrist_negative)));

  auto wrist_flip = request.seed_joints;
  wrist_flip[5] = M_PI;
  wrist_flip[4] = 0.25;
  seeds.push_back(clamp_seed(std::move(wrist_flip)));

  auto wrist_no_flip = request.seed_joints;
  wrist_no_flip[5] = 0.0;
  wrist_no_flip[4] = -0.25;
  seeds.push_back(clamp_seed(std::move(wrist_no_flip)));

  return seeds;
}

struct SeededIkAnalysis {
  Matrix6d jacobian = Matrix6d::Zero();
  double min_sigma = 0.0;
  bool near_singularity = true;
  double singularity_measure = 1.0;
};

SeededIkAnalysis analyzeSeededIk(const KinematicsBackend &backend,
                                 const KinematicsBackend::SeededIkRequest &request,
                                 const std::vector<double> &joints) {
  SeededIkAnalysis analysis;
  if (joints.size() < 6) {
    return analysis;
  }
  const double j5 = std::fabs(joints[4]);
  double wrist_measure = 0.0;
  if (j5 < request.wrist_singularity_threshold) {
    wrist_measure = 1.0 - (j5 / std::max(request.wrist_singularity_threshold, 1e-6));
  }

  analysis.jacobian = backend.computeJacobian(joints);
  Eigen::JacobiSVD<Matrix6d> svd(analysis.jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (svd.singularValues().size() == 0) {
    return analysis;
  }

  analysis.min_sigma = svd.singularValues().tail<1>()(0);
  const double jacobian_measure = std::clamp(
      1.0 - (analysis.min_sigma / std::max(request.jacobian_singularity_threshold, 1e-6)), 0.0, 1.0);
  analysis.singularity_measure = std::max(wrist_measure, jacobian_measure);
  analysis.near_singularity = std::fabs(joints[4]) < request.wrist_singularity_threshold ||
                              analysis.min_sigma < request.jacobian_singularity_threshold;
  return analysis;
}

double branchDistance(const std::vector<double> &lhs, const std::vector<double> &rhs) {
  double max_distance = 0.0;
  for (std::size_t index = 0; index < 6 && index < lhs.size() && index < rhs.size(); ++index) {
    max_distance = std::max(max_distance, std::fabs(lhs[index] - rhs[index]));
  }
  return max_distance;
}

double continuityCost(const KinematicsBackend::SeededIkRequest &request, const std::vector<double> &candidate) {
  if (candidate.size() < 6 || request.seed_joints.size() < 6) {
    return std::numeric_limits<double>::infinity();
  }
  return request.continuity_weight *
         (Eigen::Map<const Eigen::VectorXd>(candidate.data(), 6) -
          Eigen::Map<const Eigen::VectorXd>(request.seed_joints.data(), 6))
             .norm();
}

KinematicsBackend::SeededIkCandidateMetrics evaluateSeededIkCandidate(const KinematicsBackend &backend,
                                                                      const KinematicsBackend::SeededIkRequest &request,
                                                                      const std::vector<double> &candidate) {
  KinematicsBackend::SeededIkCandidateMetrics metrics;
  if (candidate.size() < 6) {
    return metrics;
  }

  const auto singularity = analyzeSeededIk(backend, request, candidate);
  metrics.branch_distance = branchDistance(candidate, request.seed_joints);
  metrics.continuity_cost = continuityCost(request, candidate);
  metrics.joint_limit_penalty =
      request.joint_limit_weight * jointLimitPenalty(candidate, request.joint_limits_min, request.joint_limits_max);
  metrics.singularity_metric = singularity.singularity_measure;
  metrics.singularity_cost = request.singularity_weight * singularity.singularity_measure;
  metrics.near_singularity = singularity.near_singularity;
  metrics.valid = true;
  return metrics;
}

bool nudgeAwayFromSingularity(const KinematicsBackend::SeededIkRequest &request, std::vector<double> &joints);

KinematicsBackend::CartesianIkSelectionResult selectBestCartesianIkSolution(
    const KinematicsBackend &backend,
    const std::vector<KinematicsBackend::VectorJ> &candidates,
    const Matrix4d &target_transform,
    const KinematicsBackend::VectorJ &seed_joints,
    const KinematicsBackend::CartesianIkOptions &options) {
  KinematicsBackend::CartesianIkSelectionResult best;
  KinematicsBackend::CartesianIkSelectionResult fallback_singular;
  double best_score = std::numeric_limits<double>::infinity();
  double fallback_singular_score = std::numeric_limits<double>::infinity();
  const auto request = makeCartesianSeededIkRequest(target_transform, seed_joints, options);

  for (const auto &candidate : candidates) {
    if (candidate.size() < 6) {
      continue;
    }
    if (options.soft_limit_enabled && violatesSoftLimit(candidate, options.soft_limits)) {
      continue;
    }

    const auto conf_score = scoreRequestedConf(candidate, options.requested_conf);
    if (conf_score.has_request && options.strict_conf && !conf_score.strict_match) {
      continue;
    }

    const auto candidate_metrics = backend.evaluateSeededIkCandidate(request, candidate);
    if (!candidate_metrics.valid) {
      continue;
    }

    const auto fk_transform = backend.computeForwardTransform(candidate);
    const double score = cartesianErrorScore(target_transform, fk_transform) +
                         candidate_metrics.branch_distance * 2.0 + conf_score.penalty +
                         candidate_metrics.singularity_metric * 40.0 + candidate_metrics.joint_limit_penalty * 10.0;

    if (options.avoid_singularity && candidate_metrics.near_singularity) {
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
    fallback_singular.message = "selected a near-singular IK branch because no better branch was available";
    return fallback_singular;
  }

  if (!best.success) {
    best.message = (!options.requested_conf.empty() && options.strict_conf) ? "no IK solution matches requested confData"
                                                                            : "no valid IK solution";
  }
  return best;
}

bool isFastCartesianSolutionAcceptable(const KinematicsBackend &backend,
                                       const std::vector<double> &candidate,
                                       const Matrix4d &target_transform,
                                       const std::vector<double> &seed_joints,
                                       const KinematicsBackend::CartesianIkOptions &options,
                                       bool is_first_point) {
  if (candidate.empty()) {
    return false;
  }

  const auto conf_score = scoreRequestedConf(candidate, options.requested_conf);
  const bool conf_ok = !conf_score.has_request || !options.strict_conf || conf_score.strict_match;
  const bool soft_limit_ok = !options.soft_limit_enabled || !violatesSoftLimit(candidate, options.soft_limits);
  const auto request = makeCartesianSeededIkRequest(target_transform, seed_joints, options);
  const auto candidate_metrics = backend.evaluateSeededIkCandidate(request, candidate);
  const bool singularity_ok = !options.avoid_singularity || !candidate_metrics.near_singularity;
  const bool continuity_ok = is_first_point || backend.branchDistance(seed_joints, candidate) <= options.branch_jump_threshold;
  return conf_ok && soft_limit_ok && singularity_ok && continuity_ok;
}

bool buildCartesianJointTrajectory(const KinematicsBackend &backend,
                                   const std::vector<Matrix4d> &cartesian_transforms,
                                   const KinematicsBackend::VectorJ &initial_seed,
                                   const KinematicsBackend::CartesianIkOptions &options,
                                   std::vector<KinematicsBackend::VectorJ> &joint_trajectory,
                                   KinematicsBackend::VectorJ &last_joints,
                                   std::string &error_message) {
  joint_trajectory.clear();
  last_joints = initial_seed;

  for (const auto &target_transform : cartesian_transforms) {
    KinematicsBackend::VectorJ resolved_joints;
    const auto fast_request = makeCartesianSeededIkRequest(target_transform, last_joints, options);
    const auto fast_solution = backend.inverseKinematicsSeeded(fast_request);
    if (isFastCartesianSolutionAcceptable(
            backend, fast_solution, target_transform, last_joints, options, joint_trajectory.empty())) {
      resolved_joints = fast_solution;
    }

    if (resolved_joints.empty() && options.avoid_singularity) {
      auto singularity_retry_seed = last_joints;
      if (nudgeAwayFromSingularity(fast_request, singularity_retry_seed)) {
        auto singularity_retry_request = fast_request;
        singularity_retry_request.seed_joints = singularity_retry_seed;
        const auto singularity_retry = backend.inverseKinematicsSeeded(singularity_retry_request);
        if (isFastCartesianSolutionAcceptable(
                backend, singularity_retry, target_transform, last_joints, options, joint_trajectory.empty())) {
          resolved_joints = singularity_retry;
        }
      }
    }

    if (resolved_joints.empty()) {
      const auto candidates = backend.inverseKinematicsMultiBranch(fast_request);
      const auto selected =
          selectBestCartesianIkSolution(backend, candidates, target_transform, last_joints, options);
      if (!selected.success) {
        error_message = selected.message;
        return false;
      }
      resolved_joints = selected.joints;
    }

    if (!joint_trajectory.empty() && backend.branchDistance(last_joints, resolved_joints) > options.branch_jump_threshold) {
      auto local_retry_seed = last_joints;
      if (nudgeAwayFromSingularity(fast_request, local_retry_seed)) {
        auto local_retry_request = fast_request;
        local_retry_request.seed_joints = local_retry_seed;
        const auto local_retry = backend.inverseKinematicsSeeded(local_retry_request);
        if (isFastCartesianSolutionAcceptable(
                backend, local_retry, target_transform, last_joints, options, false)) {
          resolved_joints = local_retry;
        }
      }
      if (backend.branchDistance(last_joints, resolved_joints) > options.branch_jump_threshold) {
        error_message = "IK branch changed discontinuously along Cartesian path";
        return false;
      }
    }

    joint_trajectory.push_back(resolved_joints);
    last_joints = resolved_joints;
  }

  return !joint_trajectory.empty();
}

bool nudgeAwayFromSingularity(const KinematicsBackend::SeededIkRequest &request, std::vector<double> &joints) {
  if (joints.size() < 6 || request.joint_limits_min.size() < 6 || request.joint_limits_max.size() < 6) {
    return false;
  }

  const double current_j5 = joints[4];
  const double target_offset = request.singularity_avoidance_offset;
  joints[4] = current_j5 >= 0.0 ? std::max(target_offset, request.joint_limits_min[4])
                                : std::min(-target_offset, request.joint_limits_max[4]);
  joints[4] = std::clamp(joints[4], request.joint_limits_min[4], request.joint_limits_max[4]);
  joints[2] = std::clamp(joints[2] + (joints[2] >= 0.0 ? 0.12 : -0.12),
                         request.joint_limits_min[2], request.joint_limits_max[2]);
  joints[3] = std::clamp(joints[3] + (joints[3] >= 0.0 ? 0.08 : -0.08),
                         request.joint_limits_min[3], request.joint_limits_max[3]);
  joints[5] = std::clamp(joints[5] + (joints[5] >= 0.0 ? 0.1 : -0.1),
                         request.joint_limits_min[5], request.joint_limits_max[5]);
  return true;
}

std::vector<double> solveSeededIk(const KinematicsBackend &backend,
                                  const KinematicsBackend::SeededIkRequest &request) {
  if (request.seed_joints.size() < 6 || request.joint_limits_min.size() < 6 || request.joint_limits_max.size() < 6) {
    return {};
  }

  std::vector<double> joints = request.seed_joints;
  const auto initial_singularity = analyzeSeededIk(backend, request, joints);
  if (initial_singularity.near_singularity) {
    nudgeAwayFromSingularity(request, joints);
  }

  double best_score = std::numeric_limits<double>::infinity();
  std::vector<double> best_joints = joints;

  for (int iter = 0; iter < request.max_iter; ++iter) {
    const Matrix4d current_transform = backend.computeForwardTransform(joints);
    const Vector6d error = poseError(request.target_transform, current_transform);

    const auto singularity = analyzeSeededIk(backend, request, joints);
    const auto candidate_metrics = evaluateSeededIkCandidate(backend, request, joints);
    const double pos_err = error.head<3>().norm();
    const double ori_err = error.tail<3>().norm();
    const double current_score =
        pos_err + request.orientation_weight * ori_err + candidate_metrics.totalCost();
    if (current_score < best_score) {
      best_score = current_score;
      best_joints = joints;
    }
    if (pos_err < request.position_tolerance && ori_err < request.orientation_tolerance) {
      break;
    }

    Matrix6d weight = Matrix6d::Identity();
    weight(3, 3) = weight(4, 4) = weight(5, 5) = request.orientation_weight;
    if (singularity.near_singularity) {
      weight(3, 3) *= 0.3;
      weight(4, 4) *= 0.3;
      weight(5, 5) *= 0.3;
    }

    Vector6d weighted_error = error;
    weighted_error.tail<3>() *= weight(3, 3);
    const Matrix6d weighted_jacobian = weight * singularity.jacobian;
    const double lambda_base =
        request.min_lambda + request.max_lambda * (1.0 - static_cast<double>(iter) / std::max(request.max_iter, 1));
    const double lambda = lambda_base * (1.0 + 2.0 * singularity.singularity_measure);
    const Matrix6d jj_t = weighted_jacobian * weighted_jacobian.transpose();
    Vector6d delta_q =
        weighted_jacobian.transpose() * (jj_t + lambda * lambda * Matrix6d::Identity()).inverse() * weighted_error;

    const double step_scale = singularity.near_singularity ? 0.6 : 1.0;
    for (int index = 0; index < 6; ++index) {
      double step = delta_q(index);
      step = std::clamp(step, -request.max_joint_step * step_scale, request.max_joint_step * step_scale);
      joints[index] += step;
      joints[index] = std::clamp(joints[index], request.joint_limits_min[index], request.joint_limits_max[index]);
    }
  }

  const Matrix4d best_transform = backend.computeForwardTransform(best_joints);
  const Vector6d best_error = poseError(request.target_transform, best_transform);
  if (best_error.head<3>().norm() > request.solution_valid_threshold ||
      best_error.tail<3>().norm() > request.solution_valid_threshold) {
    return {};
  }
  return best_joints;
}

std::vector<KinematicsBackend::VectorJ> solveMultiBranchIk(
    const KinematicsBackend &backend,
    const KinematicsBackend::SeededIkRequest &request) {
  constexpr double kDuplicateThreshold = 0.05;

  std::vector<KinematicsBackend::VectorJ> candidates;
  for (const auto &seed : makeMultiBranchSeeds(request)) {
    auto seeded_request = request;
    seeded_request.seed_joints = seed;
    const auto candidate = solveSeededIk(backend, seeded_request);
    if (candidate.empty()) {
      continue;
    }
    bool duplicate = false;
    for (const auto &accepted : candidates) {
      if (branchDistance(candidate, accepted) < kDuplicateThreshold) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      candidates.push_back(candidate);
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [&](const KinematicsBackend::VectorJ &lhs, const KinematicsBackend::VectorJ &rhs) {
              const auto lhs_metrics = backend.evaluateSeededIkCandidate(request, lhs);
              const auto rhs_metrics = backend.evaluateSeededIkCandidate(request, rhs);
              if (lhs_metrics.totalCost() == rhs_metrics.totalCost()) {
                return lhs_metrics.branch_distance < rhs_metrics.branch_distance;
              }
              return lhs_metrics.totalCost() < rhs_metrics.totalCost();
            });
  return candidates;
}

std::filesystem::path fallbackSourceShareDir() {
  auto path = std::filesystem::path(__FILE__);
  for (int i = 0; i < 3; ++i) {
    path = path.parent_path();
  }
  return path;
}

std::string resolvePackageShareDir() {
  const char *env_share = std::getenv("ROKAE_XMATE3_ROS2_SHARE_DIR");
  if (env_share != nullptr && *env_share != '\0' && std::filesystem::exists(env_share)) {
    return env_share;
  }
  try {
    return ament_index_cpp::get_package_share_directory(kPackageName);
  } catch (const std::exception &) {
    const auto fallback = fallbackSourceShareDir();
    if (std::filesystem::exists(fallback / "urdf" / "xMate3.xacro")) {
      return fallback.string();
    }
  }
  return {};
}

std::string readFile(const std::filesystem::path &path) {
  if (path.empty() || !std::filesystem::exists(path)) {
    return {};
  }
  std::ifstream stream(path);
  if (!stream.good()) {
    return {};
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

std::string readEnvValue(const char *name) {
  const char *value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return {};
  }
  return value;
}

std::string loadUrdfXml(std::string &origin) {
  if (const auto env_xml = readEnvValue("ROKAE_XMATE3_ROBOT_DESCRIPTION_XML"); !env_xml.empty()) {
    origin = "environment:ROKAE_XMATE3_ROBOT_DESCRIPTION_XML";
    return env_xml;
  }
  if (const auto env_xml = readEnvValue("ROKAE_XMATE3_ROBOT_DESCRIPTION"); !env_xml.empty()) {
    origin = "environment:ROKAE_XMATE3_ROBOT_DESCRIPTION";
    return env_xml;
  }

  if (const auto env_file = readEnvValue("ROKAE_XMATE3_ROBOT_DESCRIPTION_FILE"); !env_file.empty()) {
    if (const auto xml = readFile(env_file); !xml.empty()) {
      origin = "environment:ROKAE_XMATE3_ROBOT_DESCRIPTION_FILE";
      return xml;
    }
  }

  std::vector<std::filesystem::path> candidates;
  if (const auto share_dir = resolvePackageShareDir(); !share_dir.empty()) {
    const auto share_path = std::filesystem::path(share_dir);
    candidates.emplace_back(share_path / "generated" / "xMate3.urdf");
    candidates.emplace_back(share_path / "urdf" / "xMate3.urdf");
  }
#ifdef ROKAE_XMATE3_GENERATED_URDF_PATH
  candidates.emplace_back(std::filesystem::path(ROKAE_XMATE3_GENERATED_URDF_PATH));
#endif

  for (const auto &candidate : candidates) {
    if (const auto xml = readFile(candidate); !xml.empty()) {
      origin = candidate.string();
      return xml;
    }
  }
  origin = "unavailable";
  return {};
}

Matrix4d toEigen(const KDL::Frame &frame) {
  Matrix4d out = Matrix4d::Identity();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out(row, col) = frame.M(row, col);
    }
    out(row, 3) = frame.p(row);
  }
  return out;
}

std::vector<Matrix4d> computeKdlTransforms(const KDL::Chain &chain, const std::vector<double> &joints) {
  std::vector<Matrix4d> transforms(7, Matrix4d::Identity());
  KDL::Frame pose = KDL::Frame::Identity();
  std::size_t joint_index = 0;
  std::size_t transform_index = 1;
  for (unsigned int segment_index = 0; segment_index < chain.getNrOfSegments(); ++segment_index) {
    const auto &segment = chain.getSegment(segment_index);
    double position = 0.0;
    if (segment.getJoint().getType() != KDL::Joint::Fixed && joint_index < joints.size()) {
      position = joints[joint_index++];
    }
    pose = pose * segment.pose(position);
    if (segment.getJoint().getType() != KDL::Joint::Fixed && transform_index < transforms.size()) {
      transforms[transform_index++] = toEigen(pose);
    }
  }
  return transforms;
}

Matrix4d computeKdlTipTransform(const KDL::Chain &chain, const std::vector<double> &joints) {
  KDL::Frame pose = KDL::Frame::Identity();
  std::size_t joint_index = 0;
  for (unsigned int segment_index = 0; segment_index < chain.getNrOfSegments(); ++segment_index) {
    const auto &segment = chain.getSegment(segment_index);
    double position = 0.0;
    if (segment.getJoint().getType() != KDL::Joint::Fixed && joint_index < joints.size()) {
      position = joints[joint_index++];
    }
    pose = pose * segment.pose(position);
  }
  return toEigen(pose);
}

struct SharedKdlModel {
  bool valid = false;
  std::string reason;
  KDL::Chain chain;
};

bool smokeCheckKdlChain(const KDL::Chain &chain) {
  const std::vector<double> zero_joints(6, 0.0);
  const auto legacy_zero = legacyForwardKinematics(zero_joints);
  const auto kdl_zero = computeKdlTransforms(chain, zero_joints).back();
  const auto zero_error = poseError(legacy_zero, kdl_zero);

  const std::vector<double> reference_joints(kSmokeReferenceJoints.begin(), kSmokeReferenceJoints.end());
  const auto legacy_reference = legacyForwardKinematics(reference_joints);
  const auto kdl_reference = computeKdlTransforms(chain, reference_joints).back();
  const auto reference_error = poseError(legacy_reference, kdl_reference);

  return zero_error.head<3>().norm() < 1e-4 && zero_error.tail<3>().norm() < 1e-3 &&
         reference_error.head<3>().norm() < 1e-3 && reference_error.tail<3>().norm() < 1e-2;
}

SharedKdlModel buildSharedKdlModel() {
  SharedKdlModel model;
  std::string urdf_origin;
  const auto urdf_xml = loadUrdfXml(urdf_origin);
  if (urdf_xml.empty()) {
    model.reason = "failed to load URDF description";
    return model;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_xml, tree)) {
    model.reason = "kdl_parser failed to build a tree from the loaded URDF";
    return model;
  }

  if (!tree.getChain(kModelRoot, kModelTip, model.chain)) {
    model.reason = "failed to extract xMate3_base -> flange chain";
    return model;
  }

  if (model.chain.getNrOfJoints() != 6) {
    model.reason = "unexpected joint count in URDF-derived KDL chain";
    return model;
  }

  if (!smokeCheckKdlChain(model.chain)) {
    model.reason = "URDF-derived KDL chain failed legacy DH smoke validation from " + urdf_origin;
    return model;
  }

  model.valid = true;
  model.reason = "ok:" + urdf_origin;
  return model;
}

const SharedKdlModel &sharedKdlModel() {
  static const SharedKdlModel model = buildSharedKdlModel();
  return model;
}

class LegacyKinematicsBackend final : public KinematicsBackend {
 public:
  [[nodiscard]] std::string name() const override { return "legacy"; }

  [[nodiscard]] Matrix4d computeForwardTransform(const std::vector<double> &joints) const override {
    return legacyForwardKinematics(joints);
  }

  [[nodiscard]] std::vector<Matrix4d> computeAllTransforms(const std::vector<double> &joints) const override {
    return legacyAllTransforms(joints);
  }

  [[nodiscard]] Matrix6d computeJacobian(const std::vector<double> &joints) const override {
    Matrix6d jacobian = Matrix6d::Zero();
    const Matrix4d current = legacyForwardKinematics(joints);
    constexpr double kDelta = 1e-6;
    for (int axis = 0; axis < 6 && axis < static_cast<int>(joints.size()); ++axis) {
      std::vector<double> joints_plus = joints;
      joints_plus[axis] += kDelta;
      const Matrix4d plus = legacyForwardKinematics(joints_plus);
      jacobian.col(axis) = poseError(plus, current) / kDelta;
    }
    return jacobian;
  }

  [[nodiscard]] double branchDistance(const VectorJ &lhs, const VectorJ &rhs) const override {
    return gazebo::detail::branchDistance(lhs, rhs);
  }

  [[nodiscard]] double continuityCost(const SeededIkRequest &request, const VectorJ &candidate) const override {
    return gazebo::detail::continuityCost(request, candidate);
  }

  [[nodiscard]] double singularityMetric(const SeededIkRequest &request, const VectorJ &candidate) const override {
    return evaluateSeededIkCandidate(request, candidate).singularity_metric;
  }

  [[nodiscard]] SeededIkCandidateMetrics evaluateSeededIkCandidate(const SeededIkRequest &request,
                                                                   const VectorJ &candidate) const override {
    return gazebo::detail::evaluateSeededIkCandidate(*this, request, candidate);
  }

  [[nodiscard]] VectorJ inverseKinematicsSeeded(const SeededIkRequest &request) const override {
    return solveSeededIk(*this, request);
  }

  [[nodiscard]] std::vector<VectorJ> inverseKinematicsMultiBranch(const SeededIkRequest &request) const override {
    return solveMultiBranchIk(*this, request);
  }

  [[nodiscard]] CartesianIkSelectionResult selectBestCartesianIkSolution(
      const std::vector<VectorJ> &candidates,
      const Matrix4d &target_transform,
      const VectorJ &seed_joints,
      const CartesianIkOptions &options) const override {
    return gazebo::detail::selectBestCartesianIkSolution(*this, candidates, target_transform, seed_joints, options);
  }

  [[nodiscard]] bool buildCartesianJointTrajectory(const std::vector<Matrix4d> &cartesian_transforms,
                                                   const VectorJ &initial_seed,
                                                   const CartesianIkOptions &options,
                                                   std::vector<VectorJ> &joint_trajectory,
                                                   VectorJ &last_joints,
                                                   std::string &error_message) const override {
    return gazebo::detail::buildCartesianJointTrajectory(
        *this, cartesian_transforms, initial_seed, options, joint_trajectory, last_joints, error_message);
  }
};

class KdlKinematicsBackend final : public KinematicsBackend {
 public:
  KdlKinematicsBackend() {
    const auto &shared = sharedKdlModel();
    if (!shared.valid) {
      valid_ = false;
      failure_reason_ = shared.reason;
      return;
    }
    chain_ = shared.chain;
    jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    valid_ = jacobian_solver_ != nullptr;
    if (!valid_) {
      failure_reason_ = "failed to construct KDL Jacobian solver";
    }
  }

  [[nodiscard]] std::string name() const override { return valid_ ? "kdl-urdf" : "legacy"; }

  [[nodiscard]] Matrix4d computeForwardTransform(const std::vector<double> &joints) const override {
    if (!valid_) {
      return LegacyKinematicsBackend{}.computeForwardTransform(joints);
    }
    return computeKdlTipTransform(chain_, joints);
  }

  [[nodiscard]] std::vector<Matrix4d> computeAllTransforms(const std::vector<double> &joints) const override {
    if (!valid_) {
      return LegacyKinematicsBackend{}.computeAllTransforms(joints);
    }
    return computeKdlTransforms(chain_, joints);
  }

  [[nodiscard]] Matrix6d computeJacobian(const std::vector<double> &joints) const override {
    if (!valid_ || joints.size() < 6) {
      return LegacyKinematicsBackend{}.computeJacobian(joints);
    }

    KDL::JntArray joint_array(6);
    for (unsigned int index = 0; index < 6; ++index) {
      joint_array(index) = joints[index];
    }
    KDL::Jacobian jacobian(6);
    if (jacobian_solver_->JntToJac(joint_array, jacobian) < 0) {
      return LegacyKinematicsBackend{}.computeJacobian(joints);
    }

    Matrix6d result = Matrix6d::Zero();
    for (unsigned int row = 0; row < 6; ++row) {
      for (unsigned int col = 0; col < 6; ++col) {
        result(row, col) = jacobian(row, col);
      }
    }
    return result;
  }

  [[nodiscard]] double branchDistance(const VectorJ &lhs, const VectorJ &rhs) const override {
    return gazebo::detail::branchDistance(lhs, rhs);
  }

  [[nodiscard]] double continuityCost(const SeededIkRequest &request, const VectorJ &candidate) const override {
    return gazebo::detail::continuityCost(request, candidate);
  }

  [[nodiscard]] double singularityMetric(const SeededIkRequest &request, const VectorJ &candidate) const override {
    return evaluateSeededIkCandidate(request, candidate).singularity_metric;
  }

  [[nodiscard]] SeededIkCandidateMetrics evaluateSeededIkCandidate(const SeededIkRequest &request,
                                                                   const VectorJ &candidate) const override {
    if (!valid_) {
      return LegacyKinematicsBackend{}.evaluateSeededIkCandidate(request, candidate);
    }
    return gazebo::detail::evaluateSeededIkCandidate(*this, request, candidate);
  }

  [[nodiscard]] VectorJ inverseKinematicsSeeded(const SeededIkRequest &request) const override {
    if (!valid_) {
      return LegacyKinematicsBackend{}.inverseKinematicsSeeded(request);
    }
    return solveSeededIk(*this, request);
  }

  [[nodiscard]] std::vector<VectorJ> inverseKinematicsMultiBranch(const SeededIkRequest &request) const override {
    if (!valid_) {
      return LegacyKinematicsBackend{}.inverseKinematicsMultiBranch(request);
    }
    return solveMultiBranchIk(*this, request);
  }

  [[nodiscard]] CartesianIkSelectionResult selectBestCartesianIkSolution(
      const std::vector<VectorJ> &candidates,
      const Matrix4d &target_transform,
      const VectorJ &seed_joints,
      const CartesianIkOptions &options) const override {
    if (!valid_) {
      return LegacyKinematicsBackend{}.selectBestCartesianIkSolution(candidates, target_transform, seed_joints, options);
    }
    return gazebo::detail::selectBestCartesianIkSolution(*this, candidates, target_transform, seed_joints, options);
  }

  [[nodiscard]] bool buildCartesianJointTrajectory(const std::vector<Matrix4d> &cartesian_transforms,
                                                   const VectorJ &initial_seed,
                                                   const CartesianIkOptions &options,
                                                   std::vector<VectorJ> &joint_trajectory,
                                                   VectorJ &last_joints,
                                                   std::string &error_message) const override {
    if (!valid_) {
      return LegacyKinematicsBackend{}.buildCartesianJointTrajectory(
          cartesian_transforms, initial_seed, options, joint_trajectory, last_joints, error_message);
    }
    return gazebo::detail::buildCartesianJointTrajectory(
        *this, cartesian_transforms, initial_seed, options, joint_trajectory, last_joints, error_message);
  }

 private:
  bool valid_ = false;
  std::string failure_reason_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
};

}  // namespace

std::shared_ptr<KinematicsBackend> makePreferredKinematicsBackend() {
  auto backend = std::make_shared<KdlKinematicsBackend>();
  if (backend->name() != "legacy") {
    return backend;
  }
  return std::make_shared<LegacyKinematicsBackend>();
}

}  // namespace gazebo::detail
