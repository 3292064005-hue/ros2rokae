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

const KinematicsBackend::IKBackendConfig &backendConfig() noexcept {
  static const KinematicsBackend::IKBackendConfig config{};
  return config;
}

namespace {

using Matrix4d = Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using IKBackendConfig = KinematicsBackend::IKBackendConfig;

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

Eigen::Vector3d so3LogError(const Eigen::Matrix3d &target, const Eigen::Matrix3d &current) {
  Eigen::Quaterniond q_target(target);
  Eigen::Quaterniond q_current(current);
  q_target.normalize();
  q_current.normalize();
  Eigen::Quaterniond delta = q_target * q_current.conjugate();
  if (delta.w() < 0.0) {
    delta.coeffs() *= -1.0;
  }
  const Eigen::Vector3d vec = delta.vec();
  const double vec_norm = vec.norm();
  if (vec_norm < 1e-12) {
    return Eigen::Vector3d::Zero();
  }
  const double angle = 2.0 * std::atan2(vec_norm, std::max(delta.w(), 1e-12));
  return (angle / vec_norm) * vec;
}

Vector6d poseError(const Matrix4d &target, const Matrix4d &current) {
  Vector6d error = Vector6d::Zero();
  error.head<3>() = target.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);
  error.tail<3>() = so3LogError(target.block<3, 3>(0, 0), current.block<3, 3>(0, 0));
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
  const double orientation_error =
      so3LogError(target_transform.block<3, 3>(0, 0), candidate_transform.block<3, 3>(0, 0)).norm();
  return position_error.norm() * 1000.0 + orientation_error * 100.0;
}

KinematicsBackend::VectorJ defaultJointLimitVector(const std::vector<double> &candidate,
                                                   const std::array<double, 6> &fallback) {
  if (candidate.size() >= 6) {
    return candidate;
  }
  return KinematicsBackend::VectorJ(fallback.begin(), fallback.end());
}

KinematicsBackend::IKRequest makeCartesianSeededIkRequest(
    const Matrix4d &target_transform,
    const KinematicsBackend::VectorJ &seed_joints,
    const KinematicsBackend::CartesianIkOptions &options) {
  KinematicsBackend::IKRequest request;
  request.target_transform = target_transform;
  request.seed_joints = seed_joints;
  request.joint_limits_min = defaultJointLimitVector(options.joint_limits_min, kJointLimitsMin);
  request.joint_limits_max = defaultJointLimitVector(options.joint_limits_max, kJointLimitsMax);
  return request;
}

KinematicsBackend::IKRequest makeCurrentStateIkRequest(const KinematicsBackend::VectorJ &joints) {
  KinematicsBackend::IKRequest request;
  request.seed_joints = joints;
  request.joint_limits_min = KinematicsBackend::VectorJ(kJointLimitsMin.begin(), kJointLimitsMin.end());
  request.joint_limits_max = KinematicsBackend::VectorJ(kJointLimitsMax.begin(), kJointLimitsMax.end());
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

double wrapToPi(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

struct BranchDescriptor {
  int shoulder_sign = 1;
  int elbow_sign = 1;
  bool wrist_flip = false;
  const char *id = "SL_EU_WN";
};

constexpr std::array<BranchDescriptor, 8> kCanonicalBranches{{
    {1, 1, false, "SL_EU_WN"},
    {1, 1, true, "SL_EU_WF"},
    {1, -1, false, "SL_ED_WN"},
    {1, -1, true, "SL_ED_WF"},
    {-1, 1, false, "SR_EU_WN"},
    {-1, 1, true, "SR_EU_WF"},
    {-1, -1, false, "SR_ED_WN"},
    {-1, -1, true, "SR_ED_WF"},
}};

std::string classifyBranchId(const KinematicsBackend::VectorJ &joints) {
  if (joints.size() < 6) {
    return "unknown";
  }
  const char shoulder = joints[0] >= 0.0 ? 'L' : 'R';
  const char elbow = joints[2] >= 0.0 ? 'U' : 'D';
  const char wrist = std::fabs(wrapToPi(joints[5])) > (M_PI / 2.0) ? 'F' : 'N';
  std::string id = "S";
  id.push_back(shoulder);
  id += "_E";
  id.push_back(elbow);
  id += "_W";
  id.push_back(wrist);
  return id;
}

double branchSwitchPenalty(const std::string &lhs,
                           const std::string &rhs,
                           const IKBackendConfig &config) {
  if (lhs.empty() || rhs.empty() || lhs == rhs) {
    return 0.0;
  }
  double penalty = 0.0;
  if (lhs.size() >= 8 && rhs.size() >= 8) {
    if (lhs[1] != rhs[1]) {
      penalty += config.shoulder_elbow_branch_switch_penalty;
    }
    if (lhs[4] != rhs[4]) {
      penalty += config.shoulder_elbow_branch_switch_penalty;
    }
    if (lhs[7] != rhs[7]) {
      penalty += config.wrist_flip_branch_switch_penalty;
    }
  }
  return penalty;
}

KinematicsBackend::VectorJ clampSeedToLimits(const KinematicsBackend::SeededIkRequest &request,
                                             KinematicsBackend::VectorJ seed) {
  if (seed.size() < 6 || request.joint_limits_min.size() < 6 || request.joint_limits_max.size() < 6) {
    return seed;
  }
  for (std::size_t index = 0; index < 6; ++index) {
    seed[index] = std::clamp(seed[index], request.joint_limits_min[index], request.joint_limits_max[index]);
  }
  return seed;
}

KinematicsBackend::VectorJ makeAnalyticBranchSeed(const KinematicsBackend::SeededIkRequest &request,
                                                  const BranchDescriptor &branch) {
  KinematicsBackend::VectorJ seed =
      request.seed_joints.size() >= 6 ? request.seed_joints : KinematicsBackend::VectorJ(6, 0.0);
  if (request.joint_limits_min.size() < 6 || request.joint_limits_max.size() < 6) {
    return seed;
  }

  const Eigen::Vector3d target_position = request.target_transform.block<3, 1>(0, 3);
  const double planar = std::hypot(target_position.x(), target_position.y());
  const double base_yaw = std::atan2(target_position.y(), target_position.x());
  const double reach = std::hypot(planar, target_position.z() - kDhD[0]);
  const double shoulder_offset = 0.35 + 0.25 * std::clamp(planar, 0.0, 1.0);
  const double elbow_mag = std::clamp(0.8 + 0.9 * std::min(reach, 0.8), 0.7, 1.75);
  const double shoulder_pitch = std::atan2(target_position.z() - kDhD[0], std::max(planar, 1e-6));

  seed[0] = wrapToPi(base_yaw + static_cast<double>(branch.shoulder_sign) * shoulder_offset);
  seed[1] = shoulder_pitch - 0.35 * static_cast<double>(branch.elbow_sign) * elbow_mag;
  seed[2] = static_cast<double>(branch.elbow_sign) * elbow_mag;
  seed[3] = -seed[1] - 0.45 * seed[2];
  seed[4] = branch.wrist_flip ? -0.35 : 0.35;
  if (branch.elbow_sign < 0) {
    seed[4] = -seed[4];
  }
  const double wrist_base = request.seed_joints.size() >= 6 ? request.seed_joints[5] : 0.0;
  seed[5] = wrapToPi(wrist_base + (branch.wrist_flip ? M_PI : 0.0));
  return clampSeedToLimits(request, std::move(seed));
}

std::vector<std::pair<std::string, KinematicsBackend::VectorJ>> makeAnalyticBranchSeeds(
    const KinematicsBackend::SeededIkRequest &request) {
  std::vector<std::pair<std::string, KinematicsBackend::VectorJ>> seeds;
  if (request.seed_joints.size() < 6 || request.joint_limits_min.size() < 6 || request.joint_limits_max.size() < 6) {
    return seeds;
  }
  seeds.reserve(kCanonicalBranches.size());
  for (const auto &branch : kCanonicalBranches) {
    seeds.emplace_back(branch.id, makeAnalyticBranchSeed(request, branch));
  }
  return seeds;
}

struct SeededIkAnalysis {
  Matrix6d jacobian = Matrix6d::Zero();
  double min_sigma = 0.0;
  bool near_singularity = true;
  double singularity_measure = 1.0;
};

SeededIkAnalysis analyzeSeededIk(const KinematicsBackend &backend,
                                 const KinematicsBackend::SeededIkRequest &,
                                 const std::vector<double> &joints) {
  const auto &config = backendConfig();
  SeededIkAnalysis analysis;
  if (joints.size() < 6) {
    return analysis;
  }
  const double j5 = std::fabs(joints[4]);
  double wrist_measure = 0.0;
  if (j5 < config.wrist_singularity_soft_threshold) {
    wrist_measure = 1.0 - (j5 / std::max(config.wrist_singularity_soft_threshold, 1e-6));
  }

  analysis.jacobian = backend.computeJacobian(joints);
  Eigen::JacobiSVD<Matrix6d> svd(analysis.jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (svd.singularValues().size() == 0) {
    return analysis;
  }

  analysis.min_sigma = svd.singularValues().tail<1>()(0);
  const double jacobian_measure = std::clamp(
      1.0 - (analysis.min_sigma / std::max(config.jacobian_singularity_soft_threshold, 1e-6)), 0.0, 1.0);
  analysis.singularity_measure = std::max(wrist_measure, jacobian_measure);
  analysis.near_singularity = std::fabs(joints[4]) < config.wrist_singularity_soft_threshold ||
                              analysis.min_sigma < config.jacobian_singularity_soft_threshold;
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
  return (Eigen::Map<const Eigen::VectorXd>(candidate.data(), 6) -
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
  metrics.joint_limit_penalty = jointLimitPenalty(candidate, request.joint_limits_min, request.joint_limits_max);
  metrics.singularity_metric = singularity.singularity_measure;
  metrics.singularity_cost = singularity.singularity_measure;
  metrics.near_singularity = singularity.near_singularity;
  metrics.valid = true;
  return metrics;
}

bool nudgeAwayFromSingularity(const KinematicsBackend::SeededIkRequest &request, std::vector<double> &joints);

std::vector<double> solveSeededIk(const KinematicsBackend &backend,
                                  const KinematicsBackend::SeededIkRequest &request);

KinematicsBackend::CartesianIkOptions optionsFromRequest(const KinematicsBackend::SeededIkRequest &request) {
  KinematicsBackend::CartesianIkOptions options;
  options.avoid_singularity = true;
  options.joint_limits_min = request.joint_limits_min;
  options.joint_limits_max = request.joint_limits_max;
  return options;
}

KinematicsBackend::IKCandidate scoreCartesianCandidate(const KinematicsBackend &backend,
                                                       const KinematicsBackend::SeededIkRequest &request,
                                                       const KinematicsBackend::CartesianIkOptions &options,
                                                       const IKBackendConfig &config,
                                                       const KinematicsBackend::VectorJ &candidate) {
  KinematicsBackend::IKCandidate scored;
  scored.q = candidate;
  scored.branch_id = classifyBranchId(candidate);
  if (candidate.size() < 6) {
    return scored;
  }
  if (options.soft_limit_enabled && violatesSoftLimit(candidate, options.soft_limits)) {
    return scored;
  }

  const auto conf_score = scoreRequestedConf(candidate, options.requested_conf);
  if (conf_score.has_request && options.strict_conf && !conf_score.strict_match) {
    return scored;
  }

  const auto metrics = backend.evaluateSeededIkCandidate(request, candidate);
  if (!metrics.valid) {
    return scored;
  }

  const auto singularity = analyzeSeededIk(backend, request, candidate);
  if (options.avoid_singularity &&
      (singularity.min_sigma < config.singularity_hard_reject ||
       std::fabs(candidate[4]) < config.wrist_singularity_hard_threshold ||
       singularity.min_sigma < config.jacobian_singularity_hard_threshold)) {
    return scored;
  }

  const auto fk_transform = backend.computeForwardTransform(candidate);
  scored.pose_error = cartesianErrorScore(request.target_transform, fk_transform);
  scored.continuity_cost = metrics.continuity_cost;
  scored.singularity_cost = metrics.singularity_cost;
  scored.limit_cost = metrics.joint_limit_penalty;
  scored.preference_cost = conf_score.penalty;
  scored.total_cost = config.weight_pose * scored.pose_error +
                      config.weight_continuity * metrics.branch_distance +
                      config.weight_singularity * scored.singularity_cost +
                      config.weight_limit * scored.limit_cost +
                      config.weight_preference * scored.preference_cost;
  scored.valid = true;
  return scored;
}

double transitionCost(const KinematicsBackend &backend,
                      const KinematicsBackend::SeededIkRequest &request,
                      const KinematicsBackend::IKCandidate &previous,
                      const KinematicsBackend::IKCandidate &candidate,
                      const IKBackendConfig &config) {
  auto continuity_request = request;
  continuity_request.seed_joints = previous.q;
  const double branch_distance = backend.branchDistance(previous.q, candidate.q);
  double cost = config.weight_continuity * backend.continuityCost(continuity_request, candidate.q);
  if (branch_distance > config.joint_branch_jump_threshold) {
    cost += config.weight_continuity * (branch_distance - config.joint_branch_jump_threshold);
  }
  cost += branchSwitchPenalty(previous.branch_id, candidate.branch_id, config);
  return cost;
}

bool dedupeCandidate(const KinematicsBackend::IKCandidate &candidate,
                     const std::vector<KinematicsBackend::IKCandidate> &accepted) {
  constexpr double kDuplicateThreshold = 0.05;
  for (const auto &existing : accepted) {
    if (branchDistance(candidate.q, existing.q) < kDuplicateThreshold) {
      return true;
    }
  }
  return false;
}

std::vector<KinematicsBackend::IKCandidate> generateScoredMultiBranchCandidates(
    const KinematicsBackend &backend,
    const KinematicsBackend::SeededIkRequest &request,
    const KinematicsBackend::CartesianIkOptions &options,
    const std::vector<KinematicsBackend::IKCandidate> *previous_point_candidates,
    std::size_t max_candidates) {
  const auto &config = backendConfig();
  std::vector<KinematicsBackend::IKCandidate> accepted;
  accepted.reserve(std::min<std::size_t>(kCanonicalBranches.size(), max_candidates));

  auto seed_request = request;
  auto seed_solution = solveSeededIk(backend, seed_request);
  if (seed_solution.empty() && options.avoid_singularity) {
    auto nudged_seed = seed_request.seed_joints;
    if (nudgeAwayFromSingularity(seed_request, nudged_seed)) {
      seed_request.seed_joints = nudged_seed;
      seed_solution = solveSeededIk(backend, seed_request);
    }
  }
  if (!seed_solution.empty()) {
    auto seed_candidate = scoreCartesianCandidate(backend, seed_request, options, config, seed_solution);
    if (seed_candidate.valid) {
      accepted.push_back(std::move(seed_candidate));
    }
  }

  for (const auto &[analytic_branch_id, analytic_seed] : makeAnalyticBranchSeeds(request)) {
    std::vector<KinematicsBackend::VectorJ> trial_seeds;
    trial_seeds.push_back(analytic_seed);
    if (previous_point_candidates != nullptr) {
      const auto previous_it = std::find_if(
          previous_point_candidates->begin(), previous_point_candidates->end(),
          [&](const KinematicsBackend::IKCandidate &candidate) { return candidate.branch_id == analytic_branch_id; });
      if (previous_it != previous_point_candidates->end()) {
        trial_seeds.insert(trial_seeds.begin(), previous_it->q);
      }
    }

    KinematicsBackend::IKCandidate best_for_branch;
    for (auto trial_seed : trial_seeds) {
      auto local_request = request;
      local_request.seed_joints = clampSeedToLimits(request, std::move(trial_seed));
      auto solved = solveSeededIk(backend, local_request);
      if (solved.empty() && options.avoid_singularity) {
        auto nudged_seed = local_request.seed_joints;
        if (nudgeAwayFromSingularity(local_request, nudged_seed)) {
          local_request.seed_joints = nudged_seed;
          solved = solveSeededIk(backend, local_request);
        }
      }
      if (solved.empty()) {
        continue;
      }
      auto candidate = scoreCartesianCandidate(backend, local_request, options, config, solved);
      if (!candidate.valid) {
        continue;
      }
      if (candidate.branch_id.empty() || candidate.branch_id == "unknown") {
        candidate.branch_id = analytic_branch_id;
      }
      if (!best_for_branch.valid || candidate.total_cost < best_for_branch.total_cost) {
        best_for_branch = std::move(candidate);
      }
    }

    if (!best_for_branch.valid || dedupeCandidate(best_for_branch, accepted)) {
      continue;
    }
    accepted.push_back(std::move(best_for_branch));
  }

  std::sort(accepted.begin(), accepted.end(), [](const auto &lhs, const auto &rhs) {
    if (lhs.total_cost == rhs.total_cost) {
      return lhs.continuity_cost < rhs.continuity_cost;
    }
    return lhs.total_cost < rhs.total_cost;
  });
  const auto capped_max = std::min(max_candidates, config.max_candidates_per_point);
  if (accepted.size() > capped_max) {
    accepted.resize(capped_max);
  }
  return accepted;
}

KinematicsBackend::IKTrajectoryResult solveTrajectoryIk(const KinematicsBackend &backend,
                                                        const KinematicsBackend::IKTrajectoryRequest &request) {
  KinematicsBackend::IKTrajectoryResult result;
  if (request.target_transforms.empty()) {
    result.message = "trajectory IK request is empty";
    return result;
  }
  if (request.initial_seed.size() < 6) {
    result.message = "trajectory IK seed is incomplete";
    return result;
  }

  const auto &config = backendConfig();
  std::vector<std::vector<KinematicsBackend::IKCandidate>> candidate_layers;
  candidate_layers.reserve(request.target_transforms.size());

  for (std::size_t point_index = 0; point_index < request.target_transforms.size(); ++point_index) {
    auto point_request = makeCartesianSeededIkRequest(
        request.target_transforms[point_index],
        point_index == 0 ? request.initial_seed : candidate_layers.back().front().q,
        request.options);
    const auto *previous_candidates = candidate_layers.empty() ? nullptr : &candidate_layers.back();
    auto point_candidates = generateScoredMultiBranchCandidates(
        backend,
        point_request,
        request.options,
        previous_candidates,
        std::max<std::size_t>(1, config.max_candidates_per_point));
    if (point_candidates.empty()) {
      result.failed_index = point_index;
      result.message = "no valid IK candidate at cartesian point " + std::to_string(point_index);
      return result;
    }
    candidate_layers.push_back(std::move(point_candidates));
  }

  std::vector<std::vector<double>> dp_costs(candidate_layers.size());
  std::vector<std::vector<int>> predecessor(candidate_layers.size());
  for (std::size_t point_index = 0; point_index < candidate_layers.size(); ++point_index) {
    dp_costs[point_index].assign(candidate_layers[point_index].size(), std::numeric_limits<double>::infinity());
    predecessor[point_index].assign(candidate_layers[point_index].size(), -1);
  }

  auto first_request = makeCartesianSeededIkRequest(
      request.target_transforms.front(), request.initial_seed, request.options);
  for (std::size_t candidate_index = 0; candidate_index < candidate_layers.front().size(); ++candidate_index) {
    auto continuity_request = first_request;
    continuity_request.seed_joints = request.initial_seed;
    dp_costs.front()[candidate_index] =
        candidate_layers.front()[candidate_index].total_cost +
        config.weight_continuity * backend.continuityCost(continuity_request, candidate_layers.front()[candidate_index].q);
  }

  for (std::size_t point_index = 1; point_index < candidate_layers.size(); ++point_index) {
    auto point_request = makeCartesianSeededIkRequest(
        request.target_transforms[point_index], request.initial_seed, request.options);
    for (std::size_t candidate_index = 0; candidate_index < candidate_layers[point_index].size(); ++candidate_index) {
      const auto &candidate = candidate_layers[point_index][candidate_index];
      for (std::size_t previous_index = 0; previous_index < candidate_layers[point_index - 1].size(); ++previous_index) {
        const auto transition = transitionCost(
            backend, point_request, candidate_layers[point_index - 1][previous_index], candidate, config);
        const double total_cost =
            dp_costs[point_index - 1][previous_index] + candidate.total_cost + transition;
        if (total_cost < dp_costs[point_index][candidate_index]) {
          dp_costs[point_index][candidate_index] = total_cost;
          predecessor[point_index][candidate_index] = static_cast<int>(previous_index);
        }
      }
    }
  }

  std::size_t best_terminal_index = 0;
  double best_terminal_cost = std::numeric_limits<double>::infinity();
  const auto &terminal_costs = dp_costs.back();
  for (std::size_t candidate_index = 0; candidate_index < terminal_costs.size(); ++candidate_index) {
    if (terminal_costs[candidate_index] < best_terminal_cost) {
      best_terminal_cost = terminal_costs[candidate_index];
      best_terminal_index = candidate_index;
    }
  }
  if (!std::isfinite(best_terminal_cost)) {
    result.failed_index = candidate_layers.size() - 1;
    result.message = "trajectory IK dynamic programming failed";
    return result;
  }

  result.q_path.resize(candidate_layers.size());
  result.singularity_metric_path.resize(candidate_layers.size(), 1.0);
  result.chosen_branch_path.resize(candidate_layers.size());
  int current_index = static_cast<int>(best_terminal_index);
  for (int point_index = static_cast<int>(candidate_layers.size()) - 1; point_index >= 0; --point_index) {
    if (current_index < 0 ||
        current_index >= static_cast<int>(candidate_layers[static_cast<std::size_t>(point_index)].size())) {
      result.failed_index = static_cast<std::size_t>(point_index);
      result.message = "trajectory IK backtracking failed";
      result.q_path.clear();
      result.singularity_metric_path.clear();
      result.chosen_branch_path.clear();
      return result;
    }
    const auto &candidate = candidate_layers[static_cast<std::size_t>(point_index)][static_cast<std::size_t>(current_index)];
    result.q_path[static_cast<std::size_t>(point_index)] = candidate.q;
    result.singularity_metric_path[static_cast<std::size_t>(point_index)] =
        evaluateSeededIkCandidate(backend, makeCartesianSeededIkRequest(
            request.target_transforms[static_cast<std::size_t>(point_index)], candidate.q, request.options),
            candidate.q).singularity_metric;
    result.chosen_branch_path[static_cast<std::size_t>(point_index)] = candidate.branch_id;
    current_index = predecessor[static_cast<std::size_t>(point_index)][static_cast<std::size_t>(current_index)];
  }

  result.success = true;
  result.message = "trajectory IK solved";
  return result;
}

KinematicsBackend::CartesianIkSelectionResult selectBestCartesianIkSolution(
    const KinematicsBackend &backend,
    const std::vector<KinematicsBackend::VectorJ> &candidates,
    const Matrix4d &target_transform,
    const KinematicsBackend::VectorJ &seed_joints,
    const KinematicsBackend::CartesianIkOptions &options) {
  KinematicsBackend::CartesianIkSelectionResult best;
  const auto request = makeCartesianSeededIkRequest(target_transform, seed_joints, options);
  const auto &config = backendConfig();
  std::vector<KinematicsBackend::IKCandidate> scored_candidates;
  scored_candidates.reserve(candidates.size());
  for (const auto &candidate : candidates) {
    auto scored = scoreCartesianCandidate(backend, request, options, config, candidate);
    if (scored.valid) {
      scored_candidates.push_back(std::move(scored));
    }
  }
  if (scored_candidates.empty()) {
    best.message = (!options.requested_conf.empty() && options.strict_conf) ? "no IK solution matches requested confData"
                                                                            : "no valid IK solution";
    return best;
  }
  std::sort(scored_candidates.begin(), scored_candidates.end(), [](const auto &lhs, const auto &rhs) {
    return lhs.total_cost < rhs.total_cost;
  });
  best.success = true;
  best.q = scored_candidates.front().q;
  best.branch_id = scored_candidates.front().branch_id;
  if (!options.strict_conf && !options.requested_conf.empty() &&
      scored_candidates.front().preference_cost > 1e-6) {
    best.note = "confData relaxed fallback";
  }
  best.message.clear();
  return best;
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
  if (cartesian_transforms.empty()) {
    error_message = "cartesian IK path is empty";
    return false;
  }

  KinematicsBackend::IKTrajectoryRequest trajectory_request;
  trajectory_request.target_transforms = cartesian_transforms;
  trajectory_request.initial_seed = initial_seed;
  trajectory_request.options = options;
  const auto solved = backend.solveTrajectory(trajectory_request);
  if (!solved.success || solved.q_path.empty()) {
    error_message = solved.message.empty() ? "cartesian trajectory IK solve failed" : solved.message;
    return false;
  }
  joint_trajectory = solved.q_path;
  last_joints = solved.q_path.back();
  return true;
}

Vector6d cartesianTwistFromTransforms(const Matrix4d &previous_transform,
                                      const Matrix4d &next_transform,
                                      double dt) {
  Vector6d twist = Vector6d::Zero();
  if (dt <= 1e-9) {
    return twist;
  }

  twist.head<3>() =
      (next_transform.block<3, 1>(0, 3) - previous_transform.block<3, 1>(0, 3)) / dt;

  const Eigen::Matrix3d rotation_delta =
      next_transform.block<3, 3>(0, 0) * previous_transform.block<3, 3>(0, 0).transpose();
  const Eigen::AngleAxisd angle_axis(rotation_delta);
  Eigen::Vector3d angular_delta = angle_axis.angle() * angle_axis.axis();
  if (!std::isfinite(angular_delta.norm()) || angle_axis.angle() < 1e-8) {
    angular_delta << rotation_delta(2, 1) - rotation_delta(1, 2),
        rotation_delta(0, 2) - rotation_delta(2, 0),
        rotation_delta(1, 0) - rotation_delta(0, 1);
    angular_delta *= 0.5;
  }
  twist.tail<3>() = angular_delta / dt;
  return twist;
}

bool isFiniteVector(const Vector6d &values) {
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    if (!std::isfinite(values(index))) {
      return false;
    }
  }
  return true;
}

bool projectCartesianJointDerivatives(const KinematicsBackend &backend,
                                      const std::vector<Matrix4d> &cartesian_transforms,
                                      const std::vector<KinematicsBackend::VectorJ> &joint_trajectory,
                                      double trajectory_dt,
                                      std::vector<KinematicsBackend::VectorJ> &joint_velocity_trajectory,
                                      std::vector<KinematicsBackend::VectorJ> &joint_acceleration_trajectory) {
  joint_velocity_trajectory.clear();
  joint_acceleration_trajectory.clear();

  if (cartesian_transforms.size() != joint_trajectory.size() || joint_trajectory.empty() || trajectory_dt <= 1e-9) {
    return false;
  }

  const std::size_t point_count = joint_trajectory.size();
  const std::size_t axis_count = joint_trajectory.front().size();
  joint_velocity_trajectory.assign(point_count, KinematicsBackend::VectorJ(axis_count, 0.0));
  joint_acceleration_trajectory.assign(point_count, KinematicsBackend::VectorJ(axis_count, 0.0));
  if (point_count == 1 || axis_count == 0) {
    return true;
  }

  for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
    const Matrix4d *previous_transform = nullptr;
    const Matrix4d *next_transform = nullptr;
    double twist_dt = trajectory_dt;
    if (point_index == 0) {
      previous_transform = &cartesian_transforms[0];
      next_transform = &cartesian_transforms[1];
    } else if (point_index + 1 >= point_count) {
      previous_transform = &cartesian_transforms[point_index - 1];
      next_transform = &cartesian_transforms[point_index];
    } else {
      previous_transform = &cartesian_transforms[point_index - 1];
      next_transform = &cartesian_transforms[point_index + 1];
      twist_dt = 2.0 * trajectory_dt;
    }

    const auto twist = cartesianTwistFromTransforms(*previous_transform, *next_transform, twist_dt);
    const auto jacobian = backend.computeJacobian(joint_trajectory[point_index]);
    const auto request = makeCartesianSeededIkRequest(
        *next_transform, joint_trajectory[point_index], KinematicsBackend::CartesianIkOptions{});
    const double singularity = std::clamp(
        backend.singularityMetric(request, joint_trajectory[point_index]), 0.0, 1.0);
    const double damping = 1e-4 + 0.05 * singularity;
    const Matrix6d jj_t =
        jacobian * jacobian.transpose() + damping * damping * Matrix6d::Identity();
    const Vector6d qd = jacobian.transpose() * jj_t.ldlt().solve(twist);
    if (!isFiniteVector(qd)) {
      joint_velocity_trajectory.clear();
      joint_acceleration_trajectory.clear();
      return false;
    }
    for (std::size_t axis = 0; axis < axis_count && axis < static_cast<std::size_t>(qd.size()); ++axis) {
      joint_velocity_trajectory[point_index][axis] = qd(static_cast<Eigen::Index>(axis));
    }
  }

  for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
    for (std::size_t axis = 0; axis < axis_count; ++axis) {
      if (point_index == 0) {
        joint_acceleration_trajectory[point_index][axis] =
            (joint_velocity_trajectory[1][axis] - joint_velocity_trajectory[0][axis]) / trajectory_dt;
      } else if (point_index + 1 >= point_count) {
        joint_acceleration_trajectory[point_index][axis] =
            (joint_velocity_trajectory[point_index][axis] - joint_velocity_trajectory[point_index - 1][axis]) /
            trajectory_dt;
      } else {
        joint_acceleration_trajectory[point_index][axis] =
            (joint_velocity_trajectory[point_index + 1][axis] - joint_velocity_trajectory[point_index - 1][axis]) /
            (2.0 * trajectory_dt);
      }
    }
  }

  return true;
}

bool nudgeAwayFromSingularity(const KinematicsBackend::SeededIkRequest &request, std::vector<double> &joints) {
  const auto &config = backendConfig();
  if (joints.size() < 6 || request.joint_limits_min.size() < 6 || request.joint_limits_max.size() < 6) {
    return false;
  }

  const double current_j5 = joints[4];
  const double target_offset = config.singularity_avoidance_offset;
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
  const auto &config = backendConfig();
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

  for (int iter = 0; iter < config.seed_solver_max_iter; ++iter) {
    const Matrix4d current_transform = backend.computeForwardTransform(joints);
    const Vector6d error = poseError(request.target_transform, current_transform);

    const auto singularity = analyzeSeededIk(backend, request, joints);
    const auto candidate_metrics = evaluateSeededIkCandidate(backend, request, joints);
    const double pos_err = error.head<3>().norm();
    const double ori_err = error.tail<3>().norm();
    const double current_score =
        pos_err + config.seed_solver_orientation_weight * ori_err +
        config.seed_solver_continuity_weight * candidate_metrics.continuity_cost +
        config.seed_solver_limit_weight * candidate_metrics.joint_limit_penalty +
        config.seed_solver_singularity_weight * candidate_metrics.singularity_cost;
    if (current_score < best_score) {
      best_score = current_score;
      best_joints = joints;
    }
    if (pos_err < config.seed_solver_position_tolerance &&
        ori_err < config.seed_solver_orientation_tolerance) {
      break;
    }

    Matrix6d weight = Matrix6d::Identity();
    weight(3, 3) = weight(4, 4) = weight(5, 5) = config.seed_solver_orientation_weight;
    if (singularity.near_singularity) {
      weight(3, 3) *= 0.3;
      weight(4, 4) *= 0.3;
      weight(5, 5) *= 0.3;
    }

    Vector6d weighted_error = error;
    weighted_error.tail<3>() *= weight(3, 3);
    const Matrix6d weighted_jacobian = weight * singularity.jacobian;
    const double lambda_base =
        config.seed_solver_min_lambda +
        config.seed_solver_max_lambda *
            (1.0 - static_cast<double>(iter) / std::max(config.seed_solver_max_iter, 1));
    const double lambda = lambda_base * (1.0 + 2.0 * singularity.singularity_measure);
    const Matrix6d jj_t = weighted_jacobian * weighted_jacobian.transpose();
    Vector6d delta_q =
        weighted_jacobian.transpose() * (jj_t + lambda * lambda * Matrix6d::Identity()).inverse() * weighted_error;

    const double step_scale = singularity.near_singularity ? 0.6 : 1.0;
    for (int index = 0; index < 6; ++index) {
      double step = delta_q(index);
      step = std::clamp(
          step,
          -config.seed_solver_max_joint_step * step_scale,
          config.seed_solver_max_joint_step * step_scale);
      joints[index] += step;
      joints[index] = std::clamp(joints[index], request.joint_limits_min[index], request.joint_limits_max[index]);
    }
  }

  const Matrix4d best_transform = backend.computeForwardTransform(best_joints);
  const Vector6d best_error = poseError(request.target_transform, best_transform);
  if (best_error.head<3>().norm() > config.seed_solver_solution_valid_threshold ||
      best_error.tail<3>().norm() > config.seed_solver_solution_valid_threshold) {
    return {};
  }
  return best_joints;
}

KinematicsBackend::IKResult solveSeededIkResult(const KinematicsBackend &backend,
                                                const KinematicsBackend::IKRequest &request) {
  KinematicsBackend::IKResult result;
  const auto solved = solveSeededIk(backend, request);
  if (solved.empty()) {
    result.message = "seeded IK solve failed";
    return result;
  }

  const auto scored = scoreCartesianCandidate(
      backend, request, optionsFromRequest(request), backendConfig(), solved);
  result.success = scored.valid;
  result.q = solved;
  result.candidates = {scored};
  result.chosen_branch = scored.branch_id;
  result.singularity_metric = evaluateSeededIkCandidate(backend, request, solved).singularity_metric;
  result.message = scored.valid ? std::string{} : "seeded IK candidate rejected";
  return result;
}

KinematicsBackend::IKResult solveMultiBranchIkResult(const KinematicsBackend &backend,
                                                     const KinematicsBackend::IKRequest &request) {
  KinematicsBackend::IKResult result;
  auto candidates = generateScoredMultiBranchCandidates(
      backend, request, optionsFromRequest(request), nullptr, kCanonicalBranches.size());
  if (candidates.empty()) {
    result.message = "no valid IK branch candidates";
    return result;
  }
  result.success = true;
  result.q = candidates.front().q;
  result.chosen_branch = candidates.front().branch_id;
  result.singularity_metric = evaluateSeededIkCandidate(backend, request, candidates.front().q).singularity_metric;
  result.candidates = std::move(candidates);
  return result;
}

std::vector<KinematicsBackend::VectorJ> solveMultiBranchIk(
    const KinematicsBackend &backend,
    const KinematicsBackend::SeededIkRequest &request) {
  const auto multi_result = solveMultiBranchIkResult(backend, request);
  std::vector<KinematicsBackend::VectorJ> candidates;
  candidates.reserve(multi_result.candidates.size());
  for (const auto &candidate : multi_result.candidates) {
    candidates.push_back(candidate.q);
  }
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

  [[nodiscard]] bool avoidSingularity(VectorJ &joints) const override {
    auto request = makeCurrentStateIkRequest(joints);
    return nudgeAwayFromSingularity(request, joints);
  }

  [[nodiscard]] IKResult solveSeeded(const IKRequest &request) const override {
    return solveSeededIkResult(*this, request);
  }

  [[nodiscard]] IKResult solveMultiBranch(const IKRequest &request) const override {
    return solveMultiBranchIkResult(*this, request);
  }

  [[nodiscard]] IKTrajectoryResult solveTrajectory(const IKTrajectoryRequest &request) const override {
    return solveTrajectoryIk(*this, request);
  }

  [[nodiscard]] VectorJ inverseKinematicsSeeded(const SeededIkRequest &request) const override {
    return solveSeeded(request).q;
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

  [[nodiscard]] bool projectCartesianJointDerivatives(
      const std::vector<Matrix4d> &cartesian_transforms,
      const std::vector<VectorJ> &joint_trajectory,
      double trajectory_dt,
      std::vector<VectorJ> &joint_velocity_trajectory,
      std::vector<VectorJ> &joint_acceleration_trajectory) const override {
    return gazebo::detail::projectCartesianJointDerivatives(
        *this, cartesian_transforms, joint_trajectory, trajectory_dt, joint_velocity_trajectory,
        joint_acceleration_trajectory);
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

  [[nodiscard]] bool avoidSingularity(VectorJ &joints) const override {
    if (!valid_) {
      return LegacyKinematicsBackend{}.avoidSingularity(joints);
    }
    auto request = makeCurrentStateIkRequest(joints);
    return nudgeAwayFromSingularity(request, joints);
  }

  [[nodiscard]] IKResult solveSeeded(const IKRequest &request) const override {
    if (!valid_) {
      return LegacyKinematicsBackend{}.solveSeeded(request);
    }
    return solveSeededIkResult(*this, request);
  }

  [[nodiscard]] IKResult solveMultiBranch(const IKRequest &request) const override {
    if (!valid_) {
      return LegacyKinematicsBackend{}.solveMultiBranch(request);
    }
    return solveMultiBranchIkResult(*this, request);
  }

  [[nodiscard]] IKTrajectoryResult solveTrajectory(const IKTrajectoryRequest &request) const override {
    if (!valid_) {
      return LegacyKinematicsBackend{}.solveTrajectory(request);
    }
    return solveTrajectoryIk(*this, request);
  }

  [[nodiscard]] VectorJ inverseKinematicsSeeded(const SeededIkRequest &request) const override {
    if (!valid_) {
      return LegacyKinematicsBackend{}.inverseKinematicsSeeded(request);
    }
    return solveSeeded(request).q;
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

  [[nodiscard]] bool projectCartesianJointDerivatives(
      const std::vector<Matrix4d> &cartesian_transforms,
      const std::vector<VectorJ> &joint_trajectory,
      double trajectory_dt,
      std::vector<VectorJ> &joint_velocity_trajectory,
      std::vector<VectorJ> &joint_acceleration_trajectory) const override {
    if (!valid_) {
      return LegacyKinematicsBackend{}.projectCartesianJointDerivatives(
          cartesian_transforms, joint_trajectory, trajectory_dt, joint_velocity_trajectory,
          joint_acceleration_trajectory);
    }
    return gazebo::detail::projectCartesianJointDerivatives(
        *this, cartesian_transforms, joint_trajectory, trajectory_dt, joint_velocity_trajectory,
        joint_acceleration_trajectory);
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
