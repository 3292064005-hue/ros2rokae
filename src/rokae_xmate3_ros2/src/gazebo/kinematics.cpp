/**
 * @file kinematics.cpp
 * @brief xMate3 运动学计算实现 (改进 DH 版 - 有限差分雅可比修复版)
 */

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "gazebo/kinematics_backend.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gazebo {
namespace {

detail::KinematicsBackend::SeededIkRequest makeSeededIkRequest(
    const Matrix4d &target_transform,
    const std::vector<double> &seed_joints,
    const std::vector<double> &joint_limits_min,
    const std::vector<double> &joint_limits_max,
    int max_iter,
    double position_tolerance,
    double orientation_tolerance,
    double orientation_weight,
    double max_joint_step,
    double min_lambda,
    double max_lambda,
    double solution_valid_threshold) {
    detail::KinematicsBackend::SeededIkRequest request;
    request.target_transform = target_transform;
    request.seed_joints = seed_joints;
    request.joint_limits_min = joint_limits_min;
    request.joint_limits_max = joint_limits_max;
    request.max_iter = max_iter;
    request.position_tolerance = position_tolerance;
    request.orientation_tolerance = orientation_tolerance;
    request.orientation_weight = orientation_weight;
    request.max_joint_step = max_joint_step;
    request.min_lambda = min_lambda;
    request.max_lambda = max_lambda;
    request.solution_valid_threshold = solution_valid_threshold;
    return request;
}

xMate3Kinematics::IkCandidateMetrics toPublicMetrics(
    const detail::KinematicsBackend::SeededIkCandidateMetrics &metrics) {
    xMate3Kinematics::IkCandidateMetrics result;
    result.branch_distance = metrics.branch_distance;
    result.continuity_cost = metrics.continuity_cost;
    result.joint_limit_penalty = metrics.joint_limit_penalty;
    result.singularity_metric = metrics.singularity_metric;
    result.singularity_cost = metrics.singularity_cost;
    result.near_singularity = metrics.near_singularity;
    result.valid = metrics.valid;
    return result;
}

}  // namespace

xMate3Kinematics::xMate3Kinematics() {
    // DH 参数 (单位 m) - 完全保留你的原版
    dh_a_ = {0.0, 0.0, 0.394, 0.0, 0.0, 0.0};
    dh_alpha_ = {0.0, -M_PI/2, 0.0, M_PI/2, -M_PI/2, M_PI/2};
    dh_d_ = {0.3415, 0.0, 0.0, 0.366, 0.0, 0.2503};

    // 关节限位 - 完全保留你的原版
    joint_limits_min_ = {-3.0527, -2.0933, -2.0933, -3.0527, -2.0933, -6.1082};
    joint_limits_max_ = {3.0527, 2.0933, 2.0933, 3.0527, 2.0933, 6.1082};
    backend_ = detail::makePreferredKinematicsBackend();
}

// ==================================================
// 核心辅助函数：100%保留你的初始偏移逻辑
// ==================================================
std::vector<Matrix4d> xMate3Kinematics::computeAllTransforms(const std::vector<double>& joints) {
    if (backend_) {
        return backend_->computeAllTransforms(joints);
    }
    std::vector<Matrix4d> T(7);
    T[0] = Matrix4d::Identity();
    std::vector<double> jnt = joints;
    jnt[1] -= M_PI / 2.0;
    jnt[2] += M_PI / 2.0;
    for (int i = 0; i < 6; ++i) {
        T[i + 1] = T[i] * dhTransform(i, jnt[i]);
    }
    return T;
}

// -------------------------- 正运动学 - 完全保留你的原版 --------------------------
Matrix4d xMate3Kinematics::forwardKinematics(const std::vector<double>& joints) {
    if (backend_) {
        return backend_->computeForwardTransform(joints);
    }
    std::vector<Matrix4d> T = computeAllTransforms(joints);
    return T[6];
}

// -------------------------- 正运动学RPY - 完全保留你的原版 --------------------------
std::vector<double> xMate3Kinematics::forwardKinematicsRPY(const std::vector<double>& joints) {
    Matrix4d T = forwardKinematics(joints);
    std::vector<double> pose(6);

    pose[0] = T(0,3);
    pose[1] = T(1,3);
    pose[2] = T(2,3);

    pose[4] = atan2(-T(2,0), sqrt(T(0,0)*T(0,0) + T(1,0)*T(1,0)));
    if (fabs(pose[4] - M_PI/2) < 1e-3) {
        pose[5] = 0;
        pose[3] = atan2(T(1,2), T(1,1));
    }
    else if (fabs(pose[4] + M_PI/2) < 1e-3) {
        pose[5] = 0;
        pose[3] = atan2(-T(1,2), T(1,1));
    }
    else {
        pose[5] = atan2(T(1,0), T(0,0));
        pose[3] = atan2(T(2,1), T(2,2));
    }

    auto normalizeAngle = [](double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };

    pose[3] = normalizeAngle(pose[3]);
    pose[4] = normalizeAngle(pose[4]);
    pose[5] = normalizeAngle(pose[5]);

    return pose;
}
struct xMate3Kinematics::SingularityAnalysis {
    Matrix6d jacobian = Matrix6d::Zero();
    JacobiSVD<Matrix6d> svd;
    double min_sigma = 0.0;
    bool near_singularity = true;
    double singularity_measure = 1.0;

    SingularityAnalysis();
};

xMate3Kinematics::SingularityAnalysis::SingularityAnalysis()
    : svd(Matrix6d::Identity(), ComputeFullU | ComputeFullV) {}

// -------------------------- 逆运动学 - 多解输出版（带奇异位处理） --------------------------
// 新增返回值：所有满足精度的有效逆解，按与当前关节的运动距离从小到大排序
std::vector<std::vector<double>> xMate3Kinematics::inverseKinematicsMultiSolution(
        const std::vector<double>& target,
        const std::vector<double>& current_joints) {
    Matrix4d T_target = rpyToTransform(target);

    if (backend_) {
        auto request = makeSeededIkRequest(
            T_target,
            current_joints,
            joint_limits_min_,
            joint_limits_max_,
            16,
            1e-5,
            1e-3,
            0.8,
            0.05,
            0.02,
            0.5,
            5e-3);
        const auto backend_solutions = backend_->inverseKinematicsMultiBranch(request);
        if (!backend_solutions.empty()) {
            return backend_solutions;
        }
    }

    const double orientation_weight = 0.8;
    const double duplicate_threshold = 0.05;

    auto solveFromSeed = [&](std::vector<double> joints) -> std::pair<std::vector<double>, double> {
        auto solved = inverseKinematicsSeededFast(target, joints);
        if (solved.empty()) {
            return {{}, std::numeric_limits<double>::infinity()};
        }

        Matrix4d T_final = forwardKinematics(solved);
        Vector6d final_error = computePoseError(T_target, T_final);
        const double score = final_error.head<3>().norm() + orientation_weight * final_error.tail<3>().norm();
        return {solved, score};
    };

    std::vector<std::vector<double>> seeds;
    {
        std::vector<double> seed = current_joints;
        if (analyzeSingularity(seed).near_singularity) {
            avoidSingularity(seed);
        }
        seeds.push_back(seed);
    }
    {
        std::vector<double> seed_shoulder_left = current_joints;
        seed_shoulder_left[0] = std::clamp(2.0, joint_limits_min_[0], joint_limits_max_[0]);
        seed_shoulder_left[4] = 0.3;
        seeds.push_back(seed_shoulder_left);
    }
    {
        std::vector<double> seed_shoulder_right = current_joints;
        seed_shoulder_right[0] = std::clamp(-2.0, joint_limits_min_[0], joint_limits_max_[0]);
        seed_shoulder_right[4] = -0.3;
        seeds.push_back(seed_shoulder_right);
    }
    {
        std::vector<double> seed_elbow_up = current_joints;
        seed_elbow_up[2] = std::clamp(1.5, joint_limits_min_[2], joint_limits_max_[2]);
        seed_elbow_up[4] = 0.4;
        seeds.push_back(seed_elbow_up);
    }
    {
        std::vector<double> seed_elbow_down = current_joints;
        seed_elbow_down[2] = std::clamp(-1.5, joint_limits_min_[2], joint_limits_max_[2]);
        seed_elbow_down[4] = -0.4;
        seeds.push_back(seed_elbow_down);
    }
    {
        std::vector<double> seed_sing_avoid1 = current_joints;
        seed_sing_avoid1[4] = 0.5;
        seeds.push_back(seed_sing_avoid1);
    }
    {
        std::vector<double> seed_sing_avoid2 = current_joints;
        seed_sing_avoid2[4] = -0.5;
        seeds.push_back(seed_sing_avoid2);
    }
    {
        std::vector<double> seed_wrist_flip = current_joints;
        seed_wrist_flip[5] = std::clamp(M_PI, joint_limits_min_[5], joint_limits_max_[5]);
        seed_wrist_flip[4] = 0.25;
        seeds.push_back(seed_wrist_flip);
    }
    {
        std::vector<double> seed_wrist_noflip = current_joints;
        seed_wrist_noflip[5] = std::clamp(0.0, joint_limits_min_[5], joint_limits_max_[5]);
        seed_wrist_noflip[4] = -0.25;
        seeds.push_back(seed_wrist_noflip);
    }
    std::vector<std::vector<double>> candidate_solutions;
    for (const auto& seed : seeds) {
        auto [candidate_joints, score] = solveFromSeed(seed);
        if (score < std::numeric_limits<double>::infinity()) {
            candidate_solutions.push_back(candidate_joints);
        }
    }

    std::vector<std::vector<double>> unique_solutions;
    for (const auto& sol : candidate_solutions) {
        bool is_duplicate = false;
        for (const auto& unique_sol : unique_solutions) {
            double diff_norm = 0.0;
            for (int i = 0; i < 6; ++i) {
                diff_norm += std::pow(sol[i] - unique_sol[i], 2);
            }
            diff_norm = std::sqrt(diff_norm);
            if (diff_norm < duplicate_threshold) {
                is_duplicate = true;
                break;
            }
        }
        if (!is_duplicate) {
            unique_solutions.push_back(sol);
        }
    }

    std::sort(unique_solutions.begin(), unique_solutions.end(),
        [&current_joints](const std::vector<double>& a, const std::vector<double>& b) {
            double dist_a = 0.0;
            double dist_b = 0.0;
            for (int i = 0; i < 6; ++i) {
                dist_a += std::fabs(a[i] - current_joints[i]);
                dist_b += std::fabs(b[i] - current_joints[i]);
            }
            return dist_a < dist_b;
        });

    return unique_solutions;
}

std::vector<double> xMate3Kinematics::inverseKinematics(
        const std::vector<double>& target,
        const std::vector<double>& current_joints) {
    const auto solutions = inverseKinematicsMultiSolution(target, current_joints);
    if (solutions.empty()) {
        return {};
    }
    return solutions.front();
}

std::vector<double> xMate3Kinematics::inverseKinematicsSeededFast(
        const std::vector<double>& target,
        const std::vector<double>& seed_joints) {
    if (target.size() < 6 || seed_joints.size() < 6) {
        return {};
    }

    Matrix4d T_target = rpyToTransform(target);
    const int max_iter = 16;
    const double position_tolerance = 1e-5;
    const double orientation_tolerance = 1e-3;
    const double orientation_weight = 0.8;
    const double max_joint_step = 0.05;
    const double min_lambda = 0.02;
    const double max_lambda = 0.5;
    const double solution_valid_threshold = 5e-3;

    if (backend_) {
        auto request = makeSeededIkRequest(
            T_target,
            seed_joints,
            joint_limits_min_,
            joint_limits_max_,
            max_iter,
            position_tolerance,
            orientation_tolerance,
            orientation_weight,
            max_joint_step,
            min_lambda,
            max_lambda,
            solution_valid_threshold);
        auto solved = backend_->inverseKinematicsSeeded(request);
        if (!solved.empty()) {
            return solved;
        }
    }

    std::vector<double> joints = seed_joints;
    if (analyzeSingularity(joints).near_singularity) {
        avoidSingularity(joints);
    }

    double best_score = std::numeric_limits<double>::infinity();
    std::vector<double> best_joints = joints;

    for (int iter = 0; iter < max_iter; ++iter) {
        Matrix4d T_current = forwardKinematics(joints);
        Vector6d error = computePoseError(T_target, T_current);

        const double pos_err = error.head<3>().norm();
        const double ori_err = error.tail<3>().norm();
        const double current_score = pos_err + orientation_weight * ori_err;
        if (current_score < best_score) {
            best_score = current_score;
            best_joints = joints;
        }
        if (pos_err < position_tolerance && ori_err < orientation_tolerance) {
            break;
        }

        const auto singularity = analyzeSingularity(joints);
        Matrix6d W = Matrix6d::Identity();
        W(3,3) = W(4,4) = W(5,5) = orientation_weight;
        if (singularity.near_singularity) {
            W(3,3) *= 0.3;
            W(4,4) *= 0.3;
            W(5,5) *= 0.3;
        }

        Vector6d err_w = error;
        err_w.tail<3>() *= W(3,3);
        const Matrix6d Jw = W * singularity.jacobian;
        const double lambda_base = min_lambda + max_lambda * (1.0 - static_cast<double>(iter) / max_iter);
        const double lambda = lambda_base * (1.0 + 2.0 * singularity.singularity_measure);
        const Matrix6d JJt = Jw * Jw.transpose();
        Vector6d delta_q =
            Jw.transpose() * (JJt + lambda * lambda * Matrix6d::Identity()).inverse() * err_w;

        const double step_scale = singularity.near_singularity ? 0.6 : 1.0;
        for (int i = 0; i < 6; ++i) {
            double step = delta_q(i);
            step = std::clamp(step, -max_joint_step * step_scale, max_joint_step * step_scale);
            joints[i] += step;
            joints[i] = std::clamp(joints[i], joint_limits_min_[i], joint_limits_max_[i]);
        }
    }

    Matrix4d T_final = forwardKinematics(joints);
    Vector6d final_error = computePoseError(T_target, T_final);
    double final_pos_err = final_error.head<3>().norm();
    double final_ori_err = final_error.tail<3>().norm();
    double final_score = final_pos_err + orientation_weight * final_ori_err;

    Matrix4d T_best = forwardKinematics(best_joints);
    Vector6d best_error = computePoseError(T_target, T_best);
    const double best_pos_err = best_error.head<3>().norm();
    const double best_ori_err = best_error.tail<3>().norm();
    const double best_final_score = best_pos_err + orientation_weight * best_ori_err;
    if (best_final_score < final_score) {
        joints = best_joints;
        final_pos_err = best_pos_err;
        final_ori_err = best_ori_err;
    }

    if (final_pos_err > solution_valid_threshold || final_ori_err > solution_valid_threshold) {
        return {};
    }

    return joints;
}

xMate3Kinematics::Matrix6d xMate3Kinematics::computeJacobian(const std::vector<double>& joints) {
    ++debug_counters_.jacobian_calls;
    if (backend_) {
        return backend_->computeJacobian(joints);
    }

    Matrix6d J = Matrix6d::Zero();
    const double delta = 1e-6;
    const Matrix4d T_current = forwardKinematics(joints);

    for (int i = 0; i < 6; ++i) {
        std::vector<double> joints_plus = joints;
        joints_plus[i] += delta;
        const Matrix4d T_plus = forwardKinematics(joints_plus);
        const Vector6d error = computePoseError(T_plus, T_current);
        J.col(i) = error / delta;
    }

    return J;
}

xMate3Kinematics::SingularityAnalysis xMate3Kinematics::analyzeSingularity(const std::vector<double>& joints) {
    SingularityAnalysis analysis;
    if (joints.size() < 6) {
        return analysis;
    }

    double wrist_measure = 0.0;
    const double j5 = std::fabs(joints[4]);
    if (j5 < WRIST_SINGULARITY_THRESHOLD) {
        wrist_measure = 1.0 - (j5 / WRIST_SINGULARITY_THRESHOLD);
    }

    analysis.jacobian = computeJacobian(joints);
    ++debug_counters_.svd_calls;
    analysis.svd = JacobiSVD<Matrix6d>(analysis.jacobian, ComputeFullU | ComputeFullV);
    if (analysis.svd.singularValues().size() == 0) {
        return analysis;
    }

    analysis.min_sigma = analysis.svd.singularValues().tail<1>()(0);
    const double jacobian_measure = std::clamp(
        1.0 - (analysis.min_sigma / JACOBIAN_SINGULARITY_THRESHOLD), 0.0, 1.0);
    analysis.singularity_measure = std::max(wrist_measure, jacobian_measure);
    analysis.near_singularity =
        std::fabs(joints[4]) < WRIST_SINGULARITY_THRESHOLD ||
        analysis.min_sigma < JACOBIAN_SINGULARITY_THRESHOLD;
    return analysis;
}

bool xMate3Kinematics::isNearSingularity(const std::vector<double>& joints) {
    return analyzeSingularity(joints).near_singularity;
}

double xMate3Kinematics::computeSingularityMeasure(const std::vector<double>& joints) {
    return analyzeSingularity(joints).singularity_measure;
}

bool xMate3Kinematics::avoidSingularity(std::vector<double>& joints) {
    const auto before = analyzeSingularity(joints);
    if (!before.near_singularity) {
        return true;
    }

    const double current_j5 = joints[4];
    const double target_offset = SINGULARITY_AVOIDANCE_OFFSET;
    joints[4] = current_j5 >= 0.0 ? std::max(target_offset, joint_limits_min_[4])
                                  : std::min(-target_offset, joint_limits_max_[4]);
    joints[4] = std::clamp(joints[4], joint_limits_min_[4], joint_limits_max_[4]);

    const auto after_wrist_adjust = analyzeSingularity(joints);
    if (after_wrist_adjust.singularity_measure > 0.7) {
        joints[2] = std::clamp(joints[2] + (joints[2] >= 0.0 ? 0.12 : -0.12),
                               joint_limits_min_[2], joint_limits_max_[2]);
        joints[3] = std::clamp(joints[3] + (joints[3] >= 0.0 ? 0.08 : -0.08),
                               joint_limits_min_[3], joint_limits_max_[3]);
        joints[5] = std::clamp(joints[5] + (joints[5] >= 0.0 ? 0.1 : -0.1),
                               joint_limits_min_[5], joint_limits_max_[5]);
    }

    return true;
}

double xMate3Kinematics::branchDistance(const std::vector<double>& lhs,
                                        const std::vector<double>& rhs) const {
    if (backend_) {
        return backend_->branchDistance(lhs, rhs);
    }
    double max_distance = 0.0;
    for (std::size_t index = 0; index < 6 && index < lhs.size() && index < rhs.size(); ++index) {
        max_distance = std::max(max_distance, std::fabs(lhs[index] - rhs[index]));
    }
    return max_distance;
}

xMate3Kinematics::IkCandidateMetrics xMate3Kinematics::evaluateIkCandidate(
        const std::vector<double>& candidate,
        const std::vector<double>& seed_joints) const {
    if (candidate.size() < 6 || seed_joints.size() < 6) {
        return {};
    }

    constexpr int max_iter = 16;
    constexpr double position_tolerance = 1e-5;
    constexpr double orientation_tolerance = 1e-3;
    constexpr double orientation_weight = 0.8;
    constexpr double max_joint_step = 0.05;
    constexpr double min_lambda = 0.02;
    constexpr double max_lambda = 0.5;
    constexpr double solution_valid_threshold = 5e-3;

    auto request = makeSeededIkRequest(
        Matrix4d::Identity(),
        seed_joints,
        joint_limits_min_,
        joint_limits_max_,
        max_iter,
        position_tolerance,
        orientation_tolerance,
        orientation_weight,
        max_joint_step,
        min_lambda,
        max_lambda,
        solution_valid_threshold);

    if (backend_) {
        return toPublicMetrics(backend_->evaluateSeededIkCandidate(request, candidate));
    }

    IkCandidateMetrics metrics;
    metrics.branch_distance = branchDistance(candidate, seed_joints);
    metrics.continuity_cost = request.continuity_weight *
        (Eigen::Map<const Eigen::VectorXd>(candidate.data(), 6) -
         Eigen::Map<const Eigen::VectorXd>(seed_joints.data(), 6))
            .norm();

    double limit_penalty = 0.0;
    for (std::size_t index = 0; index < 6; ++index) {
        const double lower_margin = candidate[index] - joint_limits_min_[index];
        const double upper_margin = joint_limits_max_[index] - candidate[index];
        const double range = std::max(joint_limits_max_[index] - joint_limits_min_[index], 1e-6);
        const double normalized_margin = std::clamp(std::min(lower_margin, upper_margin) / range, 0.0, 1.0);
        limit_penalty += (1.0 - normalized_margin);
    }
    metrics.joint_limit_penalty = request.joint_limit_weight * (limit_penalty / 6.0);
    metrics.singularity_metric = 0.0;
    metrics.singularity_cost = 0.0;
    metrics.near_singularity = false;
    metrics.valid = true;
    return metrics;
}

xMate3Kinematics::IkSelectionResult xMate3Kinematics::selectBestIkSolution(
        const std::vector<std::vector<double>>& candidates,
        const std::vector<double>& target_pose,
        const std::vector<double>& seed_joints,
        const CartesianIkOptions& options) {
    IkSelectionResult result;
    if (!backend_) {
        result.message = "kinematics backend unavailable";
        return result;
    }

    detail::KinematicsBackend::CartesianIkOptions backend_options;
    backend_options.requested_conf = options.requested_conf;
    backend_options.strict_conf = options.strict_conf;
    backend_options.avoid_singularity = options.avoid_singularity;
    backend_options.soft_limit_enabled = options.soft_limit_enabled;
    backend_options.soft_limits = options.soft_limits;
    backend_options.joint_limits_min = joint_limits_min_;
    backend_options.joint_limits_max = joint_limits_max_;

    const auto target_transform = rpyToTransform(target_pose);
    const auto selected =
        backend_->selectBestCartesianIkSolution(candidates, target_transform, seed_joints, backend_options);
    result.success = selected.success;
    result.joints = selected.joints;
    result.message = selected.message;
    return result;
}

bool xMate3Kinematics::buildCartesianJointTrajectory(
        const std::vector<std::vector<double>>& cartesian_trajectory,
        const std::vector<double>& initial_seed,
        const CartesianIkOptions& options,
        std::vector<std::vector<double>>& joint_trajectory,
        std::vector<double>& last_joints,
        std::string& error_message) {
    joint_trajectory.clear();
    last_joints = initial_seed;
    if (!backend_) {
        error_message = "kinematics backend unavailable";
        return false;
    }

    detail::KinematicsBackend::CartesianIkOptions backend_options;
    backend_options.requested_conf = options.requested_conf;
    backend_options.strict_conf = options.strict_conf;
    backend_options.avoid_singularity = options.avoid_singularity;
    backend_options.soft_limit_enabled = options.soft_limit_enabled;
    backend_options.soft_limits = options.soft_limits;
    backend_options.joint_limits_min = joint_limits_min_;
    backend_options.joint_limits_max = joint_limits_max_;

    std::vector<Matrix4d> cartesian_transforms;
    cartesian_transforms.reserve(cartesian_trajectory.size());
    for (const auto &pose : cartesian_trajectory) {
        cartesian_transforms.push_back(rpyToTransform(pose));
    }

    return backend_->buildCartesianJointTrajectory(
        cartesian_transforms, initial_seed, backend_options, joint_trajectory, last_joints, error_message);
}

bool xMate3Kinematics::projectCartesianJointDerivatives(
        const std::vector<std::vector<double>>& cartesian_trajectory,
        const std::vector<std::vector<double>>& joint_trajectory,
        double trajectory_dt,
        std::vector<std::vector<double>>& joint_velocity_trajectory,
        std::vector<std::vector<double>>& joint_acceleration_trajectory) {
    joint_velocity_trajectory.clear();
    joint_acceleration_trajectory.clear();
    if (!backend_) {
        return false;
    }

    std::vector<Matrix4d> cartesian_transforms;
    cartesian_transforms.reserve(cartesian_trajectory.size());
    for (const auto &pose : cartesian_trajectory) {
        cartesian_transforms.push_back(rpyToTransform(pose));
    }

    debug_counters_.jacobian_calls += joint_trajectory.size();
    debug_counters_.svd_calls += joint_trajectory.size();

    return backend_->projectCartesianJointDerivatives(
        cartesian_transforms,
        joint_trajectory,
        trajectory_dt,
        joint_velocity_trajectory,
        joint_acceleration_trajectory);
}

void xMate3Kinematics::resetDebugCounters() {
    debug_counters_ = DebugCounters{};
}

xMate3Kinematics::DebugCounters xMate3Kinematics::debugCounters() const {
    return debug_counters_;
}

const char *xMate3Kinematics::backendName() const noexcept {
    if (!backend_) {
        return "legacy";
    }
    const std::string name = backend_->name();
    if (name.rfind("kdl", 0) == 0) {
        return "kdl";
    }
    return "legacy";
}

// -------------------------- 改进DH变换矩阵 - 完全保留你的原版 --------------------------
Matrix4d xMate3Kinematics::dhTransform(int i, double theta) {
    Matrix4d T;
    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(dh_alpha_[i]);
    double sa = sin(dh_alpha_[i]);

    T << ct,      -st,        0,        dh_a_[i],
         st*ca,   ct*ca,     -sa,      -dh_d_[i]*sa,
         st*sa,   ct*sa,      ca,       dh_d_[i]*ca,
         0,        0,          0,        1;

    return T;
}

// -------------------------- RPY转变换矩阵 - 完全保留你的原版 --------------------------
Matrix4d xMate3Kinematics::rpyToTransform(const std::vector<double>& pose) {
    double rx = pose[3];
    double ry = pose[4];
    double rz = pose[5];

    Matrix3d Rx, Ry, Rz;
    Rx << 1,0,0,
          0,cos(rx),-sin(rx),
          0,sin(rx),cos(rx);

    Ry << cos(ry),0,sin(ry),
          0,1,0,
          -sin(ry),0,cos(ry);

    Rz << cos(rz),-sin(rz),0,
          sin(rz),cos(rz),0,
          0,0,1;

    Matrix4d T = Matrix4d::Identity();
    T.block<3,3>(0,0) = Rz * Ry * Rx;
    T(0,3) = pose[0];
    T(1,3) = pose[1];
    T(2,3) = pose[2];

    return T;
}

// -------------------------- 位姿误差计算 - 完全保留你的原版 --------------------------
xMate3Kinematics::Vector6d
xMate3Kinematics::computePoseError(
        const Matrix4d& T_target,
        const Matrix4d& T_current) {

    Vector6d error;
    Vector3d p_target = T_target.block<3,1>(0,3);
    Vector3d p_current = T_current.block<3,1>(0,3);
    error.head<3>() = p_target - p_current;

    Matrix3d Rd = T_target.block<3,3>(0,0);
    Matrix3d Rc = T_current.block<3,3>(0,0);
    Matrix3d Re = Rd * Rc.transpose();

    AngleAxisd aa(Re);
    Vector3d orientation_error = aa.angle() * aa.axis();

    if (std::isnan(orientation_error.norm()) || aa.angle() < 1e-6) {
        orientation_error << 
            Re(2,1) - Re(1,2),
            Re(0,2) - Re(2,0),
            Re(1,0) - Re(0,1);
        orientation_error *= 0.5;
    }

    error.tail<3>() = orientation_error;
    return error;
}

} // namespace gazebo
