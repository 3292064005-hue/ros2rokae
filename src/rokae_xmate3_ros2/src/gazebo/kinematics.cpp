/**
 * @file kinematics.cpp
 * @brief xMate3 运动学计算实现 (改进 DH 版 - 有限差分雅可比修复版)
 */

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "gazebo/kinematics_backend.hpp"
#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gazebo {
namespace {

detail::KinematicsBackend::IKRequest makeIkRequest(
    const Matrix4d &target_transform,
    const std::vector<double> &seed_joints,
    const std::vector<double> &joint_limits_min,
    const std::vector<double> &joint_limits_max) {
    detail::KinematicsBackend::IKRequest request;
    request.target_transform = target_transform;
    request.seed_joints = seed_joints;
    request.joint_limits_min = joint_limits_min;
    request.joint_limits_max = joint_limits_max;
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
    // Improved DH helper model kept as the authoritative auxiliary solver seed / fallback model.
    dh_a_.assign(rokae_xmate3_ros2::spec::xmate3::improved_dh::kA.begin(),
                 rokae_xmate3_ros2::spec::xmate3::improved_dh::kA.end());
    dh_alpha_.assign(rokae_xmate3_ros2::spec::xmate3::improved_dh::kAlpha.begin(),
                     rokae_xmate3_ros2::spec::xmate3::improved_dh::kAlpha.end());
    dh_d_.assign(rokae_xmate3_ros2::spec::xmate3::improved_dh::kD.begin(),
                 rokae_xmate3_ros2::spec::xmate3::improved_dh::kD.end());

    joint_limits_min_.assign(rokae_xmate3_ros2::spec::xmate3::kJointLimitMin.begin(),
                             rokae_xmate3_ros2::spec::xmate3::kJointLimitMin.end());
    joint_limits_max_.assign(rokae_xmate3_ros2::spec::xmate3::kJointLimitMax.begin(),
                             rokae_xmate3_ros2::spec::xmate3::kJointLimitMax.end());
    policy_ = detail::resolveKinematicsPolicy();
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
    if (!backend_ || target.size() < 6 || current_joints.size() < 6) {
        return {};
    }

    const auto request = makeIkRequest(
        rpyToTransform(target),
        current_joints,
        joint_limits_min_,
        joint_limits_max_);
    const auto solved = backend_->solveMultiBranch(request);
    std::vector<std::vector<double>> candidates;
    candidates.reserve(solved.candidates.size());
    for (const auto &candidate : solved.candidates) {
        candidates.push_back(candidate.q);
    }
    return candidates;
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
    if (!backend_ || target.size() < 6 || seed_joints.size() < 6) {
        return {};
    }

    const auto request = makeIkRequest(
        rpyToTransform(target),
        seed_joints,
        joint_limits_min_,
        joint_limits_max_);
    return backend_->solveSeeded(request).q;
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
    if (!backend_ || joints.size() < 6) {
        return analysis;
    }

    analysis.jacobian = computeJacobian(joints);
    ++debug_counters_.svd_calls;
    const auto request = makeIkRequest(
        forwardKinematics(joints),
        joints,
        joint_limits_min_,
        joint_limits_max_);
    const auto metrics = backend_->evaluateSeededIkCandidate(request, joints);
    analysis.singularity_measure = metrics.singularity_metric;
    analysis.near_singularity = metrics.near_singularity;
    return analysis;
}

bool xMate3Kinematics::isNearSingularity(const std::vector<double>& joints) {
    return analyzeSingularity(joints).near_singularity;
}

double xMate3Kinematics::computeSingularityMeasure(const std::vector<double>& joints) {
    return analyzeSingularity(joints).singularity_measure;
}

bool xMate3Kinematics::avoidSingularity(std::vector<double>& joints) {
    if (!backend_) {
        return false;
    }
    return backend_->avoidSingularity(joints);
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

    auto request = makeIkRequest(
        Matrix4d::Identity(),
        seed_joints,
        joint_limits_min_,
        joint_limits_max_);

    if (backend_) {
        return toPublicMetrics(backend_->evaluateSeededIkCandidate(request, candidate));
    }

    IkCandidateMetrics metrics;
    metrics.branch_distance = branchDistance(candidate, seed_joints);
    metrics.continuity_cost =
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
    metrics.joint_limit_penalty = limit_penalty / 6.0;
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
    result.joints = selected.q;
    result.branch_id = selected.branch_id;
    result.note = selected.note;
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
        return "none";
    }
    const std::string name = backend_->name();
    if (name.rfind("kdl", 0) == 0) {
        return "kdl";
    }
    if (name.rfind("improved_dh", 0) == 0) {
        return "improved_dh";
    }
    return "unknown";
}

const KinematicsPolicy &xMate3Kinematics::policy() const noexcept {
    return policy_;
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
