/**
 * @file kinematics.hpp
 * @brief xMate3 运动学计算类
 */

#ifndef ROKAE_XMATE3_GAZEBO_KINEMATICS_HPP
#define ROKAE_XMATE3_GAZEBO_KINEMATICS_HPP

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"
#include "rokae_xmate3_ros2/gazebo/kinematics_policy.hpp"

namespace gazebo {

namespace detail {
class KinematicsBackend;
}

using namespace Eigen;

/**
 * @brief xMate3 运动学计算类
 */
class xMate3Kinematics {
public:
    using Vector6d = Matrix<double, 6, 1>;
    using Matrix6d = Matrix<double, 6, 6>;

    struct DebugCounters {
        std::size_t jacobian_calls = 0;
        std::size_t svd_calls = 0;
    };

    struct IkCandidateMetrics {
        double branch_distance = std::numeric_limits<double>::infinity();
        double continuity_cost = std::numeric_limits<double>::infinity();
        double joint_limit_penalty = std::numeric_limits<double>::infinity();
        double singularity_metric = 1.0;
        double singularity_cost = std::numeric_limits<double>::infinity();
        bool near_singularity = true;
        bool valid = false;

        [[nodiscard]] double totalCost() const noexcept {
            return continuity_cost + joint_limit_penalty + singularity_cost;
        }
    };

    struct CartesianIkOptions {
        std::vector<int> requested_conf;
        bool strict_conf = false;
        bool avoid_singularity = true;
        bool soft_limit_enabled = false;
        std::array<std::array<double, 2>, 6> soft_limits = rokae_xmate3_ros2::spec::xmate3::kDefaultSoftLimits;
    };

    struct IkSelectionResult {
        bool success = false;
        std::vector<double> joints;
        std::string branch_id;
        std::string note;
        std::string message;
    };

    xMate3Kinematics();
    
    /**
     * @brief 正运动学计算
     * @param joints 6个关节角度 (rad)
     * @return 4x4 齐次变换矩阵
     */
    Matrix4d forwardKinematics(const std::vector<double>& joints);
    std::vector<std::vector<double>> inverseKinematicsMultiSolution(
    const std::vector<double>& target,
    const std::vector<double>& current_joints);
    /**
     * @brief 正运动学 - 提取位置和RPY角
     */
    std::vector<double> forwardKinematicsRPY(const std::vector<double>& joints);
    
    /**
     * @brief 逆运动学计算 (数值解法)
     * @param target 目标位姿 [x,y,z,rx,ry,rz] (m, rad)
     * @param current_joints 当前关节角度 (用于选择最优解)
     * @return 6个关节角度
     */
    std::vector<double> inverseKinematics(const std::vector<double>& target, 
                                          const std::vector<double>& current_joints);

    /**
     * @brief 单种子快速逆运动学，优先用于笛卡尔路径连续求解
     */
    std::vector<double> inverseKinematicsSeededFast(const std::vector<double>& target,
                                                    const std::vector<double>& seed_joints);
    
    /**
     * @brief 计算雅可比矩阵
     */
    Matrix6d computeJacobian(const std::vector<double>& joints);

    /**
     * @brief 检测是否接近奇异位
     * @param joints 关节角度
     * @return 是否接近奇异位
     */
    bool isNearSingularity(const std::vector<double>& joints);

    /**
     * @brief 计算奇异度指标（0-1，越大越接近奇异）
     */
    double computeSingularityMeasure(const std::vector<double>& joints);

    /**
     * @brief 避开奇异位的关节调整
     * @param joints 当前关节角度（输入输出参数）
     * @return 是否成功调整
     */
    bool avoidSingularity(std::vector<double>& joints);

    [[nodiscard]] double branchDistance(const std::vector<double>& lhs,
                                        const std::vector<double>& rhs) const;
    [[nodiscard]] IkCandidateMetrics evaluateIkCandidate(const std::vector<double>& candidate,
                                                         const std::vector<double>& seed_joints) const;
    [[nodiscard]] IkSelectionResult selectBestIkSolution(
        const std::vector<std::vector<double>>& candidates,
        const std::vector<double>& target_pose,
        const std::vector<double>& seed_joints,
        const CartesianIkOptions& options);
    [[nodiscard]] bool buildCartesianJointTrajectory(
        const std::vector<std::vector<double>>& cartesian_trajectory,
        const std::vector<double>& initial_seed,
        const CartesianIkOptions& options,
        std::vector<std::vector<double>>& joint_trajectory,
        std::vector<double>& last_joints,
        std::string& error_message);
    [[nodiscard]] bool projectCartesianJointDerivatives(
        const std::vector<std::vector<double>>& cartesian_trajectory,
        const std::vector<std::vector<double>>& joint_trajectory,
        double trajectory_dt,
        std::vector<std::vector<double>>& joint_velocity_trajectory,
        std::vector<std::vector<double>>& joint_acceleration_trajectory);

    void resetDebugCounters();
    [[nodiscard]] DebugCounters debugCounters() const;
    [[nodiscard]] const char *backendName() const noexcept;
    [[nodiscard]] const KinematicsPolicy &policy() const noexcept;

 private:
    struct SingularityAnalysis;

    std::vector<double> dh_a_, dh_alpha_, dh_d_;
    std::vector<double> joint_limits_min_, joint_limits_max_;
    mutable DebugCounters debug_counters_{};
    KinematicsPolicy policy_{};
    std::shared_ptr<detail::KinematicsBackend> backend_;
    std::vector<Matrix4d> computeAllTransforms(const std::vector<double>& joints);
    Matrix4d dhTransform(int i, double theta);
    Matrix4d rpyToTransform(const std::vector<double>& pose);
    Vector6d computePoseError(const Matrix4d& T_target, const Matrix4d& T_current);
    [[nodiscard]] SingularityAnalysis analyzeSingularity(const std::vector<double>& joints);
};

} // namespace gazebo

#endif // ROKAE_XMATE3_GAZEBO_KINEMATICS_HPP
