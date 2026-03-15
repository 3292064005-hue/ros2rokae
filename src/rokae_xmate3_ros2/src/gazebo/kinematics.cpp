/**
 * @file kinematics.cpp
 * @brief xMate3 运动学计算实现 (改进 DH 版 - 有限差分雅可比修复版)
 */

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include <limits>
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gazebo {

xMate3Kinematics::xMate3Kinematics() {
    // DH 参数 (单位 m) - 完全保留你的原版
    dh_a_ = {0.0, 0.0, 0.394, 0.0, 0.0, 0.0};
    dh_alpha_ = {0.0, -M_PI/2, 0.0, M_PI/2, -M_PI/2, M_PI/2};
    dh_d_ = {0.3415, 0.0, 0.0, 0.366, 0.0, 0.2503};

    // 关节限位 - 完全保留你的原版
    joint_limits_min_ = {-3.0527, -2.0933, -2.0933, -3.0527, -2.0933, -6.1082};
    joint_limits_max_ = {3.0527, 2.0933, 2.0933, 3.0527, 2.0933, 6.1082};
}

// ==================================================
// 核心辅助函数：100%保留你的初始偏移逻辑
// ==================================================
std::vector<Matrix4d> xMate3Kinematics::computeAllTransforms(const std::vector<double>& joints) {
    std::vector<Matrix4d> T(7);
    T[0] = Matrix4d::Identity();

    // ========== 核心偏移逻辑 - 完全保留你的原版 ==========
    std::vector<double> jnt = joints;
    jnt[1] -= M_PI / 2.0;
    jnt[2] += M_PI / 2.0;
    // =====================================================

    for (int i = 0; i < 6; ++i) {
        T[i+1] = T[i] * dhTransform(i, jnt[i]);
    }

    return T;
}

// -------------------------- 正运动学 - 完全保留你的原版 --------------------------
Matrix4d xMate3Kinematics::forwardKinematics(const std::vector<double>& joints) {
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
// -------------------------- 逆运动学 - 多解输出版（带奇异位处理） --------------------------
// 新增返回值：所有满足精度的有效逆解，按与当前关节的运动距离从小到大排序
std::vector<std::vector<double>> xMate3Kinematics::inverseKinematicsMultiSolution(
        const std::vector<double>& target,
        const std::vector<double>& current_joints) {

    Matrix4d T_target = rpyToTransform(target);

    // ========== 迭代参数（增强鲁棒性配置） ==========
    const int max_iter = 1000;          // 增加迭代次数
    const double position_tolerance = 1e-5;  // 稍微放宽容差
    const double orientation_tolerance = 1e-3;
    const double orientation_weight = 0.8;
    const double max_joint_step = 0.02;      // 减小步长，增加稳定性
    const double min_lambda = 0.02;           // 增加最小阻尼
    const double max_lambda = 0.8;            // 增加最大阻尼

    // ========== 多解核心：有效解校验阈值 ==========
    const double solution_valid_threshold = 5e-3; // 放宽有效解阈值
    const double duplicate_threshold = 0.05;      // 去重阈值：关节角差的范数小于此值视为同一解

    // ========== 增强版单种子求解核心逻辑（带奇异位处理） ==========
    auto solveFromSeed = [&](std::vector<double> joints) -> std::pair<std::vector<double>, double> {
        // 预检查：如果种子在奇异位附近，先调整
        if (isNearSingularity(joints)) {
            avoidSingularity(joints);
        }

        double best_score = std::numeric_limits<double>::infinity();
        std::vector<double> best_joints = joints;

        for (int iter = 0; iter < max_iter; ++iter) {
            Matrix4d T_current = forwardKinematics(joints);
            Vector6d error = computePoseError(T_target, T_current);

            double pos_err = error.head<3>().norm();
            double ori_err = error.tail<3>().norm();
            double current_score = pos_err + orientation_weight * ori_err;

            // 保存最佳解
            if (current_score < best_score) {
                best_score = current_score;
                best_joints = joints;
            }

            if (pos_err < position_tolerance && ori_err < orientation_tolerance) {
                break;
            }

            // 检查是否接近奇异位
            bool near_singular = isNearSingularity(joints);
            double singularity_measure = computeSingularityMeasure(joints);

            MatrixXd J = computeJacobian(joints);
            Matrix6d W = Matrix6d::Identity();
            W(3,3) = W(4,4) = W(5,5) = orientation_weight;

            // 在奇异位附近增加方向权重
            if (near_singular) {
                // 对关节4和6增加阻尼，避免过大运动
                W(3,3) *= 0.3;  // 降低姿态权重
                W(4,4) *= 0.3;
                W(5,5) *= 0.3;
            }

            Vector6d err_w = error;
            err_w.tail<3>() *= W(3,3);
            MatrixXd Jw = W * J;

            // 根据奇异度动态调整lambda
            double lambda_base = min_lambda + max_lambda * (1.0 - (double)iter/max_iter);
            double lambda = lambda_base * (1.0 + 2.0 * singularity_measure);  // 奇异位时增加阻尼

            MatrixXd JJt = Jw * Jw.transpose();
            Vector6d delta_q = Jw.transpose() * (JJt + lambda * lambda * Matrix6d::Identity()).inverse() * err_w;

            // 在奇异位附近进一步限制关节步长
            double step_scale = near_singular ? 0.5 : 1.0;

            for (int i = 0; i < 6; ++i) {
                double step = delta_q(i);
                step = std::clamp(step, -max_joint_step * step_scale, max_joint_step * step_scale);
                joints[i] += step;
                joints[i] = std::clamp(joints[i], joint_limits_min_[i], joint_limits_max_[i]);
            }
        }

        // 如果最终解不够好，尝试用最佳解
        Matrix4d T_final = forwardKinematics(joints);
        Vector6d final_error = computePoseError(T_target, T_final);
        double final_pos_err = final_error.head<3>().norm();
        double final_ori_err = final_error.tail<3>().norm();
        double score = final_pos_err + orientation_weight * final_ori_err;

        // 检查最佳解是否更好
        Matrix4d T_best = forwardKinematics(best_joints);
        Vector6d best_error = computePoseError(T_target, T_best);
        double best_pos_err = best_error.head<3>().norm();
        double best_ori_err = best_error.tail<3>().norm();
        double best_final_score = best_pos_err + orientation_weight * best_ori_err;

        if (best_final_score < score) {
            joints = best_joints;
            score = best_final_score;
            final_pos_err = best_pos_err;
            final_ori_err = best_ori_err;
        }

        // 标记无效解
        if (final_pos_err > solution_valid_threshold || final_ori_err > solution_valid_threshold) {
            score = std::numeric_limits<double>::infinity();
        }

        return {joints, score};
    };

    // ========== 多解核心：覆盖8种构型的种子库（增加奇异位规避） ==========
    std::vector<std::vector<double>> seeds;
    // 1. 必选：当前关节种子（优先收敛到最近解）
    {
        std::vector<double> seed = current_joints;
        if (isNearSingularity(seed)) {
            avoidSingularity(seed);
        }
        seeds.push_back(seed);
    }

    // 2. 覆盖肩左/肩右 构型（关节1正负极限附近）
    {
        std::vector<double> seed_shoulder_left = current_joints;
        seed_shoulder_left[0] = std::clamp(2.0, joint_limits_min_[0], joint_limits_max_[0]);
        seed_shoulder_left[4] = 0.3;  // 主动避开奇异位
        seeds.push_back(seed_shoulder_left);
    }
    {
        std::vector<double> seed_shoulder_right = current_joints;
        seed_shoulder_right[0] = std::clamp(-2.0, joint_limits_min_[0], joint_limits_max_[0]);
        seed_shoulder_right[4] = -0.3;  // 主动避开奇异位
        seeds.push_back(seed_shoulder_right);
    }

    // 3. 覆盖肘上/肘下 构型（关节3正负极限附近）
    {
        std::vector<double> seed_elbow_up = current_joints;
        seed_elbow_up[2] = std::clamp(1.5, joint_limits_min_[2], joint_limits_max_[2]);
        seed_elbow_up[4] = 0.4;  // 主动避开奇异位
        seeds.push_back(seed_elbow_up);
    }
    {
        std::vector<double> seed_elbow_down = current_joints;
        seed_elbow_down[2] = std::clamp(-1.5, joint_limits_min_[2], joint_limits_max_[2]);
        seed_elbow_down[4] = -0.4;  // 主动避开奇异位
        seeds.push_back(seed_elbow_down);
    }

    // 4. 覆盖腕正/腕翻 构型（关节6正负180度）
    {
        std::vector<double> seed_wrist_flip = current_joints;
        seed_wrist_flip[5] = std::clamp(M_PI, joint_limits_min_[5], joint_limits_max_[5]);
        seed_wrist_flip[4] = 0.25;  // 主动避开奇异位
        seeds.push_back(seed_wrist_flip);
    }
    {
        std::vector<double> seed_wrist_noflip = current_joints;
        seed_wrist_noflip[5] = std::clamp(0.0, joint_limits_min_[5], joint_limits_max_[5]);
        seed_wrist_noflip[4] = -0.25;  // 主动避开奇异位
        seeds.push_back(seed_wrist_noflip);
    }

    // 5. 补充：关节零位种子（兜底覆盖基础构型）- 但避开奇异位
    {
        std::vector<double> zero_seed = {0,0,0,0,0,0};
        zero_seed[4] = 0.2;  // 避开奇异位
        seeds.push_back(zero_seed);
    }
    {
        std::vector<double> zero_seed2 = {0,0,0,0,0,0};
        zero_seed2[4] = -0.2;  // 另一个方向避开奇异位
        seeds.push_back(zero_seed2);
    }

    // 6. 新增：专门针对奇异位附近的种子
    {
        std::vector<double> seed_sing_avoid1 = current_joints;
        seed_sing_avoid1[4] = 0.5;  // 明确避开奇异
        seeds.push_back(seed_sing_avoid1);
    }
    {
        std::vector<double> seed_sing_avoid2 = current_joints;
        seed_sing_avoid2[4] = -0.5;  // 明确避开奇异
        seeds.push_back(seed_sing_avoid2);
    }

    // ========== 批量求解 ==========
    std::vector<std::vector<double>> candidate_solutions;
    for (const auto& seed : seeds) {
        auto [candidate_joints, score] = solveFromSeed(seed);
        // 只保留有效解
        if (score < std::numeric_limits<double>::infinity()) {
            candidate_solutions.push_back(candidate_joints);
        }
    }

    // ========== 多解核心：去重处理 ==========
    std::vector<std::vector<double>> unique_solutions;
    for (const auto& sol : candidate_solutions) {
        bool is_duplicate = false;
        for (const auto& unique_sol : unique_solutions) {
            // 计算两个解的关节角差的范数
            double diff_norm = 0.0;
            for (int i = 0; i < 6; ++i) {
                diff_norm += pow(sol[i] - unique_sol[i], 2);
            }
            diff_norm = sqrt(diff_norm);
            if (diff_norm < duplicate_threshold) {
                is_duplicate = true;
                break;
            }
        }
        if (!is_duplicate) {
            unique_solutions.push_back(sol);
        }
    }

    // ========== 多解优化：按与当前关节的运动距离排序 ==========
    std::sort(unique_solutions.begin(), unique_solutions.end(),
        [&current_joints](const std::vector<double>& a, const std::vector<double>& b) {
            double dist_a = 0.0, dist_b = 0.0;
            for (int i = 0; i < 6; ++i) {
                dist_a += fabs(a[i] - current_joints[i]);
                dist_b += fabs(b[i] - current_joints[i]);
            }
            return dist_a < dist_b;
        });

    return unique_solutions;
}

// 保留原单解函数，兼容原有代码调用
std::vector<double> xMate3Kinematics::inverseKinematics(
        const std::vector<double>& target,
        const std::vector<double>& current_joints) {
    const auto solutions = inverseKinematicsMultiSolution(target, current_joints);
    if (solutions.empty()) {
        return {};
    }
    return solutions.front();
}
// -------------------------- 逆运动学 - 优化迭代参数 --------------------------

// -------------------------- 核心修复：位姿误差有限差分雅可比 --------------------------
MatrixXd xMate3Kinematics::computeJacobian(const std::vector<double>& joints)
{
    MatrixXd J(6, 6);
    const double delta = 1e-6;

    // 当前位姿
    Matrix4d T_current = forwardKinematics(joints);

    // 遍历每个关节
    for (int i = 0; i < 6; ++i) {
        std::vector<double> joints_plus = joints;
        joints_plus[i] += delta;
        Matrix4d T_plus = forwardKinematics(joints_plus);
        Vector6d error = computePoseError(T_plus, T_current);
        J.col(i) = error / delta;
    }

    return J;
}

// -------------------------- 奇异位检测：关节4和6共轴检测 --------------------------
bool xMate3Kinematics::isNearSingularity(const std::vector<double>& joints) {
    if (joints.size() < 6) {
        return true;
    }

    // 腕部奇异：关节5接近0时，关节4与6接近共轴。
    if (std::fabs(joints[4]) < WRIST_SINGULARITY_THRESHOLD) {
        return true;
    }

    // 同时结合整体雅可比条件数，避免只盯着腕部奇异。
    const MatrixXd jacobian = computeJacobian(joints);
    JacobiSVD<MatrixXd> svd(jacobian, ComputeThinU | ComputeThinV);
    if (svd.singularValues().size() == 0) {
        return true;
    }

    const double min_sigma = svd.singularValues().tail(1)(0);
    return min_sigma < JACOBIAN_SINGULARITY_THRESHOLD;
}

double xMate3Kinematics::computeSingularityMeasure(const std::vector<double>& joints) {
    if (joints.size() < 6) {
        return 1.0;
    }

    double wrist_measure = 0.0;
    const double j5 = std::fabs(joints[4]);
    if (j5 < WRIST_SINGULARITY_THRESHOLD) {
        wrist_measure = 1.0 - (j5 / WRIST_SINGULARITY_THRESHOLD);
    }

    const MatrixXd jacobian = computeJacobian(joints);
    JacobiSVD<MatrixXd> svd(jacobian, ComputeThinU | ComputeThinV);
    if (svd.singularValues().size() == 0) {
        return 1.0;
    }

    const double min_sigma = svd.singularValues().tail(1)(0);
    const double jacobian_measure = std::clamp(
        1.0 - (min_sigma / JACOBIAN_SINGULARITY_THRESHOLD), 0.0, 1.0);

    return std::max(wrist_measure, jacobian_measure);
}

bool xMate3Kinematics::avoidSingularity(std::vector<double>& joints) {
    // 如果不在奇异位附近，无需调整
    if (!isNearSingularity(joints)) {
        return true;
    }

    // 优先把腕部拉离 joint5=0 的共轴奇异位置。
    const double current_j5 = joints[4];
    const double target_offset = SINGULARITY_AVOIDANCE_OFFSET;
    joints[4] = current_j5 >= 0.0 ? std::max(target_offset, joint_limits_min_[4])
                                  : std::min(-target_offset, joint_limits_max_[4]);
    joints[4] = std::clamp(joints[4], joint_limits_min_[4], joint_limits_max_[4]);

    // 如果整体雅可比仍然接近奇异，再补一个轻微的肘部/腕部扰动，帮助种子跳出坏分支。
    if (computeSingularityMeasure(joints) > 0.7) {
        joints[2] = std::clamp(joints[2] + (joints[2] >= 0.0 ? 0.12 : -0.12),
                               joint_limits_min_[2], joint_limits_max_[2]);
        joints[3] = std::clamp(joints[3] + (joints[3] >= 0.0 ? 0.08 : -0.08),
                               joint_limits_min_[3], joint_limits_max_[3]);
        joints[5] = std::clamp(joints[5] + (joints[5] >= 0.0 ? 0.1 : -0.1),
                               joint_limits_min_[5], joint_limits_max_[5]);
    }

    return true;
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
