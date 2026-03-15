/**
 * @file trajectory_planner.cpp
 * @brief 轨迹规划器实现（修正优化版）
 */

#include "rokae_xmate3_ros2/gazebo/trajectory_planner.hpp"
#include <algorithm>
#include <stdexcept>

namespace gazebo {

// -------------------------- 内部辅助工具函数（仅本文件可见） --------------------------
namespace {

/**
 * @brief 通用参数安全校验
 */
bool checkInputValid(
    const std::vector<double>& start,
    const std::vector<double>& end,
    size_t expect_dim,
    double speed_percent,
    double dt) {
    if (start.size() != expect_dim || end.size() != expect_dim) return false;
    if (speed_percent <= 1e-6 || speed_percent > 100.0) return false;
    if (dt <= 1e-9) return false;
    return true;
}

/**
 * @brief RPY欧拉角转四元数（XYZ固定角）
 */
Quaterniond rpy2Quat(const std::vector<double>& rpy) {
    return AngleAxisd(rpy[2], Vector3d::UnitZ())
         * AngleAxisd(rpy[1], Vector3d::UnitY())
         * AngleAxisd(rpy[0], Vector3d::UnitX());
}

/**
 * @brief 四元数转RPY欧拉角
 */
std::vector<double> quat2Rpy(const Quaterniond& quat) {
    Vector3d rpy = quat.matrix().eulerAngles(2, 1, 0).reverse();
    return {rpy[0], rpy[1], rpy[2]};
}

/**
 * @brief 姿态球面线性插值（SLERP）
 */
std::vector<double> slerpAttitude(
    const std::vector<double>& start_rpy,
    const std::vector<double>& end_rpy,
    double s) {
    Quaterniond q_start = rpy2Quat(start_rpy);
    Quaterniond q_end = rpy2Quat(end_rpy);
    Quaterniond q_slerp = q_start.slerp(s, q_end);
    return quat2Rpy(q_slerp);
}

/**
 * @brief 三点法计算圆弧的圆心、半径、旋转轴
 */
bool computeCircleParams(
    const Vector3d& p1, const Vector3d& p2, const Vector3d& p3,
    Vector3d& center, double& radius, Vector3d& rotate_axis) {
    Vector3d v1 = p2 - p1;
    Vector3d v2 = p3 - p1;
    rotate_axis = v1.cross(v2);
    
    if (rotate_axis.norm() < 1e-6) return false; // 三点共线
    rotate_axis.normalize();

    double v1_sq = v1.squaredNorm();
    double v2_sq = v2.squaredNorm();
    double v1_dot_v2 = v1.dot(v2);
    double denom = 2.0 * rotate_axis.squaredNorm();

    double k1 = (v2_sq * v1_dot_v2 - v1_sq * v2.dot(v2)) / denom;
    double k2 = (v1_sq * v1_dot_v2 - v2_sq * v1.dot(v1)) / denom;

    center = p1 + k1 * v1 + k2 * v2;
    radius = (p1 - center).norm();
    return true;
}

} // 匿名命名空间结束

// -------------------------- 核心规划函数实现 --------------------------

/**
 * @brief 计算梯形加减速的归一化插值比例（修正版）
 * @note 若需S型曲线，可替换为五次多项式或双S型实现
 */
double TrajectoryPlanner::computeBlendRatio(double t, double T) {
    constexpr double ACCEL_RATIO = 0.2; // 加减速时间占比
    const double ta = ACCEL_RATIO * T;  // 加速时间
    const double td = ta;                // 减速时间
    const double tc = T - ta - td;       // 匀速时间

    // 边界值保护
    if (t <= 0.0) return 0.0;
    if (t >= T) return 1.0;

    // 计算归一化最大速度（保证总位移严格为1）
    const double v_max = 1.0 / (0.5 * ta + tc + 0.5 * td);
    const double a = v_max / ta; // 匀加速度

    // 梯形速度规划分段计算
    if (t < ta) {
        return 0.5 * a * t * t;
    } else if (t < ta + tc) {
        return 0.5 * a * ta * ta + v_max * (t - ta);
    } else {
        const double t_dec = T - t;
        return 1.0 - 0.5 * a * t_dec * t_dec;
    }
}

/**
 * @brief 关节空间直线插补 (MoveJ/MoveAbsJ)
 */
std::vector<std::vector<double>> TrajectoryPlanner::planJointMove(
    const std::vector<double>& start,
    const std::vector<double>& end,
    double speed_percent,
    double dt) {
    
    std::vector<std::vector<double>> trajectory;
    // 入参合法性校验
    if (!checkInputValid(start, end, start.size(), speed_percent, dt) || start.empty()) {
        return trajectory;
    }

    // 计算最大关节位移
    double max_displacement = 0.0;
    for (size_t i = 0; i < start.size(); ++i) {
        max_displacement = std::max(max_displacement, std::abs(end[i] - start[i]));
    }
    // 零位移直接返回起点
    if (max_displacement < 1e-9) {
        trajectory.push_back(start);
        return trajectory;
    }

    // 时间规划（适配xMate3额定关节最大速度π rad/s）
    const double max_vel = M_PI * speed_percent / 100.0;
    const double total_time = max_displacement / max_vel;
    const int num_points = std::max(10, static_cast<int>(std::ceil(total_time / dt)));

    // 生成轨迹点
    trajectory.reserve(num_points + 1);
    for (int i = 0; i <= num_points; ++i) {
        const double t = static_cast<double>(i) / num_points * total_time;
        const double s = computeBlendRatio(t, total_time);
        
        std::vector<double> point(start.size());
        for (size_t j = 0; j < start.size(); ++j) {
            point[j] = start[j] + s * (end[j] - start[j]);
        }
        trajectory.push_back(point);
    }
    
    return trajectory;
}

/**
 * @brief 笛卡尔空间直线插补 (MoveL)
 */
std::vector<std::vector<double>> TrajectoryPlanner::planCartesianLine(
    const std::vector<double>& start,
    const std::vector<double>& end,
    double speed_percent,
    double dt) {
    
    std::vector<std::vector<double>> trajectory;
    // 入参校验：笛卡尔位姿固定6维（xyz+rpy）
    if (!checkInputValid(start, end, 6, speed_percent, dt)) {
        return trajectory;
    }

    // 计算直线位移
    Vector3d p_start(start[0], start[1], start[2]);
    Vector3d p_end(end[0], end[1], end[2]);
    const double displacement = (p_end - p_start).norm();
    // 零位移直接返回起点
    if (displacement < 1e-9) {
        trajectory.push_back(start);
        return trajectory;
    }

    // 时间规划（适配xMate3额定末端最大线速度1m/s）
    const double max_vel = 1.0 * speed_percent / 100.0;
    const double total_time = displacement / max_vel;
    const int num_points = std::max(10, static_cast<int>(std::ceil(total_time / dt)));

    // 拆分起点/终点姿态
    const std::vector<double> start_rpy = {start[3], start[4], start[5]};
    const std::vector<double> end_rpy = {end[3], end[4], end[5]};

    // 生成轨迹点
    trajectory.reserve(num_points + 1);
    for (int i = 0; i <= num_points; ++i) {
        const double t = static_cast<double>(i) / num_points * total_time;
        const double s = computeBlendRatio(t, total_time);
        
        std::vector<double> point(6);
        // 位置线性插值
        Vector3d pos = p_start + s * (p_end - p_start);
        for (int j = 0; j < 3; ++j) {
            point[j] = pos[j];
        }
        // 姿态球面插值SLERP
        std::vector<double> rpy_slerp = slerpAttitude(start_rpy, end_rpy, s);
        for (int j = 0; j < 3; ++j) {
            point[3 + j] = rpy_slerp[j];
        }
        trajectory.push_back(point);
    }
    
    return trajectory;
}

/**
 * @brief 圆弧插补 (MoveC)
 */
std::vector<std::vector<double>> TrajectoryPlanner::planCircularArc(
    const std::vector<double>& start,
    const std::vector<double>& aux,
    const std::vector<double>& end,
    double speed_percent,
    double dt) {
    
    std::vector<std::vector<double>> trajectory;
    // 入参校验
    if (!checkInputValid(start, end, 6, speed_percent, dt) || aux.size() != 6) {
        return trajectory;
    }

    // 三点坐标提取
    Vector3d p1(start[0], start[1], start[2]);
    Vector3d p2(aux[0], aux[1], aux[2]);
    Vector3d p3(end[0], end[1], end[2]);

    // 计算圆弧参数
    Vector3d center, rotate_axis;
    double radius;
    if (!computeCircleParams(p1, p2, p3, center, radius, rotate_axis)) {
        return trajectory; // 三点共线，返回空轨迹
    }

    // 计算圆弧总角度与弧长
    Vector3d v_start = p1 - center;
    Vector3d v_end = p3 - center;
    const double total_angle = acos(std::clamp(v_start.dot(v_end) / (v_start.norm() * v_end.norm()), -1.0, 1.0));
    const double arc_length = radius * total_angle;

    // 时间规划（圆弧运动降速至0.5m/s）
    const double max_vel = 0.5 * speed_percent / 100.0;
    const double total_time = arc_length / max_vel;
    const int num_points = std::max(20, static_cast<int>(std::ceil(total_time / dt)));

    // 姿态拆分
    const std::vector<double> start_rpy = {start[3], start[4], start[5]};
    const std::vector<double> end_rpy = {end[3], end[4], end[5]};

    // 生成圆弧轨迹
    trajectory.reserve(num_points + 1);
    for (int i = 0; i <= num_points; ++i) {
        const double t = static_cast<double>(i) / num_points * total_time;
        const double s = computeBlendRatio(t, total_time);
        const double current_angle = s * total_angle;

        std::vector<double> point(6);
        // 精确圆弧位置计算（绕轴旋转）
        Vector3d pos = center + AngleAxisd(current_angle, rotate_axis) * v_start;
        for (int j = 0; j < 3; ++j) {
            point[j] = pos[j];
        }
        // 姿态平滑插值
        std::vector<double> rpy_slerp = slerpAttitude(start_rpy, end_rpy, s);
        for (int j = 0; j < 3; ++j) {
            point[3 + j] = rpy_slerp[j];
        }
        trajectory.push_back(point);
    }
    
    return trajectory;
}

/**
 * @brief 连续圆弧运动 (MoveCF)
 */
std::vector<std::vector<double>> TrajectoryPlanner::planCircularContinuous(
    const std::vector<double>& start,
    const std::vector<double>& aux,
    const std::vector<double>& end,
    double angle,
    double speed_percent,
    double dt) {

    std::vector<std::vector<double>> trajectory;
    // 入参校验
    if (!checkInputValid(start, end, 6, speed_percent, dt) || aux.size() != 6 || std::abs(angle) < 1e-6) {
        return trajectory;
    }

    // 三点坐标提取
    Vector3d p1(start[0], start[1], start[2]);
    Vector3d p2(aux[0], aux[1], aux[2]);
    Vector3d p3(end[0], end[1], end[2]);

    // 计算圆弧参数
    Vector3d center, rotate_axis;
    double radius;
    if (!computeCircleParams(p1, p2, p3, center, radius, rotate_axis)) {
        return trajectory;
    }

    // 自定义角度弧长计算
    const double arc_length = radius * std::abs(angle);
    // 时间规划
    const double max_vel = 0.5 * speed_percent / 100.0;
    const double total_time = arc_length / max_vel;
    const int num_points = std::max(20, static_cast<int>(std::ceil(total_time / dt)));

    // 姿态拆分
    const std::vector<double> start_rpy = {start[3], start[4], start[5]};
    const std::vector<double> end_rpy = {end[3], end[4], end[5]};
    Vector3d v_start = p1 - center;

    // 生成连续圆弧轨迹
    trajectory.reserve(num_points + 1);
    for (int i = 0; i <= num_points; ++i) {
        const double t = static_cast<double>(i) / num_points * total_time;
        const double s = computeBlendRatio(t, total_time);
        const double current_angle = s * angle;

        std::vector<double> point(6);
        // 自定义角度圆弧位置计算
        Vector3d pos = center + AngleAxisd(current_angle, rotate_axis) * v_start;
        for (int j = 0; j < 3; ++j) {
            point[j] = pos[j];
        }
        // 姿态平滑插值
        std::vector<double> rpy_slerp = slerpAttitude(start_rpy, end_rpy, s);
        for (int j = 0; j < 3; ++j) {
            point[3 + j] = rpy_slerp[j];
        }
        trajectory.push_back(point);
    }

    return trajectory;
}

/**
 * @brief 螺旋线运动 (MoveSP)
 */
std::vector<std::vector<double>> TrajectoryPlanner::planSpiralMove(
    const std::vector<double>& start,
    const std::vector<double>& end,
    double radius,
    double radius_step,
    double angle,
    bool direction,
    double speed_percent,
    double dt) {

    std::vector<std::vector<double>> trajectory;
    // 入参校验
    if (!checkInputValid(start, end, 6, speed_percent, dt) || std::abs(angle) < 1e-6 || radius < 0) {
        return trajectory;
    }

    // 起点终点坐标
    Vector3d p_start(start[0], start[1], start[2]);
    Vector3d p_end(end[0], end[1], end[2]);
    Vector3d spiral_axis = p_end - p_start;
    const double axis_length = spiral_axis.norm();

    // 零轴长直接返回
    if (axis_length < 1e-9) {
        trajectory.push_back(start);
        return trajectory;
    }
    spiral_axis.normalize();

    // 构建螺旋局部坐标系（z轴为螺旋轴）
    Vector3d local_x, local_y;
    if (std::abs(spiral_axis.dot(Vector3d::UnitZ())) < 0.999) {
        local_x = spiral_axis.cross(Vector3d::UnitZ()).normalized();
    } else {
        local_x = spiral_axis.cross(Vector3d::UnitX()).normalized();
    }
    local_y = spiral_axis.cross(local_x).normalized();

    // 螺旋线总长度估算
    const double avg_radius = radius + radius_step * angle * 0.5;
    const double spiral_length = axis_length + avg_radius * std::abs(angle);
    // 时间规划（螺旋运动降速至0.3m/s）
    const double max_vel = 0.3 * speed_percent / 100.0;
    const double total_time = spiral_length / max_vel;
    const int num_points = std::max(30, static_cast<int>(std::ceil(total_time / dt)));

    // 姿态拆分
    const std::vector<double> start_rpy = {start[3], start[4], start[5]};
    const std::vector<double> end_rpy = {end[3], end[4], end[5]};
    const int rotate_dir = direction ? 1 : -1;

    // 生成螺旋轨迹
    trajectory.reserve(num_points + 1);
    for (int i = 0; i <= num_points; ++i) {
        const double t = static_cast<double>(i) / num_points * total_time;
        const double s = computeBlendRatio(t, total_time);
        const double current_angle = s * angle * rotate_dir;
        const double current_radius = radius + radius_step * s * angle;

        std::vector<double> point(6);
        // 螺旋线位置计算（沿螺旋轴的直线+局部平面圆周运动）
        Vector3d axis_pos = p_start + s * (p_end - p_start);
        Vector3d circle_offset = current_radius * (cos(current_angle) * local_x + sin(current_angle) * local_y);
        Vector3d pos = axis_pos + circle_offset;

        for (int j = 0; j < 3; ++j) {
            point[j] = pos[j];
        }
        // 姿态平滑插值
        std::vector<double> rpy_slerp = slerpAttitude(start_rpy, end_rpy, s);
        for (int j = 0; j < 3; ++j) {
            point[3 + j] = rpy_slerp[j];
        }
        trajectory.push_back(point);
    }

    return trajectory;
}

} // namespace gazebo