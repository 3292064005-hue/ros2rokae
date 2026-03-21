/**
 * @file trajectory_planner.cpp
 * @brief 轨迹规划器实现（修正优化版）
 */

#include "rokae_xmate3_ros2/gazebo/trajectory_planner.hpp"

#include <algorithm>
#include <cmath>

namespace gazebo {

namespace {

constexpr double kMinNrtSpeedMmPerSec = 5.0;
constexpr double kMaxNrtSpeedMmPerSec = 4000.0;
constexpr double kReferenceJointFullSpeedMmPerSec = 500.0;
constexpr double kMinTrajectorySampleDt = 0.001;
constexpr double kMaxTrajectorySampleDt = 0.05;
constexpr double kMaxJointStepRad = 0.025;
constexpr double kMaxCartesianStepMeters = 0.002;
constexpr double kMaxOrientationStepRad = 0.03;

bool checkInputValid(
    const std::vector<double>& start,
    const std::vector<double>& end,
    size_t expect_dim,
    double speed_mm_per_s,
    double dt) {
    if (start.size() != expect_dim || end.size() != expect_dim) return false;
    if (speed_mm_per_s < kMinNrtSpeedMmPerSec || speed_mm_per_s > kMaxNrtSpeedMmPerSec) return false;
    if (dt <= 1e-9) return false;
    return true;
}

Quaterniond rpy2Quat(const std::vector<double>& rpy) {
    return AngleAxisd(rpy[2], Vector3d::UnitZ())
         * AngleAxisd(rpy[1], Vector3d::UnitY())
         * AngleAxisd(rpy[0], Vector3d::UnitX());
}

std::vector<double> quat2Rpy(const Quaterniond& quat) {
    Vector3d rpy = quat.matrix().eulerAngles(2, 1, 0).reverse();
    return {rpy[0], rpy[1], rpy[2]};
}

double orientationDistance(const std::vector<double>& start_pose, const std::vector<double>& end_pose) {
    const Quaterniond q_start = rpy2Quat({start_pose[3], start_pose[4], start_pose[5]});
    const Quaterniond q_end = rpy2Quat({end_pose[3], end_pose[4], end_pose[5]});
    return q_start.angularDistance(q_end);
}

std::vector<double> slerpAttitude(
    const std::vector<double>& start_rpy,
    const std::vector<double>& end_rpy,
    double s) {
    Quaterniond q_start = rpy2Quat(start_rpy);
    Quaterniond q_end = rpy2Quat(end_rpy);
    Quaterniond q_slerp = q_start.slerp(s, q_end);
    return quat2Rpy(q_slerp);
}

double signedAngleAroundAxis(const Vector3d& from, const Vector3d& to, const Vector3d& axis) {
    const Vector3d from_n = from.normalized();
    const Vector3d to_n = to.normalized();
    const double sin_term = axis.normalized().dot(from_n.cross(to_n));
    const double cos_term = std::clamp(from_n.dot(to_n), -1.0, 1.0);
    return std::atan2(sin_term, cos_term);
}

bool computeCircleParams(
    const Vector3d& p1, const Vector3d& p2, const Vector3d& p3,
    Vector3d& center, double& radius, Vector3d& rotate_axis) {
    const Vector3d u = p2 - p1;
    const Vector3d v = p3 - p1;
    const Vector3d w = u.cross(v);
    const double w_sq = w.squaredNorm();

    if (w_sq < 1e-12) {
        return false;
    }

    rotate_axis = w.normalized();
    const Vector3d center_offset =
        (u.squaredNorm() * v.cross(w) + v.squaredNorm() * w.cross(u)) / (2.0 * w_sq);
    center = p1 + center_offset;
    radius = (p1 - center).norm();

    return std::isfinite(radius) && radius > 1e-9;
}

double clampCartesianSpeedMetersPerSecond(double speed_mm_per_s) {
    return std::clamp(speed_mm_per_s, kMinNrtSpeedMmPerSec, kMaxNrtSpeedMmPerSec) / 1000.0;
}

double clampJointSpeedRadPerSecond(double speed_mm_per_s) {
    const double normalized =
        std::clamp(speed_mm_per_s, kMinNrtSpeedMmPerSec, kMaxNrtSpeedMmPerSec) /
        kReferenceJointFullSpeedMmPerSec;
    return std::clamp(normalized * M_PI, 0.05, M_PI);
}

TrajectorySamples makeSinglePointTrajectory(const std::vector<double>& point, double fallback_dt) {
    TrajectorySamples result;
    result.points.push_back(point);
    result.sample_dt = std::clamp(fallback_dt, kMinTrajectorySampleDt, kMaxTrajectorySampleDt);
    result.total_time = 0.0;
    return result;
}

int determine_interval_count(double total_time,
                             double requested_dt,
                             double path_metric,
                             double max_path_step,
                             double orientation_metric = 0.0,
                             double max_orientation_step = kMaxOrientationStepRad) {
    if (total_time <= 1e-9) {
        return 1;
    }
    const double clamped_max_dt = std::clamp(requested_dt, kMinTrajectorySampleDt, kMaxTrajectorySampleDt);
    const int time_intervals = std::max(1, static_cast<int>(std::ceil(total_time / clamped_max_dt)));
    const int path_intervals = max_path_step > 1e-9
        ? std::max(1, static_cast<int>(std::ceil(path_metric / max_path_step)))
        : 1;
    const int orientation_intervals = max_orientation_step > 1e-9
        ? std::max(1, static_cast<int>(std::ceil(orientation_metric / max_orientation_step)))
        : 1;
    const int min_dt_intervals = std::max(1, static_cast<int>(std::ceil(total_time / kMinTrajectorySampleDt)));
    return std::clamp(std::max({time_intervals, path_intervals, orientation_intervals}), 1, min_dt_intervals);
}

TrajectorySamples finalizeTrajectory(std::vector<std::vector<double>> trajectory,
                                     int num_intervals,
                                     double total_time,
                                     double fallback_dt) {
    TrajectorySamples result;
    result.points = std::move(trajectory);
    result.total_time = std::max(total_time, 0.0);
    if (num_intervals > 0 && result.total_time > 1e-9) {
        result.sample_dt = result.total_time / static_cast<double>(num_intervals);
    } else {
        result.sample_dt = std::clamp(fallback_dt, kMinTrajectorySampleDt, kMaxTrajectorySampleDt);
    }
    return result;
}

} // namespace

double TrajectoryPlanner::computeBlendRatio(double t, double T) {
    constexpr double ACCEL_RATIO = 0.2;
    const double ta = ACCEL_RATIO * T;
    const double td = ta;
    const double tc = T - ta - td;

    if (t <= 0.0) return 0.0;
    if (t >= T) return 1.0;

    const double v_max = 1.0 / (0.5 * ta + tc + 0.5 * td);
    const double a = v_max / ta;

    if (t < ta) {
        return 0.5 * a * t * t;
    } else if (t < ta + tc) {
        return 0.5 * a * ta * ta + v_max * (t - ta);
    } else {
        const double t_dec = T - t;
        return 1.0 - 0.5 * a * t_dec * t_dec;
    }
}

TrajectorySamples TrajectoryPlanner::planJointMove(
    const std::vector<double>& start,
    const std::vector<double>& end,
    double speed_mm_per_s,
    double dt) {

    if (!checkInputValid(start, end, start.size(), speed_mm_per_s, dt) || start.empty()) {
        return {};
    }

    double max_displacement = 0.0;
    for (size_t i = 0; i < start.size(); ++i) {
        max_displacement = std::max(max_displacement, std::abs(end[i] - start[i]));
    }
    if (max_displacement < 1e-9) {
        return makeSinglePointTrajectory(start, dt);
    }

    const double max_vel = clampJointSpeedRadPerSecond(speed_mm_per_s);
    const double total_time = max_displacement / max_vel;
    const int num_points = determine_interval_count(total_time, dt, max_displacement, kMaxJointStepRad);

    std::vector<std::vector<double>> trajectory;
    trajectory.reserve(num_points + 1);
    for (int i = 0; i <= num_points; ++i) {
        const double t = static_cast<double>(i) / num_points * total_time;
        const double s = computeBlendRatio(t, total_time);

        std::vector<double> point(start.size());
        for (size_t j = 0; j < start.size(); ++j) {
            point[j] = start[j] + s * (end[j] - start[j]);
        }
        trajectory.push_back(std::move(point));
    }

    return finalizeTrajectory(std::move(trajectory), num_points, total_time, dt);
}

TrajectorySamples TrajectoryPlanner::planCartesianLine(
    const std::vector<double>& start,
    const std::vector<double>& end,
    double speed_mm_per_s,
    double dt) {

    if (!checkInputValid(start, end, 6, speed_mm_per_s, dt)) {
        return {};
    }

    Vector3d p_start(start[0], start[1], start[2]);
    Vector3d p_end(end[0], end[1], end[2]);
    const double displacement = (p_end - p_start).norm();
    if (displacement < 1e-9) {
        return makeSinglePointTrajectory(start, dt);
    }

    const double max_vel = clampCartesianSpeedMetersPerSecond(speed_mm_per_s);
    const double total_time = displacement / max_vel;
    const int num_points = determine_interval_count(
        total_time, dt, displacement, kMaxCartesianStepMeters, orientationDistance(start, end));

    const std::vector<double> start_rpy = {start[3], start[4], start[5]};
    const std::vector<double> end_rpy = {end[3], end[4], end[5]};

    std::vector<std::vector<double>> trajectory;
    trajectory.reserve(num_points + 1);
    for (int i = 0; i <= num_points; ++i) {
        const double t = static_cast<double>(i) / num_points * total_time;
        const double s = computeBlendRatio(t, total_time);

        std::vector<double> point(6);
        Vector3d pos = p_start + s * (p_end - p_start);
        for (int j = 0; j < 3; ++j) {
            point[j] = pos[j];
        }
        const std::vector<double> rpy_slerp = slerpAttitude(start_rpy, end_rpy, s);
        for (int j = 0; j < 3; ++j) {
            point[3 + j] = rpy_slerp[j];
        }
        trajectory.push_back(std::move(point));
    }

    return finalizeTrajectory(std::move(trajectory), num_points, total_time, dt);
}

TrajectorySamples TrajectoryPlanner::planCircularArc(
    const std::vector<double>& start,
    const std::vector<double>& aux,
    const std::vector<double>& end,
    double speed_mm_per_s,
    double dt) {

    if (!checkInputValid(start, end, 6, speed_mm_per_s, dt) || aux.size() != 6) {
        return {};
    }

    Vector3d p1(start[0], start[1], start[2]);
    Vector3d p2(aux[0], aux[1], aux[2]);
    Vector3d p3(end[0], end[1], end[2]);

    Vector3d center, rotate_axis;
    double radius;
    if (!computeCircleParams(p1, p2, p3, center, radius, rotate_axis)) {
        return {};
    }

    Vector3d v_start = p1 - center;
    Vector3d v_aux = p2 - center;
    Vector3d v_end = p3 - center;
    double aux_angle = signedAngleAroundAxis(v_start, v_aux, rotate_axis);
    double total_angle = signedAngleAroundAxis(v_start, v_end, rotate_axis);
    if (aux_angle < 0.0) {
        aux_angle += 2.0 * M_PI;
    }
    if (total_angle < 0.0) {
        total_angle += 2.0 * M_PI;
    }
    if (total_angle < aux_angle) {
        total_angle += 2.0 * M_PI;
    }
    const double arc_length = radius * total_angle;

    const double max_vel = clampCartesianSpeedMetersPerSecond(speed_mm_per_s);
    const double total_time = arc_length / max_vel;
    const int num_points = determine_interval_count(
        total_time, dt, arc_length, kMaxCartesianStepMeters, orientationDistance(start, end));

    const std::vector<double> start_rpy = {start[3], start[4], start[5]};
    const std::vector<double> end_rpy = {end[3], end[4], end[5]};

    std::vector<std::vector<double>> trajectory;
    trajectory.reserve(num_points + 1);
    for (int i = 0; i <= num_points; ++i) {
        const double t = static_cast<double>(i) / num_points * total_time;
        const double s = computeBlendRatio(t, total_time);
        const double current_angle = s * total_angle;

        std::vector<double> point(6);
        Vector3d pos = center + AngleAxisd(current_angle, rotate_axis) * v_start;
        for (int j = 0; j < 3; ++j) {
            point[j] = pos[j];
        }
        const std::vector<double> rpy_slerp = slerpAttitude(start_rpy, end_rpy, s);
        for (int j = 0; j < 3; ++j) {
            point[3 + j] = rpy_slerp[j];
        }
        trajectory.push_back(std::move(point));
    }

    return finalizeTrajectory(std::move(trajectory), num_points, total_time, dt);
}

TrajectorySamples TrajectoryPlanner::planCircularContinuous(
    const std::vector<double>& start,
    const std::vector<double>& aux,
    const std::vector<double>& end,
    double angle,
    double speed_mm_per_s,
    double dt) {

    if (!checkInputValid(start, end, 6, speed_mm_per_s, dt) || aux.size() != 6 || std::abs(angle) < 1e-6) {
        return {};
    }

    Vector3d p1(start[0], start[1], start[2]);
    Vector3d p2(aux[0], aux[1], aux[2]);
    Vector3d p3(end[0], end[1], end[2]);

    Vector3d center, rotate_axis;
    double radius;
    if (!computeCircleParams(p1, p2, p3, center, radius, rotate_axis)) {
        return {};
    }

    const double arc_length = radius * std::abs(angle);
    const double max_vel = clampCartesianSpeedMetersPerSecond(speed_mm_per_s);
    const double total_time = arc_length / max_vel;
    const int num_points = determine_interval_count(
        total_time, dt, arc_length, kMaxCartesianStepMeters, orientationDistance(start, end));

    const std::vector<double> start_rpy = {start[3], start[4], start[5]};
    const std::vector<double> end_rpy = {end[3], end[4], end[5]};
    Vector3d v_start = p1 - center;

    std::vector<std::vector<double>> trajectory;
    trajectory.reserve(num_points + 1);
    for (int i = 0; i <= num_points; ++i) {
        const double t = static_cast<double>(i) / num_points * total_time;
        const double s = computeBlendRatio(t, total_time);
        const double current_angle = s * angle;

        std::vector<double> point(6);
        Vector3d pos = center + AngleAxisd(current_angle, rotate_axis) * v_start;
        for (int j = 0; j < 3; ++j) {
            point[j] = pos[j];
        }
        const std::vector<double> rpy_slerp = slerpAttitude(start_rpy, end_rpy, s);
        for (int j = 0; j < 3; ++j) {
            point[3 + j] = rpy_slerp[j];
        }
        trajectory.push_back(std::move(point));
    }

    return finalizeTrajectory(std::move(trajectory), num_points, total_time, dt);
}

TrajectorySamples TrajectoryPlanner::planSpiralMove(
    const std::vector<double>& start,
    const std::vector<double>& end,
    double radius,
    double radius_step,
    double angle,
    bool direction,
    double speed_mm_per_s,
    double dt) {

    if (!checkInputValid(start, end, 6, speed_mm_per_s, dt) || std::abs(angle) < 1e-6 || radius < 0) {
        return {};
    }

    Vector3d p_start(start[0], start[1], start[2]);
    Vector3d p_end(end[0], end[1], end[2]);
    Vector3d spiral_axis = p_end - p_start;
    const double axis_length = spiral_axis.norm();

    if (axis_length < 1e-9) {
        return makeSinglePointTrajectory(start, dt);
    }
    spiral_axis.normalize();

    Vector3d local_x, local_y;
    if (std::abs(spiral_axis.dot(Vector3d::UnitZ())) < 0.999) {
        local_x = spiral_axis.cross(Vector3d::UnitZ()).normalized();
    } else {
        local_x = spiral_axis.cross(Vector3d::UnitX()).normalized();
    }
    local_y = spiral_axis.cross(local_x).normalized();

    const double avg_radius = radius + radius_step * angle * 0.5;
    const double spiral_length = axis_length + avg_radius * std::abs(angle);
    const double max_vel = clampCartesianSpeedMetersPerSecond(speed_mm_per_s);
    const double total_time = spiral_length / max_vel;
    const int num_points = determine_interval_count(
        total_time, dt, spiral_length, kMaxCartesianStepMeters, orientationDistance(start, end));

    const std::vector<double> start_rpy = {start[3], start[4], start[5]};
    const std::vector<double> end_rpy = {end[3], end[4], end[5]};
    const int rotate_dir = direction ? 1 : -1;

    std::vector<std::vector<double>> trajectory;
    trajectory.reserve(num_points + 1);
    for (int i = 0; i <= num_points; ++i) {
        const double t = static_cast<double>(i) / num_points * total_time;
        const double s = computeBlendRatio(t, total_time);
        const double current_angle = s * angle * rotate_dir;
        const double current_radius = radius + radius_step * s * angle;

        std::vector<double> point(6);
        Vector3d axis_pos = p_start + s * (p_end - p_start);
        Vector3d circle_offset = current_radius * (std::cos(current_angle) * local_x + std::sin(current_angle) * local_y);
        Vector3d pos = axis_pos + circle_offset;

        for (int j = 0; j < 3; ++j) {
            point[j] = pos[j];
        }
        const std::vector<double> rpy_slerp = slerpAttitude(start_rpy, end_rpy, s);
        for (int j = 0; j < 3; ++j) {
            point[3 + j] = rpy_slerp[j];
        }
        trajectory.push_back(std::move(point));
    }

    return finalizeTrajectory(std::move(trajectory), num_points, total_time, dt);
}

} // namespace gazebo
