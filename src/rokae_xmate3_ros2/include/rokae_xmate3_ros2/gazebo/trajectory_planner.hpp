/**
 * @file trajectory_planner.hpp
 * @brief 轨迹规划器
 */

#ifndef ROKAE_XMATE3_GAZEBO_TRAJECTORY_PLANNER_HPP
#define ROKAE_XMATE3_GAZEBO_TRAJECTORY_PLANNER_HPP

#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace gazebo {

using namespace Eigen;

struct TrajectorySamples {
    std::vector<std::vector<double>> points;
    double sample_dt = 0.001;
    double total_time = 0.0;

    [[nodiscard]] bool empty() const noexcept { return points.empty(); }
};

/**
 * @brief 轨迹规划器
 */
class TrajectoryPlanner {
public:
    /**
     * @brief 关节空间直线插补 (MoveJ/MoveAbsJ)
     */
    static TrajectorySamples planJointMove(
        const std::vector<double>& start,
        const std::vector<double>& end,
        double speed_mm_per_s,
        double dt = 0.001);

    /**
     * @brief 笛卡尔空间直线插补 (MoveL)
     */
    static TrajectorySamples planCartesianLine(
        const std::vector<double>& start,
        const std::vector<double>& end,
        double speed_mm_per_s,
        double dt = 0.001);

    /**
     * @brief 圆弧插补 (MoveC)
     */
    static TrajectorySamples planCircularArc(
        const std::vector<double>& start,
        const std::vector<double>& aux,
        const std::vector<double>& end,
        double speed_mm_per_s,
        double dt = 0.001);

    /**
     * @brief 连续圆弧运动 (MoveCF)
     */
    static TrajectorySamples planCircularContinuous(
        const std::vector<double>& start,
        const std::vector<double>& aux,
        const std::vector<double>& end,
        double angle,
        double speed_mm_per_s,
        double dt = 0.001);

    /**
     * @brief 螺旋线运动 (MoveSP)
     */
    static TrajectorySamples planSpiralMove(
        const std::vector<double>& start,
        const std::vector<double>& end,
        double radius,
        double radius_step,
        double angle,
        bool direction,
        double speed_mm_per_s,
        double dt = 0.001);

private:
    /**
     * @brief 计算S型曲线混合比例 (0-1)
     */
    static double computeBlendRatio(double t, double T);
};

} // namespace gazebo

#endif // ROKAE_XMATE3_GAZEBO_TRAJECTORY_PLANNER_HPP
