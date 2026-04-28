#ifndef ROKAE_XMATE3_ROS2_MOTION_GENERATORS_H
#define ROKAE_XMATE3_ROS2_MOTION_GENERATORS_H

#include <array>
#include <vector>
#include <system_error>

#include "rokae_xmate3_ros2/robot.hpp"

namespace rokae::ros2 {

class JointMotionGenerator {
public:
    JointMotionGenerator(int speed, const std::array<double, 6>& target)
        : speed_(speed), target_(target) {}

    bool execute(xMateRobot& robot, std::error_code& ec) const {
        rokae::MoveAbsJCommand cmd;
        cmd.target.joints.assign(target_.begin(), target_.end());
        cmd.speed = speed_;
        cmd.zone = 0;
        robot.moveReset(ec);
        if (ec) return false;
        robot.moveAbsJ(cmd, ec);
        if (ec) return false;
        robot.moveStart(ec);
        return !ec;
    }

    std::vector<std::array<double, 6>> preview(xMateRobot& robot,
                                                const std::array<double, 6>& start,
                                                double& total_time,
                                                std::error_code& ec) const {
        std::vector<std::array<double, 6>> points;
        robot.generateSTrajectory(start, target_, points, total_time, ec);
        return points;
    }

private:
    int speed_ = 50;
    std::array<double, 6> target_{};
};

class CartMotionGenerator {
public:
    CartMotionGenerator(int speed, const rokae::CartesianPosition& target)
        : speed_(speed), target_(target) {}

    bool execute(xMateRobot& robot, std::error_code& ec) const {
        rokae::MoveLCommand cmd;
        cmd.target = target_;
        cmd.speed = speed_;
        cmd.zone = 0;
        robot.moveReset(ec);
        if (ec) return false;
        robot.moveL(cmd, ec);
        if (ec) return false;
        robot.moveStart(ec);
        return !ec;
    }

private:
    int speed_ = 50;
    rokae::CartesianPosition target_{};
};

class FollowPosition {
public:
    explicit FollowPosition(std::vector<std::array<double, 6>> points)
        : points_(std::move(points)) {}

    bool execute(xMateRobot& robot, std::error_code& ec) const {
        robot.moveReset(ec);
        if (ec) return false;
        for (const auto& point : points_) {
            rokae::MoveAbsJCommand cmd;
            cmd.target.joints.assign(point.begin(), point.end());
            cmd.speed = 20;
            cmd.zone = 0;
            robot.moveAbsJ(cmd, ec);
            if (ec) return false;
        }
        robot.moveStart(ec);
        return !ec;
    }

private:
    std::vector<std::array<double, 6>> points_;
};

}  // namespace rokae::ros2

#endif  // ROKAE_XMATE3_ROS2_MOTION_GENERATORS_H
