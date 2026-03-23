#ifndef ROKAE_XMATE3_ROS2_MODEL_H
#define ROKAE_XMATE3_ROS2_MODEL_H

#include <array>
#include <system_error>

#include "rokae_xmate3_ros2/robot.hpp"
#include "rokae_xmate3_ros2/gazebo/approximate_model.hpp"
#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"

namespace rokae::ros2 {

class XMateModel {
public:
    explicit XMateModel(xMateRobot& robot)
        : robot_(robot) {}

    void setTcpCoor(const rokae::Toolset& toolset) {
        tcp_ = toolset;
    }

    std::array<double, 6> getCartPose(std::error_code& ec) {
        const auto q = robot_.jointPos(ec);
        if (ec) {
            return {};
        }
        return facade().cartPose(q);
    }

    std::array<double, 6> getCartVel(std::error_code& ec) {
        const auto q = robot_.jointPos(ec);
        if (ec) {
            return {};
        }
        const auto dq = robot_.jointVel(ec);
        if (ec) {
            return {};
        }
        return facade().cartVelocity(q, dq);
    }

    std::array<double, 6> getCartAcc(std::error_code& ec) {
        const auto q = robot_.jointPos(ec);
        if (ec) {
            return {};
        }
        const auto dq = robot_.jointVel(ec);
        if (ec) {
            return {};
        }
        const std::array<double, 6> ddq{};
        return facade().cartAcceleration(q, dq, ddq);
    }

    std::array<double, 6> getJointPos(std::error_code& ec) {
        return robot_.jointPos(ec);
    }

    std::array<double, 6> getJointVel(std::error_code& ec) {
        return robot_.jointVel(ec);
    }

    std::array<double, 6> getJointAcc(std::error_code& ec) {
        const auto q = robot_.jointPos(ec);
        if (ec) {
            return {};
        }
        const auto cart_acc = getCartAcc(ec);
        if (ec) {
            return {};
        }
        return facade().jointAcceleration(cart_acc, q);
    }

    std::array<double, 6> getTorque(std::error_code& ec) {
        const auto q = robot_.jointPos(ec);
        if (ec) {
            return {};
        }
        const auto dq = robot_.jointVel(ec);
        if (ec) {
            return {};
        }
        const std::array<double, 6> qdd{};
        const std::array<double, 6> wrench{};
        return facade().dynamics(q, dq, qdd, wrench).full;
    }

    Eigen::MatrixXd jacobian(std::error_code& ec) {
        const auto q = robot_.jointPos(ec);
        if (ec) {
            return Eigen::MatrixXd::Zero(6, 6);
        }
        return facade().jacobian(q);
    }

private:
    [[nodiscard]] rokae_xmate3_ros2::gazebo_model::ModelFacade facade() {
        return rokae_xmate3_ros2::gazebo_model::configuredModelFacade(
            kinematics_, tcp_.tool_pose, {tcp_.load.mass, tcp_.load.cog});
    }

    xMateRobot& robot_;
    gazebo::xMate3Kinematics kinematics_;
    rokae::Toolset tcp_{};
};

inline XMateModel model(xMateRobot& robot) {
    return XMateModel(robot);
}

}  // namespace rokae::ros2

#endif  // ROKAE_XMATE3_ROS2_MODEL_H
