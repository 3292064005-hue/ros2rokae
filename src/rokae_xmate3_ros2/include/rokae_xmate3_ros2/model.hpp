#ifndef ROKAE_XMATE3_ROS2_MODEL_H
#define ROKAE_XMATE3_ROS2_MODEL_H

#include <array>
#include <vector>
#include <system_error>

#include "rokae_xmate3_ros2/robot.hpp"
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
        auto pose = robot_.cartPosture(rokae::CoordinateType::flangeInBase, ec);
        return {pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz};
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
        std::vector<double> qv(q.begin(), q.end());
        const auto J = kinematics_.computeJacobian(qv);
        Eigen::Matrix<double, 6, 1> dq_vec;
        for (size_t i = 0; i < 6; ++i) {
            dq_vec(i) = dq[i];
        }
        Eigen::Matrix<double, 6, 1> cart = J * dq_vec;
        std::array<double, 6> out{};
        for (size_t i = 0; i < 6; ++i) {
            out[i] = cart(i);
        }
        return out;
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
        std::vector<double> qv(q.begin(), q.end());
        const auto J = kinematics_.computeJacobian(qv);
        Eigen::Matrix<double, 6, 1> dq_vec = Eigen::Matrix<double, 6, 1>::Zero();
        for (size_t i = 0; i < 6; ++i) {
            dq_vec(i) = dq[i];
        }
        const Eigen::Matrix<double, 6, 1> ddq_vec = J.completeOrthogonalDecomposition().pseudoInverse() * (J * dq_vec);
        const Eigen::Matrix<double, 6, 1> cart_acc = J * ddq_vec;
        std::array<double, 6> out{};
        for (size_t i = 0; i < 6; ++i) {
            out[i] = cart_acc(i);
        }
        return out;
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
        std::vector<double> qv(q.begin(), q.end());
        const auto J = kinematics_.computeJacobian(qv);
        Eigen::Matrix<double, 6, 1> cart_acc_vec;
        for (size_t i = 0; i < 6; ++i) {
            cart_acc_vec(i) = cart_acc[i];
        }
        const Eigen::Matrix<double, 6, 1> joint_acc = J.completeOrthogonalDecomposition().pseudoInverse() * cart_acc_vec;
        std::array<double, 6> out{};
        for (size_t i = 0; i < 6; ++i) {
            out[i] = joint_acc(i);
        }
        return out;
    }

    std::array<double, 6> getTorque(std::error_code& ec) {
        return robot_.jointTorque(ec);
    }

    Eigen::MatrixXd jacobian(std::error_code& ec) {
        const auto q = robot_.jointPos(ec);
        std::vector<double> qv(q.begin(), q.end());
        return kinematics_.computeJacobian(qv);
    }

private:
    xMateRobot& robot_;
    gazebo::xMate3Kinematics kinematics_;
    rokae::Toolset tcp_{};
};

inline XMateModel model(xMateRobot& robot) {
    return XMateModel(robot);
}

}  // namespace rokae::ros2

#endif  // ROKAE_XMATE3_ROS2_MODEL_H
