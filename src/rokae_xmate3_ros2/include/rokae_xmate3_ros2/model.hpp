#ifndef ROKAE_XMATE3_ROS2_MODEL_H
#define ROKAE_XMATE3_ROS2_MODEL_H

#include <array>
#include <memory>
#include <system_error>

#include <Eigen/Dense>

#include "rokae_xmate3_ros2/robot.hpp"

namespace rokae::ros2 {

/**
 * @brief Public xMateER3 model helper for the ROS2 compatibility SDK surface.
 *
 * Boundary behavior:
 * - keeps the public header independent of Gazebo concrete provider types;
 * - owns the simulation-grade provider through an opaque implementation in the compiled SDK;
 * - returns zero/empty values when robot state queries report an error, preserving the existing SDK error-code style;
 * - does not claim hardware-model parity or expose a future hardware backend contract.
 *
 * @param robot Robot facade used as the source of current joint position and velocity.
 * @throws std::bad_alloc if the opaque model implementation cannot be allocated.
 */
class XMateModel {
public:
    explicit XMateModel(xMateRobot& robot);
    ~XMateModel();

    XMateModel(XMateModel&&) noexcept;
    XMateModel& operator=(XMateModel&&) noexcept;
    XMateModel(const XMateModel&) = delete;
    XMateModel& operator=(const XMateModel&) = delete;

    /**
     * @brief Set the TCP/tool context used by pose and dynamics helpers.
     * @param toolset Current SDK toolset; load and tool pose are consumed by the simulation-grade model facade.
     * @note This only affects local model calculations and does not push configuration to the robot/runtime.
     */
    void setTcpCoor(const rokae::Toolset& toolset);

    /** @brief Return the current TCP Cartesian pose derived from the robot joint state. */
    std::array<double, 6> getCartPose(std::error_code& ec);
    /** @brief Return the current TCP Cartesian velocity derived from joint position and velocity. */
    std::array<double, 6> getCartVel(std::error_code& ec);
    /** @brief Return a simulation-grade Cartesian acceleration estimate. */
    std::array<double, 6> getCartAcc(std::error_code& ec);
    /** @brief Return the current joint position from the robot facade. */
    std::array<double, 6> getJointPos(std::error_code& ec);
    /** @brief Return the current joint velocity from the robot facade. */
    std::array<double, 6> getJointVel(std::error_code& ec);
    /** @brief Return a simulation-grade joint acceleration estimate. */
    std::array<double, 6> getJointAcc(std::error_code& ec);
    /** @brief Return a simulation-grade inverse-dynamics torque estimate. */
    std::array<double, 6> getTorque(std::error_code& ec);
    /** @brief Return the current simulation-grade 6x6 Jacobian estimate. */
    Eigen::MatrixXd jacobian(std::error_code& ec);

private:
    struct Impl;

    xMateRobot* robot_ = nullptr;
    std::unique_ptr<Impl> impl_;
};

inline XMateModel model(xMateRobot& robot) {
    return XMateModel(robot);
}

}  // namespace rokae::ros2

#endif  // ROKAE_XMATE3_ROS2_MODEL_H
