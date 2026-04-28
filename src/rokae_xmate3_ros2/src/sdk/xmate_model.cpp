#include "rokae_xmate3_ros2/model.hpp"

#include <utility>

#include "rokae_xmate3_ros2/model_facade.hpp"
#include "runtime/kinematics_provider.hpp"

namespace rokae::ros2 {
namespace {

[[nodiscard]] rokae_xmate3_ros2::model_facade::ModelLoadContext toLoadContext(const rokae::Toolset& toolset) {
    return {toolset.load.mass, toolset.load.cog};
}

}  // namespace

struct XMateModel::Impl {
    rokae_xmate3_ros2::kinematics::OwnedGazeboProvider provider{};
    rokae::Toolset tcp{};

    [[nodiscard]] rokae_xmate3_ros2::model_facade::ModelFacade facade() {
        return rokae_xmate3_ros2::model_facade::makeModelFacade(provider, tcp.tool_pose, toLoadContext(tcp));
    }
};

XMateModel::XMateModel(xMateRobot& robot)
    : robot_(&robot), impl_(std::make_unique<Impl>()) {}

XMateModel::~XMateModel() = default;
XMateModel::XMateModel(XMateModel&&) noexcept = default;
XMateModel& XMateModel::operator=(XMateModel&&) noexcept = default;

void XMateModel::setTcpCoor(const rokae::Toolset& toolset) {
    impl_->tcp = toolset;
}

std::array<double, 6> XMateModel::getCartPose(std::error_code& ec) {
    const auto q = robot_->jointPos(ec);
    if (ec) {
        return {};
    }
    return impl_->facade().cartPose(q);
}

std::array<double, 6> XMateModel::getCartVel(std::error_code& ec) {
    const auto q = robot_->jointPos(ec);
    if (ec) {
        return {};
    }
    const auto dq = robot_->jointVel(ec);
    if (ec) {
        return {};
    }
    return impl_->facade().cartVelocity(q, dq);
}

std::array<double, 6> XMateModel::getCartAcc(std::error_code& ec) {
    const auto q = robot_->jointPos(ec);
    if (ec) {
        return {};
    }
    const auto dq = robot_->jointVel(ec);
    if (ec) {
        return {};
    }
    const std::array<double, 6> ddq{};
    return impl_->facade().cartAcceleration(q, dq, ddq);
}

std::array<double, 6> XMateModel::getJointPos(std::error_code& ec) {
    return robot_->jointPos(ec);
}

std::array<double, 6> XMateModel::getJointVel(std::error_code& ec) {
    return robot_->jointVel(ec);
}

std::array<double, 6> XMateModel::getJointAcc(std::error_code& ec) {
    const auto q = robot_->jointPos(ec);
    if (ec) {
        return {};
    }
    const auto cart_acc = getCartAcc(ec);
    if (ec) {
        return {};
    }
    return impl_->facade().jointAcceleration(cart_acc, q);
}

std::array<double, 6> XMateModel::getTorque(std::error_code& ec) {
    const auto q = robot_->jointPos(ec);
    if (ec) {
        return {};
    }
    const auto dq = robot_->jointVel(ec);
    if (ec) {
        return {};
    }
    const std::array<double, 6> qdd{};
    const std::array<double, 6> wrench{};
    return impl_->facade().inverseDynamics(q, dq, qdd, wrench);
}

Eigen::MatrixXd XMateModel::jacobian(std::error_code& ec) {
    const auto q = robot_->jointPos(ec);
    if (ec) {
        return Eigen::MatrixXd::Zero(6, 6);
    }
    return impl_->facade().jacobian(q);
}

}  // namespace rokae::ros2
