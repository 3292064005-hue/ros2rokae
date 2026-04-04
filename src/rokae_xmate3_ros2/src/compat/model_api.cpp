#include "compat/internal/compat_shared.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "rokae/model.h"
#include "rokae/utility.h"
#include "rokae_xmate3_ros2/gazebo/model_facade.hpp"

namespace rokae {
namespace {

template <typename Range>
bool allFinite(const Range &values) {
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

}  // namespace

struct xMateModel<6>::Impl {
  std::shared_ptr<detail::CompatRobotHandle> handle;
  gazebo::xMate3Kinematics kinematics;
  Load load{};
  std::array<double, 16> f_t_ee{detail::identity_matrix16()};
  std::array<double, 16> ee_t_k{detail::identity_matrix16()};

  [[nodiscard]] rokae_xmate3_ros2::gazebo_model::ModelFacade facade() const {
    std::array<double, 6> tool_pose{};
    std::array<double, 6> posture{};
    Utils::transArrayToPosture(f_t_ee, posture);
    tool_pose = posture;
    return rokae_xmate3_ros2::gazebo_model::makeModelFacade(
        const_cast<gazebo::xMate3Kinematics &>(kinematics), tool_pose, {load.mass, load.cog});
  }

  [[nodiscard]] std::array<double, 6> current_seed() const {
    if (handle && handle->backend) {
      error_code ec;
      const auto joints = handle->backend->jointPos(ec);
      if (!ec) {
        return joints;
      }
    }
    return {};
  }
};

xMateModel<6>::xMateModel()
    : impl_(std::make_shared<Impl>()) {}

xMateModel<6>::xMateModel(std::shared_ptr<detail::CompatRobotHandle> handle)
    : impl_(std::make_shared<Impl>()) {
  impl_->handle = std::move(handle);
  if (impl_->handle) {
    std::lock_guard<std::mutex> lock(impl_->handle->mutex);
    impl_->load = impl_->handle->model_load_cache;
    impl_->f_t_ee = impl_->handle->model_f_t_ee;
    impl_->ee_t_k = impl_->handle->model_ee_t_k;
  }
}

xMateModel<6>::~xMateModel() = default;

CartesianPosition xMateModel<6>::calcFk(const std::array<double, 6> &joints, error_code &ec) const {
  if (!allFinite(joints)) {
    ec = make_error_code(SdkError::fk_invalid_input);
    return CartesianPosition{};
  }
  ec.clear();
  return CartesianPosition(impl_->facade().cartPose(joints));
}

CartesianPosition xMateModel<6>::calcFk(const JointPosition &joints, error_code &ec) const {
  std::array<double, 6> q{};
  for (std::size_t i = 0; i < 6 && i < joints.joints.size(); ++i) {
    q[i] = joints.joints[i];
  }
  return calcFk(q, ec);
}

std::array<double, 6> xMateModel<6>::calcIk(const CartesianPosition &posture, error_code &ec) const {
  const std::array<double, 6> posture_values{posture.x, posture.y, posture.z, posture.rx, posture.ry, posture.rz};
  if (!allFinite(posture_values)) {
    ec = make_error_code(SdkError::model_invalid_input);
    return {};
  }
  const auto seed = impl_->current_seed();
  const auto solution = impl_->kinematics.inverseKinematics(
      {posture.x, posture.y, posture.z, posture.rx, posture.ry, posture.rz},
      std::vector<double>(seed.begin(), seed.end()));
  if (solution.size() != 6) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return {};
  }
  ec.clear();
  std::array<double, 6> out{};
  std::copy_n(solution.begin(), 6, out.begin());
  return out;
}

JointPosition xMateModel<6>::calcIkAsJointPosition(const CartesianPosition &posture, error_code &ec) const {
  JointPosition out;
  const auto joints = calcIk(posture, ec);
  out.joints.assign(joints.begin(), joints.end());
  return out;
}

std::array<double, 16> xMateModel<6>::getCartPose(const std::array<double, 6> &jntPos, SegmentFrame nr) const {
  auto pose = impl_->facade().cartPose(jntPos);
  if (nr == SegmentFrame::flange) {
    std::array<double, 16> matrix{};
    Utils::postureToTransArray(pose, matrix);
    return matrix;
  }
  std::array<double, 16> matrix{};
  Utils::postureToTransArray(pose, matrix);
  return matrix;
}

std::array<double, 6> xMateModel<6>::getCartVel(const std::array<double, 6> &jntPos,
                                                 const std::array<double, 6> &jntVel,
                                                 SegmentFrame) const {
  return impl_->facade().cartVelocity(jntPos, jntVel);
}

std::array<double, 6> xMateModel<6>::getCartAcc(const std::array<double, 6> &jntPos,
                                                 const std::array<double, 6> &jntVel,
                                                 const std::array<double, 6> &jntAcc,
                                                 SegmentFrame) const {
  return impl_->facade().cartAcceleration(jntPos, jntVel, jntAcc);
}

std::array<double, 6> xMateModel<6>::getJointAcc(const std::array<double, 6> &cartAcc,
                                                  const std::array<double, 6> &jntPos,
                                                  const std::array<double, 6> &) const {
  return impl_->facade().jointAcceleration(cartAcc, jntPos);
}

int xMateModel<6>::getJointPos(const std::array<double, 16> &cartPos,
                               double,
                               const std::array<double, 6> &jntInit,
                               std::array<double, 6> &jntPos) const {
  if (!allFinite(cartPos) || !allFinite(jntInit)) {
    return -1;
  }
  std::array<double, 6> posture{};
  Utils::transArrayToPosture(cartPos, posture);
  const auto solution = impl_->kinematics.inverseKinematics(
      std::vector<double>(posture.begin(), posture.end()),
      std::vector<double>(jntInit.begin(), jntInit.end()));
  if (solution.size() != 6) {
    return -1;
  }
  std::copy_n(solution.begin(), 6, jntPos.begin());
  return 0;
}

std::array<double, 6> xMateModel<6>::getJointVel(const std::array<double, 6> &cartVel,
                                                  const std::array<double, 6> &jntPos) const {
  const auto jac = impl_->facade().jacobian(jntPos);
  const auto pinv = jac.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::Matrix<double, 6, 1> v;
  for (std::size_t i = 0; i < 6; ++i) v(static_cast<Eigen::Index>(i)) = cartVel[i];
  const auto qd = pinv * v;
  std::array<double, 6> out{};
  for (std::size_t i = 0; i < 6; ++i) out[i] = qd(static_cast<Eigen::Index>(i));
  return out;
}

std::array<double, 6> xMateModel<6>::getTorque(const std::array<double, 6> &jntPos,
                                                const std::array<double, 6> &jntVel,
                                                const std::array<double, 6> &jntAcc,
                                                TorqueType torque_type) const {
  const auto dynamics = impl_->facade().dynamics(jntPos, jntVel, jntAcc, {});
  switch (torque_type) {
    case TorqueType::full: return dynamics.full;
    case TorqueType::inertia: return dynamics.inertia;
    case TorqueType::coriolis: return dynamics.coriolis;
    case TorqueType::friction: {
      std::array<double, 6> friction{};
      for (std::size_t i = 0; i < 6; ++i) {
        friction[i] = dynamics.full[i] - dynamics.inertia[i] - dynamics.coriolis[i] - dynamics.gravity[i];
      }
      return friction;
    }
    case TorqueType::gravity: return dynamics.gravity;
  }
  return {};
}

void xMateModel<6>::getTorqueNoFriction(const std::array<double, 6> &jntPos,
                                        const std::array<double, 6> &jntVel,
                                        const std::array<double, 6> &jntAcc,
                                        std::array<double, 6> &trq_full,
                                        std::array<double, 6> &trq_inertia,
                                        std::array<double, 6> &trq_coriolis,
                                        std::array<double, 6> &trq_gravity) const {
  const auto dynamics = impl_->facade().dynamics(jntPos, jntVel, jntAcc, {});
  trq_full = dynamics.full;
  trq_inertia = dynamics.inertia;
  trq_coriolis = dynamics.coriolis;
  trq_gravity = dynamics.gravity;
}

void xMateModel<6>::getTorqueWithFriction(const std::array<double, 6> &jntPos,
                                          const std::array<double, 6> &jntVel,
                                          const std::array<double, 6> &jntAcc,
                                          std::array<double, 6> &trq_full,
                                          std::array<double, 6> &trq_inertia,
                                          std::array<double, 6> &trq_coriolis,
                                          std::array<double, 6> &trq_friction,
                                          std::array<double, 6> &trq_gravity) const {
  getTorqueNoFriction(jntPos, jntVel, jntAcc, trq_full, trq_inertia, trq_coriolis, trq_gravity);
  for (std::size_t i = 0; i < 6; ++i) {
    trq_friction[i] = trq_full[i] - trq_inertia[i] - trq_coriolis[i] - trq_gravity[i];
  }
}

std::array<double, 36> xMateModel<6>::jacobian(const std::array<double, 6> &jntPos, SegmentFrame) const {
  const auto jac = impl_->facade().jacobian(jntPos);
  std::array<double, 36> out{};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t col = 0; col < 6; ++col) {
      out[row * 6 + col] = jac(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col));
    }
  }
  return out;
}

std::array<double, 36> xMateModel<6>::jacobian(const std::array<double, 6> &jntPos,
                                                const std::array<double, 16> &f_t_ee,
                                                const std::array<double, 16> &ee_t_k,
                                                SegmentFrame nr) const {
  auto copy = *this;
  copy.setTcpCoor(f_t_ee, ee_t_k);
  return copy.jacobian(jntPos, nr);
}

void xMateModel<6>::setLoad(double mass,
                            const std::array<double, 3> &cog,
                            const std::array<double, 3> &inertia) noexcept {
  setLoad(Load{mass, cog, inertia});
}

void xMateModel<6>::setLoad(const Load &load) noexcept {
  impl_->load = load;
  if (impl_->handle) {
    std::lock_guard<std::mutex> lock(impl_->handle->mutex);
    impl_->handle->model_load_cache = load;
  }
}

void xMateModel<6>::setTcpCoor(const std::array<double, 16> &f_t_ee,
                               const std::array<double, 16> &ee_t_k) noexcept {
  impl_->f_t_ee = f_t_ee;
  impl_->ee_t_k = ee_t_k;
  if (impl_->handle) {
    std::lock_guard<std::mutex> lock(impl_->handle->mutex);
    impl_->handle->model_f_t_ee = f_t_ee;
    impl_->handle->model_ee_t_k = ee_t_k;
  }
}

}  // namespace rokae
