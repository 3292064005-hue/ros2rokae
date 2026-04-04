#ifndef ROKAE_MODEL_H
#define ROKAE_MODEL_H

#include <array>
#include <memory>
#include <system_error>

#include "rokae/base.h"
#include "rokae/data_types.h"

namespace rokae {

namespace detail {
struct CompatRobotHandle;
}

class BaseCobot;
template <unsigned short DoF>
class Cobot;

/**
 * @brief Segment selector used by the xMate6 compatibility model.
 * @details The compatibility lane only promises xMate 6-axis semantics. Values that are
 *          not meaningful for the current simulation backend are accepted but may be mapped
 *          to the nearest supported segment frame.
 */
enum class SegmentFrame : unsigned {
  joint1 = 1,
  joint2 = 2,
  joint3 = 3,
  joint4 = 4,
  joint5 = 5,
  joint6 = 6,
  flange = 8,
  endEffector = 9,
  stiffness = 10,
};

template <unsigned short DoF>
class xMateModel;

template <unsigned short DoF>
using XMateModel = xMateModel<DoF>;

/**
 * @brief xMate 6-axis model facade with official-SDK-like surface.
 * @details The implementation is compiled into the compatibility library and delegates to the
 *          project kinematics/model backend. When constructed from a robot instance the model
 *          shares the robot's tool/load context; default construction creates a standalone model.
 */
template <>
class XCORE_API xMateModel<6> {
 public:
  xMateModel();
  xMateModel(const xMateModel &) = default;
  xMateModel(xMateModel &&) noexcept = default;
  xMateModel &operator=(const xMateModel &) = default;
  xMateModel &operator=(xMateModel &&) noexcept = default;
  ~xMateModel();

  /**
   * @brief Compute forward kinematics and return TCP posture in end-in-ref convention.
   * @param joints Joint positions in radians.
   * @param ec Output error code. Cleared on success.
   * @return Cartesian pose expressed as {x,y,z,rx,ry,rz} plus compatibility metadata.
   * @throws No exception. Transport/model errors are reported through @p ec.
   * @note When the current request context is invalid the function returns a zero pose and sets @p ec.
   */
  CartesianPosition calcFk(const std::array<double, 6> &joints, error_code &ec) const;
  CartesianPosition calcFk(const JointPosition &joints, error_code &ec) const;

  /**
   * @brief Compute inverse kinematics for the requested Cartesian posture.
   * @param posture Target Cartesian posture in end-in-ref convention.
   * @param ec Output error code. Cleared on success.
   * @return Joint solution in radians.
   * @throws No exception. Kinematic failures are reported through @p ec.
   * @note The compatibility model selects the nearest valid branch using the current joint state
   *       when bound to a robot, otherwise it uses a zero seed.
   */
  std::array<double, 6> calcIk(const CartesianPosition &posture, error_code &ec) const;
  JointPosition calcIkAsJointPosition(const CartesianPosition &posture, error_code &ec) const;

  std::array<double, 16> getCartPose(const std::array<double, 6> &jntPos,
                                     SegmentFrame nr = SegmentFrame::flange) const;
  std::array<double, 6> getCartVel(const std::array<double, 6> &jntPos,
                                   const std::array<double, 6> &jntVel,
                                   SegmentFrame nr = SegmentFrame::flange) const;
  std::array<double, 6> getCartAcc(const std::array<double, 6> &jntPos,
                                   const std::array<double, 6> &jntVel,
                                   const std::array<double, 6> &jntAcc,
                                   SegmentFrame nr = SegmentFrame::flange) const;
  std::array<double, 6> getJointAcc(const std::array<double, 6> &cartAcc,
                                    const std::array<double, 6> &jntPos,
                                    const std::array<double, 6> &jntVel) const;
  int getJointPos(const std::array<double, 16> &cartPos,
                  double elbow,
                  const std::array<double, 6> &jntInit,
                  std::array<double, 6> &jntPos) const;
  std::array<double, 6> getJointVel(const std::array<double, 6> &cartVel,
                                    const std::array<double, 6> &jntPos) const;
  std::array<double, 6> getTorque(const std::array<double, 6> &jntPos,
                                  const std::array<double, 6> &jntVel,
                                  const std::array<double, 6> &jntAcc,
                                  TorqueType torque_type) const;
  void getTorqueNoFriction(const std::array<double, 6> &jntPos,
                           const std::array<double, 6> &jntVel,
                           const std::array<double, 6> &jntAcc,
                           std::array<double, 6> &trq_full,
                           std::array<double, 6> &trq_inertia,
                           std::array<double, 6> &trq_coriolis,
                           std::array<double, 6> &trq_gravity) const;
  void getTorqueWithFriction(const std::array<double, 6> &jntPos,
                             const std::array<double, 6> &jntVel,
                             const std::array<double, 6> &jntAcc,
                             std::array<double, 6> &trq_full,
                             std::array<double, 6> &trq_inertia,
                             std::array<double, 6> &trq_coriolis,
                             std::array<double, 6> &trq_friction,
                             std::array<double, 6> &trq_gravity) const;
  std::array<double, 36> jacobian(const std::array<double, 6> &jntPos,
                                  SegmentFrame nr = SegmentFrame::flange) const;
  std::array<double, 36> jacobian(const std::array<double, 6> &jntPos,
                                  const std::array<double, 16> &f_t_ee,
                                  const std::array<double, 16> &ee_t_k,
                                  SegmentFrame nr = SegmentFrame::flange) const;

  /**
   * @brief Update model-only payload parameters.
   * @param mass Payload mass in kilograms.
   * @param cog Center of gravity in metres.
   * @param inertia Diagonal inertia terms in kg·m^2.
   * @throws No exception.
   * @note This only affects compatibility-model calculations; it does not push the payload to the robot controller.
   */
  void setLoad(double mass,
               const std::array<double, 3> &cog,
               const std::array<double, 3> &inertia = {}) noexcept;
  void setLoad(const Load &load) noexcept;

  /**
   * @brief Configure TCP and stiffness transforms used by model-only computations.
   * @param f_t_ee Homogeneous transform from flange to TCP.
   * @param ee_t_k Homogeneous transform from TCP to stiffness frame.
   * @throws No exception.
   * @note The transforms are kept in the shared compatibility model context and reused by RT helpers.
   */
  void setTcpCoor(const std::array<double, 16> &f_t_ee,
                  const std::array<double, 16> &ee_t_k = {}) noexcept;

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;

  explicit xMateModel(std::shared_ptr<detail::CompatRobotHandle> handle);
  friend class BaseCobot;
  friend class Cobot<6>;
};

}  // namespace rokae

#endif  // ROKAE_MODEL_H
