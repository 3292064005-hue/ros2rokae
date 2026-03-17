/**
 * @file 03_kinematics.cpp
 * @brief 官方 SDK 风格 - 运动学与模型接口
 */

#include <array>

#include "rokae/model.h"
#include "rokae/robot.h"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 3: 运动学与模型接口", "官方 SDK 风格");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }

  auto model = robot.model();

  printSection("1 当前姿态与正运动学");
  const auto q_current = robot.jointPos(ec);
  if (reportError("jointPos", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printArray("joint position", q_current, 4, " rad");

  const auto tcp_xyzabc = robot.posture(CoordinateType::endInRef, ec);
  if (reportError("posture(endInRef)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printArray("tcp xyzabc", tcp_xyzabc, 4);

  const auto fk_current = model.calcFk(q_current, ec);
  if (reportError("model.calcFk", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printPose("fk(current)", fk_current);
  printArray("getCartPose(flange)", model.getCartPose(q_current), 4);

  printSection("2 逆运动学与结果校验");
  CartesianPosition target_pose({0.40, 0.00, 0.50, kPi, 0.0, 0.0});
  target_pose.confData = {-1, 1, -1, 0, 1, 0, 0, 2};
  printPose("ik target", target_pose);
  printVector("ik confData", target_pose.confData, 0);

  const auto q_target = model.calcIk(target_pose, ec);
  if (reportError("model.calcIk", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printArray("ik result", q_target, 4, " rad");

  const auto fk_verify = model.calcFk(q_target, ec);
  if (reportError("model.calcFk(ik)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printPose("fk(ik result)", fk_verify);

  const auto cart_target = model.getCartPose(q_target);
  printArray("getCartPose(ik result)", cart_target, 4);

  std::array<double, 6> q_recovered{};
  const auto ik_ret = model.getJointPos(cart_target, 0.0, q_target, q_recovered);
  os << "getJointPos ret = " << ik_ret << std::endl;
  if (ik_ret == 0) {
    printArray("getJointPos result", q_recovered, 4, " rad");
  }

  printSection("3 雅可比与速度加速度映射");
  const auto jac = model.jacobian(q_current);
  std::array<double, 6> jac_row0{};
  std::copy_n(jac.begin(), 6, jac_row0.begin());
  printArray("jacobian row0", jac_row0, 4);

  const std::array<double, 6> dq_sample{0.10, 0.05, 0.02, 0.00, 0.03, 0.01};
  const std::array<double, 6> ddq_sample{0.20, 0.10, 0.05, 0.00, 0.02, 0.01};
  printArray("cartesian velocity", model.getCartVel(q_current, dq_sample), 4);
  printArray("cartesian acceleration", model.getCartAcc(q_current, dq_sample, ddq_sample), 4);
  printArray("joint velocity(recovered)", model.getJointVel(model.getCartVel(q_current, dq_sample), q_current), 4, " rad/s");
  printArray("joint acceleration(recovered)", model.getJointAcc(model.getCartAcc(q_current, dq_sample, ddq_sample), q_current, dq_sample), 4, " rad/s^2");

  printSection("4 动力学项");
  std::array<double, 6> torque_full{};
  std::array<double, 6> torque_inertia{};
  std::array<double, 6> torque_coriolis{};
  std::array<double, 6> torque_gravity{};
  model.getTorqueNoFriction(q_current, dq_sample, ddq_sample, torque_full, torque_inertia, torque_coriolis, torque_gravity);
  printArray("torque full(no friction)", torque_full, 4, " Nm");
  printArray("torque inertia", torque_inertia, 4, " Nm");
  printArray("torque coriolis", torque_coriolis, 4, " Nm");
  printArray("torque gravity", torque_gravity, 4, " Nm");
  printArray("torque friction", model.getTorque(q_current, dq_sample, ddq_sample, TorqueType::friction), 4, " Nm");

  printSection("5 断开连接");
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
