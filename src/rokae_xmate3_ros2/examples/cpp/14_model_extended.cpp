/**
 * @file 14_model_extended.cpp
 * @brief 外部 xmatemodel_er3_er7p 的 xMate3 模型接口整理版
 */

#include <array>

#include "rokae/model.h"
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

namespace {

template <size_t N>
std::array<double, N> degToRad(const std::array<double, N> &deg) {
  std::array<double, N> rad{};
  for (size_t i = 0; i < N; ++i) {
    rad[i] = deg[i] * kPi / 180.0;
  }
  return rad;
}

}  // namespace

int main() {
  printHeader("示例 14: xMate3 模型扩展示例", "xmatemodel_er3_er7p 对齐版");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }

  auto model = robot.model();

  const std::array<double, 6> zeros{};
  const auto joint_pos_in = degToRad<6>({-20.0, 37.0, 70.0, 0.0, 71.0, -19.0});
  const auto joint_vel_in = std::array<double, 6>{0.3, 0.2, 0.5, 0.4, 0.1, 0.1};
  const auto joint_acc_in = std::array<double, 6>{1.3, 3.1, 4.1, 1.5, 1.6, 4.1};
  const auto joint_init = degToRad<6>({-21.0, 38.0, 71.0, 0.0, 70.0, -20.0});
  const std::array<double, 16> flange_to_ee{1, 0, 0, 0,
                                            0, 1, 0, 0,
                                            0, 0, 1, 0,
                                            0, 0, 0, 1};
  const std::array<double, 16> ee_to_k{1, 0, 0, 0,
                                       0, 1, 0, 0,
                                       0, 0, 1, 0,
                                       0, 0, 0, 1};

  printSection("1 雅可比与动力学");
  printArray("jacobian", model.jacobian(zeros), 4);
  printArray("jacobian(end effector)", model.jacobian(zeros, flange_to_ee, ee_to_k, SegmentFrame::endEffector), 4);

  std::array<double, 6> torque_full{};
  std::array<double, 6> torque_inertia{};
  std::array<double, 6> torque_coriolis{};
  std::array<double, 6> torque_gravity{};
  model.getTorqueNoFriction(zeros, zeros, zeros, torque_full, torque_inertia, torque_coriolis, torque_gravity);
  printArray("torque full(no friction)", torque_full, 4, " Nm");
  printArray("torque inertia", torque_inertia, 4, " Nm");
  printArray("torque coriolis", torque_coriolis, 4, " Nm");
  printArray("torque gravity", torque_gravity, 4, " Nm");
  printArray("torque inertia only", model.getTorque(zeros, zeros, zeros, TorqueType::inertia), 4, " Nm");

  printSection("2 TCP、末端与速度加速度");
  const auto flange_pose = model.getCartPose(joint_pos_in);
  printArray("flange posture", flange_pose, 4);
  model.setTcpCoor(flange_to_ee, ee_to_k);
  printArray("end effector posture", model.getCartPose(joint_pos_in, SegmentFrame::endEffector), 4);
  const auto cart_vel = model.getCartVel(joint_pos_in, joint_vel_in);
  const auto cart_acc = model.getCartAcc(joint_pos_in, joint_vel_in, joint_acc_in);
  printArray("cartesian velocity", cart_vel, 4);
  printArray("cartesian acceleration", cart_acc, 4);
  printArray("joint velocity(recovered)", model.getJointVel(cart_vel, joint_pos_in), 4, " rad/s");
  printArray("joint acceleration(recovered)", model.getJointAcc(cart_acc, joint_pos_in, joint_vel_in), 4, " rad/s^2");

  printSection("3 IK 校验与负载设置");
  std::array<double, 6> joint_out{};
  const double psi = -7.543 * kPi / 180.0;
  const auto ret = model.getJointPos(flange_pose, psi, joint_init, joint_out);
  os << "getJointPos ret = " << ret << std::endl;
  printArray("ik result", joint_out, 4, " rad");
  printArray("fk(ik result)", model.getCartPose(joint_out), 4);
  model.setLoad(2.0, {0.1, 0.1, 0.1}, {3.0, 2.0, 5.0});
  os << "model load updated" << std::endl;

  printSection("4 断开连接");
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
