/**
 * @file 25_rt_s_line.cpp
 * @brief 外部 joint_s_line/cartesian_s_line 的 xMate3 规划整理版
 */

#include <array>

#include <Eigen/Geometry>

#include "rokae/planner.h"
#include "rokae/robot.h"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 25: RT S-Line 规划", "joint_s_line / cartesian_s_line 对齐版");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }

  printSection("1 轴空间 S-Line 规划采样");
  std::array<double, 6> q_init{};
  const auto current = robot.jointPos(ec);
  if (reportError("jointPos", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  std::copy(current.begin(), current.end(), q_init.begin());
  const std::array<double, 6> q_goal{0.0, kPi / 6.0, kPi / 3.0, 0.0, kPi / 2.0, 0.0};
  JointMotionGenerator joint_s(0.5, q_goal);
  joint_s.calculateSynchronizedValues(q_init);
  for (double t = 0.0; t <= joint_s.getTime(); t += 0.05) {
    std::array<double, 6> delta{};
    joint_s.calculateDesiredValues(t, delta);
    printArray("joint delta", delta, 4, " rad");
  }

  printSection("2 笛卡尔 S-Line 规划采样");
  const auto pose = robot.cartPosture(CoordinateType::flangeInBase, ec);
  if (reportError("cartPosture(flangeInBase)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  const Eigen::Vector3d p_start(pose.x, pose.y, pose.z);
  const Eigen::Vector3d p_target(pose.x, pose.y, pose.z - 0.10);
  const double distance = (p_target - p_start).norm();
  CartMotionGenerator cart_s(0.05, distance);
  cart_s.calculateSynchronizedValues(0.0);
  for (double t = 0.0; t <= cart_s.getTime(); t += 0.05) {
    double delta_s = 0.0;
    cart_s.calculateDesiredValues(t, &delta_s);
    os << "cartesian delta_s: " << std::fixed << std::setprecision(4) << delta_s << std::endl;
  }

  printSection("3 断开连接");
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
