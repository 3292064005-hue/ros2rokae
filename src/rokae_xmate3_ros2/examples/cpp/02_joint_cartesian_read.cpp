/**
 * @file 02_joint_cartesian_read.cpp
 * @brief 官方 SDK 风格 - 读取关节、位姿和状态流数据
 */

#include <array>
#include <chrono>

#include "rokae/robot.h"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 2: 关节与笛卡尔数据读取", "官方 SDK 风格");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }

  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  if (reportError("setMotionControlMode", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("1 一次性读取当前状态");
  printArray("joint position", robot.jointPos(ec), 4, " rad");
  if (reportError("jointPos", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printArray("joint velocity", robot.jointVel(ec), 4, " rad/s");
  if (reportError("jointVel", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printArray("joint torque", robot.jointTorques(ec), 4, " Nm");
  if (reportError("jointTorques", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  const auto tcp_in_ref = robot.cartPosture(CoordinateType::endInRef, ec);
  if (reportError("cartPosture(endInRef)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printPose("tcp in ref", tcp_in_ref);

  const auto flange_in_base = robot.cartPosture(CoordinateType::flangeInBase, ec);
  if (reportError("cartPosture(flangeInBase)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printPose("flange in base", flange_in_base);

  printArray("posture(endInRef)", robot.posture(CoordinateType::endInRef, ec));
  if (reportError("posture", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printArray("base frame", robot.baseFrame(ec));
  if (reportError("baseFrame", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("2 读取机器人状态流快照");
  using namespace RtSupportedFields;
  robot.startReceiveRobotState(std::chrono::seconds(1), {tcpPose_m, tau_m, jointPos_m});

  for (int sample = 1; sample <= 3; ++sample) {
    std::array<double, 16> tcp_pose_m{};
    std::array<double, 6> joint_pos_m{};
    std::array<double, 6> tau_m_data{};

    robot.updateRobotState(std::chrono::seconds(1));
    os << "sample " << sample << ':' << std::endl;
    if (robot.getStateData(jointPos_m, joint_pos_m) == 0) {
      printArray("jointPos_m", joint_pos_m, 4, " rad");
    }
    if (robot.getStateData(tau_m, tau_m_data) == 0) {
      printArray("tau_m", tau_m_data, 4, " Nm");
    }
    if (robot.getStateData(tcpPose_m, tcp_pose_m) == 0) {
      printArray("tcpPose_m", tcp_pose_m, 4);
    }
  }

  robot.stopReceiveRobotState();
  os << "state stream stopped" << std::endl;

  printSection("3 断开连接");
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
