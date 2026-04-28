/**
 * @file 04_motion_basic.cpp
 * @brief 官方 SDK 风格 - 非实时关节运动
 */

#include <array>
#include <string>
#include <vector>

#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

namespace {
using JointTarget = std::array<double, 6>;

MoveAbsJCommand makeMoveAbsJ(const JointTarget &target, int speed, int zone) {
  return MoveAbsJCommand(JointPosition(std::vector<double>(target.begin(), target.end())), speed, zone);
}

void printJointPosition(xMateRobot &robot, error_code &ec) {
  const auto joints = robot.jointPos(ec);
  if (!ec) {
    printArray("joint position", joints, 4, " rad");
  }
}

}  // namespace

int main() {
  printHeader("示例 4: 基础运动控制", "官方 SDK 风格");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticNrt(robot, ec, 20, 8)) {
    cleanupRobot(robot);
    return 1;
  }

  robot.setDefaultConfOpt(false, ec);
  if (reportError("setDefaultConfOpt", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.setEventWatcher(Event::moveExecution,
                        [](const EventInfo &info) { printMoveEvent(info); },
                        ec);
  if (reportError("setEventWatcher", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  const JointTarget ready_pose{0.0, 0.45, 0.08, 0.0, 0.72, 0.0};
  const JointTarget left_pose{0.20, 0.32, 0.18, 0.12, 0.88, 0.06};
  const JointTarget right_pose{-0.22, 0.56, 0.12, -0.08, 0.74, -0.06};
  const JointTarget center_pose{0.0, 0.50, 0.05, 0.0, 0.82, 0.0};
  const JointTarget park_pose{0.0, 0.35, 0.0, 0.0, 0.65, 0.0};

  printSection("1 MoveAbsJ 到准备位");
  const auto ready_cmd = makeMoveAbsJ(ready_pose, 18, 0);
  robot.executeCommand(std::vector<MoveAbsJCommand>{ready_cmd}, ec);
  if (reportError("executeCommand(MoveAbsJ ready)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printJointPosition(robot, ec);

  printSection("2 多段关节路径");
  robot.executeCommand(std::vector<MoveAbsJCommand>{makeMoveAbsJ(left_pose, 16, 12),
                                                    makeMoveAbsJ(right_pose, 16, 12),
                                                    makeMoveAbsJ(center_pose, 16, 0)},
                       ec);
  if (reportError("executeCommand(multi MoveAbsJ)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printJointPosition(robot, ec);

  printSection("3 精确停靠");
  const auto park_cmd = makeMoveAbsJ(park_pose, 14, 0);
  robot.executeCommand(std::vector<MoveAbsJCommand>{park_cmd}, ec);
  if (reportError("executeCommand(MoveAbsJ park)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printJointPosition(robot, ec);

  printSection("4 断开连接");
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
