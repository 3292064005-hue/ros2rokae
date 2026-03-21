/**
 * @file 05_motion_cartesian.cpp
 * @brief 官方 SDK 风格 - MoveJ / MoveL / MoveC
 */

#include <array>
#include <string>
#include <vector>

#include "rokae/robot.h"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

namespace {

void printCartesianPosition(xMateRobot &robot, error_code &ec) {
  const auto pose = robot.cartPosture(CoordinateType::flangeInBase, ec);
  if (!ec) {
    printPose("flange in base", pose);
  }
}

}  // namespace

int main() {
  printHeader("示例 5: 笛卡尔空间运动", "官方 SDK 风格");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticNrt(robot, ec, 180, 5)) {
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

  printSection("1 MoveAbsJ 到起始位");
  robot.executeCommand(std::vector<MoveAbsJCommand>{MoveAbsJCommand({0.0, 0.5, 0.0, 0.0, 0.5, 0.0}, 180, 0)}, ec);
  if (reportError("executeCommand(MoveAbsJ start)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printCartesianPosition(robot, ec);

  printSection("2 MoveJ 到笛卡尔目标");
  robot.executeCommand(std::vector<MoveJCommand>{MoveJCommand({0.40, 0.00, 0.50, kPi, 0.0, 0.0}, 220, 5)}, ec);
  if (reportError("executeCommand(MoveJ)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printCartesianPosition(robot, ec);

  printSection("3 MoveL 直线运动");
  robot.executeCommand(std::vector<MoveLCommand>{MoveLCommand({0.30, 0.20, 0.50, kPi, 0.0, 0.0}, 180, 0)}, ec);
  if (reportError("executeCommand(MoveL)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printCartesianPosition(robot, ec);

  printSection("4 MoveC 圆弧运动");
  robot.executeCommand(std::vector<MoveCCommand>{MoveCCommand(CartesianPosition({0.40, 0.00, 0.45, kPi, 0.0, 0.0}),
                                                              CartesianPosition({0.35, 0.25, 0.50, kPi, 0.0, 0.0}),
                                                              160,
                                                              0)},
                       ec);
  if (reportError("executeCommand(MoveC)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printCartesianPosition(robot, ec);

  printSection("5 多段 MoveL");
  robot.executeCommand(std::vector<MoveLCommand>{MoveLCommand({0.40, 0.10, 0.50, kPi, 0.0, 0.0}, 180, 5),
                                                 MoveLCommand({0.40, -0.10, 0.50, kPi, 0.0, 0.0}, 180, 5),
                                                 MoveLCommand({0.40, 0.00, 0.40, kPi, 0.0, 0.0}, 180, 0)},
                       ec);
  if (reportError("executeCommand(multi MoveL)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printCartesianPosition(robot, ec);

  printSection("6 断开连接");
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
