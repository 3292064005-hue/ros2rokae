/**
 * @file 21_rt_move_commands.cpp
 * @brief 外部 rt/move_commands 的 xMate3 版
 */

#include "rokae/motion_control_rt.h"
#include "rokae/robot.h"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 21: RT MoveJ / MoveL / MoveC", "Gazebo simulated RT facade");
  os << "note: RT move commands are executed as simulated RT commands in Gazebo" << std::endl;

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticRt(robot, ec, 20)) {
    return isSimulationOnlyCapabilityError(ec)
               ? skipExample(robot, "RT move commands unavailable in current simulation backend: " + ec.message())
               : (cleanupRobot(robot), 1);
  }

  auto rt = robot.getRtMotionController().lock();
  if (!rt) {
    return skipExample(robot, "RT move command controller unavailable in current simulation backend");
  }

  printSection("1 RT MoveJ");
  rt->MoveJ(0.3, robot.jointPos(ec), kXMate3DragPose);
  if (const auto movej_ec = robot.lastErrorCode(); reportError("rt MoveJ", movej_ec)) {
    return isSimulationOnlyCapabilityError(movej_ec)
               ? skipExample(robot, "RT MoveJ unavailable in current simulation backend: " + movej_ec.message())
               : (cleanupRobot(robot), 1);
  }

  printSection("2 RT MoveC");
  auto start = robot.cartPosture(CoordinateType::flangeInBase, ec);
  if (reportError("cartPosture(start)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  auto aux = start;
  auto target = start;
  aux.x -= 0.03;
  aux.y += 0.02;
  aux.z += 0.01;
  target.x -= 0.06;
  target.z += 0.02;
  rt->MoveC(0.12, start, aux, target);
  if (const auto movec_ec = robot.lastErrorCode(); reportError("rt MoveC", movec_ec)) {
    return isSimulationOnlyCapabilityError(movec_ec)
               ? skipExample(robot, "RT MoveC unavailable in current simulation backend: " + movec_ec.message())
               : (cleanupRobot(robot), 1);
  }

  printSection("3 RT MoveL");
  start = robot.cartPosture(CoordinateType::flangeInBase, ec);
  if (reportError("cartPosture(line start)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  target = start;
  target.x -= 0.02;
  target.z += 0.03;
  rt->MoveL(0.12, start, target);
  if (const auto movel_ec = robot.lastErrorCode(); reportError("rt MoveL", movel_ec)) {
    return isSimulationOnlyCapabilityError(movel_ec)
               ? skipExample(robot, "RT MoveL unavailable in current simulation backend: " + movel_ec.message())
               : (cleanupRobot(robot), 1);
  }
  printPose("flange in base", robot.cartPosture(CoordinateType::flangeInBase, ec));

  printSection("4 恢复并断开连接");
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
