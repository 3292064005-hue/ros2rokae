/**
 * @file 21_rt_move_commands.cpp
 * @brief 外部 rt/move_commands 的 xMate3 版
 */

#include "rokae/motion_control_rt.h"
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 21: RT MoveJ / MoveL / MoveC", "官方 SDK 风格");
  os << "note: RT 指令在 ROS2/Gazebo 仿真环境执行，接口调用方式与官方 SDK 一致" << std::endl;
  os << "rt profile: 1kHz state period (" << kRtControlPeriod.count() << " ms)" << std::endl;

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
  robot.startReceiveRobotState(kRtControlPeriod, {RtSupportedFields::jointPos_m, RtSupportedFields::tcpPose_m});

  printSection("1 RT MoveJ");
  rt->MoveJ(0.3, robot.jointPos(ec), kXMate3DragPose);
  if (const auto movej_ec = robot.lastErrorCode(); reportError("rt MoveJ", movej_ec)) {
    robot.stopReceiveRobotState();
    return isSimulationOnlyCapabilityError(movej_ec)
               ? skipExample(robot, "RT MoveJ unavailable in current simulation backend: " + movej_ec.message())
               : (cleanupRobot(robot), 1);
  }
  robot.updateRobotState(kRtControlPeriod);

  printSection("2 RT MoveC");
  auto start = robot.cartPosture(CoordinateType::flangeInBase, ec);
  if (reportError("cartPosture(start)", ec)) {
    robot.stopReceiveRobotState();
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
    robot.stopReceiveRobotState();
    return isSimulationOnlyCapabilityError(movec_ec)
               ? skipExample(robot, "RT MoveC unavailable in current simulation backend: " + movec_ec.message())
               : (cleanupRobot(robot), 1);
  }
  robot.updateRobotState(kRtControlPeriod);

  printSection("3 RT MoveL");
  start = robot.cartPosture(CoordinateType::flangeInBase, ec);
  if (reportError("cartPosture(line start)", ec)) {
    robot.stopReceiveRobotState();
    cleanupRobot(robot);
    return 1;
  }
  target = start;
  target.x -= 0.02;
  target.z += 0.03;
  rt->MoveL(0.12, start, target);
  if (const auto movel_ec = robot.lastErrorCode(); reportError("rt MoveL", movel_ec)) {
    robot.stopReceiveRobotState();
    return isSimulationOnlyCapabilityError(movel_ec)
               ? skipExample(robot, "RT MoveL unavailable in current simulation backend: " + movel_ec.message())
               : (cleanupRobot(robot), 1);
  }
  robot.updateRobotState(kRtControlPeriod);
  printPose("flange in base", robot.cartPosture(CoordinateType::flangeInBase, ec));

  printSection("4 恢复并断开连接");
  robot.stopReceiveRobotState();
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
