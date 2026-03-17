/**
 * @file 99_complete_demo.cpp
 * @brief 官方 SDK 风格 - 综合功能演示
 */

#include <array>
#include <chrono>
#include <string>
#include <vector>

#include "rokae/model.h"
#include "rokae/robot.h"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

namespace {

void printCurrentState(xMateRobot &robot, error_code &ec) {
  const auto joints = robot.jointPos(ec);
  if (reportError("jointPos", ec)) {
    return;
  }
  const auto pose = robot.cartPosture(CoordinateType::flangeInBase, ec);
  if (reportError("cartPosture", ec)) {
    return;
  }
  printArray("joint position", joints, 4, " rad");
  printPose("flange in base", pose);
}

}  // namespace

int main() {
  printHeader("示例 99: 综合功能演示", "官方 SDK 风格");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticNrt(robot, ec, 25, 5)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("1 初始化与基本信息");
  const auto info = robot.robotInfo(ec);
  if (reportError("robotInfo", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "robot type: " << info.type << std::endl;
  os << "controller version: " << info.version << std::endl;
  printCurrentState(robot, ec);
  if (ec) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("2 开启状态流");
  robot.startReceiveRobotState(std::chrono::milliseconds(200),
                               {RtSupportedFields::jointPos_m, RtSupportedFields::tcpPose_m});
  robot.updateRobotState(std::chrono::milliseconds(200));
  std::array<double, 6> q_m{};
  std::array<double, 16> tcp_pose_m{};
  if (robot.getStateData(RtSupportedFields::jointPos_m, q_m) == 0) {
    printArray("jointPos_m", q_m, 4, " rad");
  }
  if (robot.getStateData(RtSupportedFields::tcpPose_m, tcp_pose_m) == 0) {
    printArray("tcpPose_m", tcp_pose_m, 4);
  }
  robot.stopReceiveRobotState();

  printSection("3 关节与笛卡尔运动");
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

  robot.executeCommand(std::vector<MoveAbsJCommand>{MoveAbsJCommand({0.0, 0.5, 0.0, 0.0, 0.5, 0.0}, 30, 0)}, ec);
  if (reportError("executeCommand(MoveAbsJ)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.executeCommand(std::vector<MoveJCommand>{MoveJCommand({0.40, 0.00, 0.50, kPi, 0.0, 0.0}, 40, 5)}, ec);
  if (reportError("executeCommand(MoveJ)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.executeCommand(std::vector<MoveLCommand>{MoveLCommand({0.40, 0.10, 0.50, kPi, 0.0, 0.0}, 20, 5),
                                                 MoveLCommand({0.40, -0.10, 0.50, kPi, 0.0, 0.0}, 20, 5),
                                                 MoveLCommand({0.40, 0.00, 0.40, kPi, 0.0, 0.0}, 20, 0)},
                       ec);
  if (reportError("executeCommand(multi MoveL)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printCurrentState(robot, ec);
  if (ec) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("4 IO 与寄存器");
  robot.setSimulationMode(true, ec);
  if (reportError("setSimulationMode(true)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.setDO(0, 0, true, ec);
  if (reportError("setDO", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "DO[0][0] = " << (robot.getDO(0, 0, ec) ? "ON" : "OFF") << std::endl;
  if (reportError("getDO", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  float demo_reg = 3.14F;
  robot.writeRegister("demo_register", 0, demo_reg, ec);
  if (reportError("writeRegister(demo_register)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  demo_reg = 0.0F;
  robot.readRegister("demo_register", 0, demo_reg, ec);
  if (reportError("readRegister(demo_register)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "demo_register[0] = " << demo_reg << std::endl;
  robot.setSimulationMode(false, ec);

  printSection("5 模型接口");
  auto model = robot.model();
  const auto q = robot.jointPos(ec);
  if (reportError("jointPos", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  const auto fk = model.calcFk(q, ec);
  if (reportError("model.calcFk", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printPose("fk(current)", fk);

  printSection("6 断开连接");
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
