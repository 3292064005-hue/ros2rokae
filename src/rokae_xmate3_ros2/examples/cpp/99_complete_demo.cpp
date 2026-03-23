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

bool executeRobustJointMoveFromCurrentState(xMateRobot &robot, error_code &ec) {
  const auto base_joints = robot.jointPos(ec);
  if (reportError("jointPos", ec)) {
    return false;
  }

  const std::array<std::array<double, 6>, 4> candidates{{
      {base_joints[0] + 0.10, base_joints[1] + 0.08, base_joints[2] - 0.12, base_joints[3], base_joints[4] - 0.05, base_joints[5] + 0.08},
      {base_joints[0] - 0.12, base_joints[1] + 0.05, base_joints[2] + 0.08, base_joints[3], base_joints[4] + 0.03, base_joints[5] - 0.10},
      {base_joints[0] + 0.06, base_joints[1] - 0.04, base_joints[2] + 0.10, base_joints[3], base_joints[4] + 0.08, base_joints[5] + 0.05},
      {base_joints[0] - 0.08, base_joints[1] - 0.02, base_joints[2] - 0.08, base_joints[3], base_joints[4] - 0.04, base_joints[5] - 0.06},
  }};

  for (const auto &target_joints : candidates) {
    MoveAbsJCommand target(std::vector<double>(target_joints.begin(), target_joints.end()), 35, 5);
    ec.clear();
    robot.executeCommand(std::vector<MoveAbsJCommand>{target}, ec);
    if (!ec && waitForCommandResult(robot, "executeCommand(MoveAbsJ refine)", ec)) {
      printArray("joint refine target", target_joints, 4, " rad");
      return true;
    }
    if (ec) {
      printCapabilityStatus("retry", "Joint refine target rejected, trying next candidate: " + ec.message());
    }
    error_code ignore_error;
    robot.moveReset(ignore_error);
  }

  return !reportError("executeCommand(MoveAbsJ refine)", ec);
}

bool executeRobustMoveLSequence(xMateRobot &robot, error_code &ec) {
  const auto base_pose = robot.cartPosture(CoordinateType::flangeInBase, ec);
  if (reportError("cartPosture(flangeInBase)", ec)) {
    return false;
  }

  struct LineSequence {
    CartesianPosition a;
    CartesianPosition b;
    CartesianPosition c;
    const char *label;
  };
  const auto make_pose = [&base_pose](double dx, double dy, double dz) {
    CartesianPosition pose = base_pose;
    pose.x += dx;
    pose.y += dy;
    pose.z = std::max(0.15, base_pose.z + dz);
    return pose;
  };
  const std::array<LineSequence, 4> candidates{{
      {make_pose(0.00, 0.04, 0.00),
       make_pose(0.00, -0.04, 0.00),
       make_pose(0.00, 0.00, -0.03),
       "y=+/-0.04 z=-0.03"},
      {make_pose(0.01, 0.03, 0.00),
       make_pose(0.01, -0.03, 0.00),
       make_pose(0.01, 0.00, -0.02),
       "x=+0.01 y=+/-0.03"},
      {make_pose(-0.01, 0.02, 0.00),
       make_pose(-0.01, -0.02, 0.00),
       make_pose(-0.01, 0.00, -0.02),
       "x=-0.01 y=+/-0.02"},
      {make_pose(0.00, 0.02, 0.01),
       make_pose(0.00, -0.02, 0.01),
       make_pose(0.00, 0.00, -0.01),
       "y=+/-0.02 z sweep"},
  }};

  for (const auto &sequence : candidates) {
    ec.clear();
    robot.executeCommand(std::vector<MoveLCommand>{
                             MoveLCommand(sequence.a, 120, 5),
                             MoveLCommand(sequence.b, 120, 5),
                             MoveLCommand(sequence.c, 120, 0),
                         },
                         ec);
    if (!ec && waitForCommandResult(robot, "executeCommand(multi MoveL)", ec)) {
      os << "MoveL sequence: " << sequence.label << std::endl;
      return true;
    }
    if (ec) {
      printCapabilityStatus("retry", std::string("MoveL sequence ") + sequence.label +
                                         " rejected, trying next path: " + ec.message());
    }
    error_code ignore_error;
    robot.moveReset(ignore_error);
  }

  return !reportError("executeCommand(multi MoveL)", ec);
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

  const std::vector<MoveAbsJCommand> start_cmds{
      MoveAbsJCommand({0.0, 0.5, 0.0, 0.0, 0.5, 0.0}, 30, 0),
  };
  robot.executeCommand(start_cmds, ec);
  if (reportError("executeCommand(MoveAbsJ)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  if (!waitForCommandResult(robot, "executeCommand(MoveAbsJ)", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  if (!executeRobustJointMoveFromCurrentState(robot, ec)) {
    ec.clear();
    printCapabilityStatus("approximate",
                          "Joint-space refine demo target set was not reachable in current simulation state; continuing with line motion demo");
  }

  if (!executeRobustMoveLSequence(robot, ec)) {
    ec.clear();
    printCapabilityStatus("approximate",
                          "MoveL sequence demo target set was not fully reachable; continuing with remaining SDK facade demo");
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
