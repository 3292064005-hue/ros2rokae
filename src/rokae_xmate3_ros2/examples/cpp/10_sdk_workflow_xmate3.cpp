/**
 * @file 10_sdk_workflow_xmate3.cpp
 * @brief 外部 sdk_example 的 xMate3 工作流整理版
 */

#include <array>
#include <thread>
#include <vector>

#include "rokae/robot.h"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

namespace {

const std::array<std::array<double, 6>, 4> kCalibrationPoses{{
    {{0.00, kPi / 6.0, kPi / 3.0, 0.00, kPi / 2.0, 0.00}},
    {{0.10, kPi / 6.0 + 0.05, kPi / 3.0 - 0.08, 0.00, kPi / 2.0 - 0.05, 0.10}},
    {{-0.08, kPi / 6.0 - 0.06, kPi / 3.0 + 0.10, 0.08, kPi / 2.0 - 0.08, -0.12}},
    {{0.05, kPi / 6.0 + 0.02, kPi / 3.0 + 0.04, -0.10, kPi / 2.0 + 0.03, 0.08}},
}};

std::vector<double> toVector(const std::array<double, 6> &joints) {
  return std::vector<double>(joints.begin(), joints.end());
}

}  // namespace

int main() {
  printHeader("示例 10: xMate3 SDK 工作流", "sdk_example 对齐版");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticNrt(robot, ec, 30, 5)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("1 设置与读取工具工件组");
  Toolset toolset;
  toolset.load.mass = 0.5;
  toolset.end = Frame({0.0, 0.0, 0.05, 0.0, 0.0, 0.0});
  toolset.ref = Frame({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  robot.setToolset(toolset, ec);
  if (reportError("setToolset", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printToolset(robot.toolset(ec));
  if (reportError("toolset", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("2 自动采点标定工具坐标系");
  std::vector<std::array<double, 6>> captured_points;
  for (size_t i = 0; i < kCalibrationPoses.size(); ++i) {
    robot.executeCommand(std::vector<MoveAbsJCommand>{MoveAbsJCommand(toVector(kCalibrationPoses[i]), 20, 0)}, ec);
    if (reportError("executeCommand(calibration pose)", ec)) {
      cleanupRobot(robot);
      return 1;
    }
    captured_points.push_back(robot.jointPos(ec));
    if (reportError("jointPos(capture)", ec)) {
      cleanupRobot(robot);
      return 1;
    }
    os << "captured point " << i + 1 << std::endl;
  }

  const auto result = robot.calibrateFrame(FrameType::tool, captured_points, true, ec);
  if (reportError("calibrateFrame(tool)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printArray("calibrated tool trans", result.frame.trans, 4, " m");
  printArray("calibrated tool rpy", result.frame.rpy, 4, " rad");
  printArray("calibration errors", result.errors, 6, " m");

  printSection("3 打开和关闭拖动");
  robot.setPowerState(false, ec);
  if (reportError("setPowerState(off)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.setOperateMode(OperateMode::manual, ec);
  if (reportError("setOperateMode(manual)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec);
  if (reportError("enableDrag", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  robot.disableDrag(ec);
  if (reportError("disableDrag", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "drag toggled" << std::endl;

  printSection("4 Jog 示例");
  robot.setPowerState(true, ec);
  if (reportError("setPowerState(on)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.startJog(JogOpt::world, 0.10, 10.0, 2, true, ec);
  if (reportError("startJog(world)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  robot.stop(ec);
  if (reportError("stop(world jog)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.startJog(JogOpt::jointSpace, 0.05, 5.0, 5, false, ec);
  if (reportError("startJog(jointSpace)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  robot.stop(ec);
  if (reportError("stop(joint jog)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "jog completed" << std::endl;

  printSection("5 打开和关闭碰撞检测");
  robot.enableCollisionDetection({1.0, 1.0, 0.5, 0.5, 0.5, 0.5}, StopLevel::stop1, 0.01, ec);
  if (reportError("enableCollisionDetection", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  robot.disableCollisionDetection(ec);
  if (reportError("disableCollisionDetection", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "collision detection toggled" << std::endl;

  printSection("6 恢复自动模式并断开连接");
  robot.setOperateMode(OperateMode::automatic, ec);
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
