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

  printSection("2 打开和关闭拖动");
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

  printSection("3 Jog 示例");
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

  printSection("4 打开和关闭碰撞检测");
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

  printSection("5 恢复自动模式并断开连接");
  robot.setOperateMode(OperateMode::automatic, ec);
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
