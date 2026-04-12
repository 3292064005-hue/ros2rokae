/**
 * @file 07_safety_collision.cpp
 * @brief 官方 SDK 风格 - 碰撞检测、软限位与拖动示教
 */

#include <array>
#include <thread>

#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 7: 安全与碰撞检测", "官方 SDK 风格");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticNrt(robot, ec, 20, 5)) {
    cleanupRobot(robot);
    return 1;
  }

  robot.setEventWatcher(
      Event::safety,
      [](const EventInfo &info) {
        const auto it = info.find(EventInfoKey::Safety::Collided);
        if (it != info.end()) {
          try {
            os << "[safety] collided="
               << (std::any_cast<bool>(it->second) ? "true" : "false") << std::endl;
          } catch (const std::exception &) {
          }
        }
      },
      ec);
  if (reportError("setEventWatcher(safety)", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("1 启用碰撞检测");
  const std::array<double, 6> sensitivity{0.8, 0.8, 0.8, 0.8, 0.8, 0.8};
  robot.enableCollisionDetection(sensitivity, StopLevel::stop2, 0.01, ec);
  if (reportError("enableCollisionDetection", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printArray("collision sensitivity", sensitivity, 2);

  printSection("2 查询并回写当前软限位");
  std::array<double[2], 6> limits{};
  const bool enabled = robot.getSoftLimit(limits, ec);
  if (reportError("getSoftLimit", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "soft limit enabled: " << (enabled ? "true" : "false") << std::endl;
  for (size_t i = 0; i < limits.size(); ++i) {
    os << "joint" << (i + 1) << ": [" << limits[i][0] << ", " << limits[i][1] << "]" << std::endl;
  }
  robot.setSoftLimit(enabled, ec, limits);
  if (reportError("setSoftLimit", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("3 打开并关闭拖动示教");
  robot.setOperateMode(OperateMode::manual, ec);
  if (reportError("setOperateMode(manual)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.setPowerState(false, ec);
  if (reportError("setPowerState(false)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec);
  if (reportError("enableDrag", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "drag enabled for 2 seconds" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(2));
  robot.disableDrag(ec);
  if (reportError("disableDrag", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("4 恢复自动模式并清除伺服报警");
  robot.setOperateMode(OperateMode::automatic, ec);
  if (reportError("setOperateMode(automatic)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.setPowerState(true, ec);
  if (reportError("setPowerState(true)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.clearServoAlarm(ec);
  if (reportError("clearServoAlarm", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("5 断开连接");
  robot.disableCollisionDetection(ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
