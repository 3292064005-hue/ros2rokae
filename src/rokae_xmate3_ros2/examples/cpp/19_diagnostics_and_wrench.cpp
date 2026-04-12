/**
 * @file 19_diagnostics_and_wrench.cpp
 * @brief 诊断、末端力矩与奇异规避示例
 */

#include <array>
#include <set>
#include <thread>

#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 19: 诊断与末端力矩", "getEndTorque / avoid singularity / diagnostics");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticNrt(robot, ec, 20, 5)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("1 默认配置选项与能力边界");
  robot.setAvoidSingularity(true, ec);
  if (ec) {
    os << "setAvoidSingularity(true): xMate6 当前实现返回 unsupported（符合机型能力边界） -> "
       << ec.message() << std::endl;
    ec.clear();
  }
  robot.setDefaultConfOpt(true, ec);
  if (reportError("setDefaultConfOpt(true)", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("2 查询末端力矩和力");
  std::array<double, 6> joint_tau{};
  std::array<double, 6> ext_tau{};
  std::array<double, 3> cart_tau{};
  std::array<double, 3> cart_force{};
  robot.getEndTorque(FrameType::tool, joint_tau, ext_tau, cart_tau, cart_force, ec);
  if (reportError("getEndTorque(tool)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printArray("joint torque measured", joint_tau, 4, " Nm");
  printArray("external torque measured", ext_tau, 4, " Nm");
  printArray("cartesian torque", cart_tau, 4, " Nm");
  printArray("cartesian force", cart_force, 4, " N");

  printSection("3 打开碰撞检测并查看安全事件");
  robot.setEventWatcher(Event::safety,
                        [](const EventInfo &info) { printSafetyEvent(info); },
                        ec);
  if (reportError("setEventWatcher(safety)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.enableCollisionDetection({1.0, 1.0, 0.6, 0.6, 0.6, 0.6}, StopLevel::stop1, 0.01, ec);
  if (reportError("enableCollisionDetection", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  printSafetyEvent(robot.queryEventInfo(Event::safety, ec));
  if (reportError("queryEventInfo(safety)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.disableCollisionDetection(ec);
  if (reportError("disableCollisionDetection", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("4 诊断与恢复接口");
  robot.clearServoAlarm(ec);
  if (reportError("clearServoAlarm", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  const auto logs = robot.queryControllerLog(
      3,
      {LogInfo::Level::warning, LogInfo::Level::error, LogInfo::Level::fatal},
      ec);
  if (reportError("queryControllerLog", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "diagnostic logs: " << logs.size() << std::endl;
  for (const auto &log : logs) {
    printLogInfo(log);
  }
  const auto last_error = robot.lastErrorCode();
  os << "last error: " << (last_error ? last_error.message() : "ok") << std::endl;

  printSection("5 恢复默认设置并断开连接");
  robot.setDefaultConfOpt(false, ec);
  robot.setAvoidSingularity(false, ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
