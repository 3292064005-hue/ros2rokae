/**
 * @file 15_move_queue_and_events.cpp
 * @brief 运动队列、事件回调与控制器日志示例
 */

#include <set>
#include <vector>

#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 15: 运动队列与事件", "moveAppend / moveStart / Event / Log");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticNrt(robot, ec, 60, 2)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("1 配置缓存与事件观察者");
  robot.setMaxCacheSize(32, ec);
  if (reportError("setMaxCacheSize", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.setEventWatcher(Event::moveExecution,
                        [](const EventInfo &info) { printMoveEvent(info); },
                        ec);
  if (reportError("setEventWatcher(moveExecution)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.setEventWatcher(Event::safety,
                        [](const EventInfo &info) { printSafetyEvent(info); },
                        ec);
  if (reportError("setEventWatcher(safety)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "event watchers ready" << std::endl;

  printSection("2 查询最近控制器日志");
  const auto logs = robot.queryControllerLog(
      5,
      {LogInfo::Level::info, LogInfo::Level::warning, LogInfo::Level::error},
      ec);
  if (reportError("queryControllerLog", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "log count: " << logs.size() << std::endl;
  for (const auto &log : logs) {
    printLogInfo(log);
  }

  printSection("3 构造并执行三段 MoveAbsJ 队列");
  robot.moveReset(ec);
  if (reportError("moveReset", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  const std::vector<MoveAbsJCommand> cmds{
      MoveAbsJCommand({0.00, 0.35, 0.00, 0.00, 0.65, 0.00}, 60, 2),
      MoveAbsJCommand({0.04, 0.38, 0.02, 0.00, 0.68, -0.04}, 60, 2),
      MoveAbsJCommand({-0.04, 0.36, 0.00, 0.00, 0.66, 0.04}, 60, 0),
  };

  std::string cmd_id;
  robot.moveAppend(cmds, cmd_id, ec);
  if (reportError("moveAppend", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "queued cmd_id: " << cmd_id << std::endl;

  robot.moveStart(ec);
  if (reportError("moveStart", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  if (!waitForCommandOrIdle(robot,
                            cmd_id,
                            static_cast<int>(cmds.size()) - 1,
                            ec,
                            std::chrono::seconds(30))) {
    reportError("waitForCommandOrIdle", ec);
    cleanupRobot(robot);
    return 1;
  }

  printSection("4 查询当前运动事件快照");
  const auto event = robot.queryEventInfo(Event::moveExecution, ec);
  if (reportError("queryEventInfo(moveExecution)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printMoveEvent(event);
  printArray("joint position", robot.jointPos(ec), 4, " rad");
  if (reportError("jointPos", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("5 最近错误码与断开连接");
  const auto last_error = robot.lastErrorCode();
  os << "last error: " << (last_error ? last_error.message() : "ok") << std::endl;
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
