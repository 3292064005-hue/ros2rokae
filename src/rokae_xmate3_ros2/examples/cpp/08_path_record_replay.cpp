/**
 * @file 08_path_record_replay.cpp
 * @brief 官方 SDK 风格 - 路径录制与回放
 */

#include <string>

#include "rokae/robot.h"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 8: 路径录制与回放", "官方 SDK 风格");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticNrt(robot, ec, 15, 0)) {
    cleanupRobot(robot);
    return 1;
  }

  const std::string path_name = "sdk_demo_path";
  {
    error_code ignore_ec;
    robot.removePath(path_name, ignore_ec);
  }

  printSection("1 查询已有路径");
  const auto paths_before = robot.queryPathLists(ec);
  if (reportError("queryPathLists", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printVector("paths before record", paths_before);

  printSection("2 开始录制并执行一段示教轨迹");
  robot.startRecordPath(6, ec);
  if (reportError("startRecordPath", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.executeCommand({MoveAbsJCommand({0.0, 0.35, 0.05, 0.0, 0.75, 0.0}, 10, 0),
                        MoveAbsJCommand({0.0, 0.45, 0.00, 0.0, 0.65, 0.0}, 10, 0)},
                       ec);
  if (reportError("executeCommand(record trajectory)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.stopRecordPath(ec);
  if (reportError("stopRecordPath", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("3 保存并查询路径");
  robot.saveRecordPath(path_name, ec);
  if (reportError("saveRecordPath", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  const auto paths_after = robot.queryPathLists(ec);
  if (reportError("queryPathLists", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printVector("paths after record", paths_after);

  printSection("4 回放路径");
  os << "replay rate uses time-axis reparameterization in the Gazebo shim" << std::endl;
  robot.replayPath(path_name, 1.0, ec);
  if (reportError("replayPath", ec) || !waitMotionCycle(robot, ec, std::chrono::seconds(60))) {
    reportError("waitMotionCycle", ec);
    cleanupRobot(robot);
    return 1;
  }
  os << "path replay finished @ rate=1.0" << std::endl;

  robot.replayPath(path_name, 1.5, ec);
  if (reportError("replayPath(rate=1.5)", ec) || !waitMotionCycle(robot, ec, std::chrono::seconds(60))) {
    reportError("waitMotionCycle(rate=1.5)", ec);
    cleanupRobot(robot);
    return 1;
  }
  os << "path replay finished @ rate=1.5" << std::endl;

  printSection("5 断开连接");
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
