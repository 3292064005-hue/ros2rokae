/**
 * @file 18_toolset_only.cpp
 * @brief 官方 SDK 风格 - 工具与工具组管理演示（toolset-only）
 */

#include <array>
#include <string>

#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 18: 工具与工具组管理", "toolset only / no calibration");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticNrt(robot, ec, 120, 0)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("1 配置工具和工具组");
  Toolset toolset;
  toolset.tool_name = "tool_demo";
  toolset.wobj_name = "fixture_demo";
  toolset.load.mass = 0.8;
  toolset.end = Frame({0.0, 0.0, 0.08, 0.0, 0.0, 0.0});
  toolset.ref = Frame({0.45, 0.00, 0.20, 0.0, 0.0, 0.0});
  robot.setToolset(toolset, ec);
  if (reportError("setToolset", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  const auto active_toolset = robot.toolset(ec);
  if (reportError("toolset", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printToolset(active_toolset);

  printSection("2 切换为命名工具组");
  robot.setToolset("tool0", "wobj0", ec);
  if (reportError("setToolset(tool0, wobj0)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printToolset(robot.toolset(ec));
  if (reportError("toolset(named)", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("3 工具组与模型上下文一致性");
  auto model = robot.model();
  const auto q = robot.jointPos(ec);
  if (reportError("jointPos", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  const auto fk = model.calcFk(q, ec);
  if (ec) {
    printCapabilityStatus("approximate", "model.calcFk skipped in toolset-only smoke: " + ec.message());
    ec.clear();
  } else {
    printPose("fk(current toolset)", fk);
  }
  os << "toolset only complete: 工具组" << std::endl;

  printSection("4 断开连接");
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
