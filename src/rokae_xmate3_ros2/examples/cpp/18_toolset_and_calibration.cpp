/**
 * @file 18_toolset_and_calibration.cpp
 * @brief 工具、工件与坐标系标定示例
 */

#include <array>
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
  printHeader("示例 18: 工具与坐标系标定", "toolset / calibrateFrame");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticNrt(robot, ec, 120, 0)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("1 配置工具和工件");
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
  printToolset(robot.toolset(ec));
  if (reportError("toolset", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  const auto tools = robot.toolsInfo(ec);
  if (reportError("toolsInfo", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "tools count: " << tools.size() << std::endl;
  for (const auto &tool : tools) {
    os << "tool meta: " << tool.name << ", alias=" << tool.alias << ", mass=" << tool.load.mass << std::endl;
  }

  printSection("2 采集标定点");
  std::vector<std::array<double, 6>> captured_points;
  for (size_t i = 0; i < kCalibrationPoses.size(); ++i) {
    robot.executeCommand(std::vector<MoveAbsJCommand>{MoveAbsJCommand(toVector(kCalibrationPoses[i]), 120, 0)}, ec);
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

  printSection("3 标定工具坐标系");
  const auto tool_result = robot.calibrateFrame(FrameType::tool, captured_points, true, ec);
  if (reportError("calibrateFrame(tool)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printArray("tool trans", tool_result.frame.trans, 4, " m");
  printArray("tool rpy", tool_result.frame.rpy, 4, " rad");
  printArray("tool error stats", tool_result.errors, 6, " m");

  printSection("4 标定工件参考系");
  const std::vector<std::array<double, 6>> ref_points{
      captured_points[0],
      captured_points[1],
      captured_points[2],
  };
  const auto ref_result = robot.calibrateFrame(FrameType::wobj, ref_points, true, ec);
  if (reportError("calibrateFrame(wobj)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printArray("wobj trans", ref_result.frame.trans, 4, " m");
  printArray("wobj rpy", ref_result.frame.rpy, 4, " rad");
  printArray("wobj error stats", ref_result.errors, 6, " m");

  printSection("5 断开连接");
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
