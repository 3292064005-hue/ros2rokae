/**
 * @file 01_basic_connect.cpp
 * @brief 官方 SDK 风格 - 基本连接、信息查询与状态读取
 */

#include <set>
#include <string>

#include "rokae/robot.h"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 1: 基本连接与信息查询", "官方 SDK 风格");

  error_code ec;
  xMateRobot robot;

  printSection("1 连接机器人并读取版本信息");
  os << "xCore-SDK version: " << robot.sdkVersion() << std::endl;
  if (!connectRobot(robot, ec)) {
    return 1;
  }

  const auto info = robot.robotInfo(ec);
  if (reportError("robotInfo", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "controller version: " << info.version << std::endl;
  os << "robot type: " << info.type << std::endl;
  os << "joint count: " << info.joint_num << std::endl;

  printSection("2 读取当前机器人状态");
  printArray("joint position", robot.jointPos(ec), 4, " rad");
  if (reportError("jointPos", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  const auto tcp_xyzabc = robot.posture(CoordinateType::endInRef, ec);
  if (reportError("posture(endInRef)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printArray("tcp xyzabc", tcp_xyzabc, 4);

  const auto flange_in_base = robot.cartPosture(CoordinateType::flangeInBase, ec);
  if (reportError("cartPosture(flangeInBase)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printPose("flange in base", flange_in_base);

  os << "power state: " << toString(robot.powerState(ec)) << std::endl;
  if (reportError("powerState", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "operate mode: " << toString(robot.operateMode(ec)) << std::endl;
  if (reportError("operateMode", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "operation state: " << toString(robot.operationState(ec)) << std::endl;
  if (reportError("operationState", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("3 切换到自动非实时模式");
  if (!prepareAutomaticNrt(robot, ec, 20, 5)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "power state: " << toString(robot.powerState(ec)) << std::endl;
  os << "operate mode: " << toString(robot.operateMode(ec)) << std::endl;
  os << "operation state: " << toString(robot.operationState(ec)) << std::endl;

  printSection("4 查询控制器日志");
  const auto logs = robot.queryControllerLog(3, std::set<LogInfo::Level>{}, ec);
  if (reportError("queryControllerLog", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  if (logs.empty()) {
    os << "no controller logs" << std::endl;
  } else {
    for (size_t i = 0; i < logs.size(); ++i) {
      os << '[' << i << "] " << logs[i].content << std::endl;
    }
  }

  printSection("5 断开连接");
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
