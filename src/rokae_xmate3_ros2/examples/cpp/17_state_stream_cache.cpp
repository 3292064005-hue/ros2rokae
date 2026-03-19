/**
 * @file 17_state_stream_cache.cpp
 * @brief 机器人状态缓存与轮询示例
 */

#include <chrono>
#include <thread>
#include <vector>

#include "rokae/robot.h"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

namespace {

void printSnapshot(xMateRobot &robot, error_code &ec, int index) {
  const auto updated = robot.updateRobotState(std::chrono::milliseconds(100));
  os << "snapshot " << index << ": updated fields = " << updated << std::endl;

  std::array<double, 6> q_m{};
  std::array<double, 6> dq_m{};
  std::array<double, 6> tau_m{};
  std::array<double, 16> tcp_pose_m{};
  double psi_m = 0.0;
  if (robot.getStateData(RtSupportedFields::jointPos_m, q_m) == 0) {
    printArray("q_m", q_m, 4, " rad");
  }
  if (robot.getStateData(RtSupportedFields::jointVel_m, dq_m) == 0) {
    printArray("dq_m", dq_m, 4, " rad/s");
  }
  if (robot.getStateData(RtSupportedFields::tau_m, tau_m) == 0) {
    printArray("tau_m", tau_m, 4, " Nm");
  }
  if (robot.getStateData(RtSupportedFields::tcpPose_m, tcp_pose_m) == 0) {
    printArray("tcpPose_m", tcp_pose_m, 4);
  }
  if (robot.getStateData(RtSupportedFields::elbow_m, psi_m) == 0) {
    os << "psi_m: " << psi_m << std::endl;
  }
  if (ec) {
    reportError("updateRobotState", ec);
  }
}

}  // namespace

int main() {
  printHeader("示例 17: 状态缓存轮询", "startReceiveRobotState / updateRobotState");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticNrt(robot, ec, 20, 5)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("1 启动状态缓存");
  robot.startReceiveRobotState(
      std::chrono::milliseconds(50),
      {RtSupportedFields::jointPos_m,
       RtSupportedFields::jointVel_m,
       RtSupportedFields::tau_m,
       RtSupportedFields::tcpPose_m,
       RtSupportedFields::elbow_m});
  printSnapshot(robot, ec, 0);
  if (ec) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("2 异步执行一段 MoveAbsJ 并轮询状态");
  robot.moveReset(ec);
  if (reportError("moveReset", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  std::string cmd_id;
  std::vector<MoveAbsJCommand> cmds{
      MoveAbsJCommand({0.05, 0.40, 0.05, 0.00, 0.75, 0.00}, 20, 5),
      MoveAbsJCommand({-0.05, 0.45, 0.00, 0.00, 0.80, 0.00}, 20, 0),
  };
  robot.moveAppend(cmds, cmd_id, ec);
  if (reportError("moveAppend", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.moveStart(ec);
  if (reportError("moveStart", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  for (int i = 1; i <= 5; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    printSnapshot(robot, ec, i);
    if (ec) {
      cleanupRobot(robot);
      return 1;
    }
  }

  if (!waitRobot(robot, ec, std::chrono::seconds(10))) {
    reportError("waitRobot", ec);
    cleanupRobot(robot);
    return 1;
  }
  robot.stopReceiveRobotState();

  printSection("3 断开连接");
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
