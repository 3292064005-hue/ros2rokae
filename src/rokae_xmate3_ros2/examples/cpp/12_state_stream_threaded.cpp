/**
 * @file 12_state_stream_threaded.cpp
 * @brief 外部 read_robot_state 的线程化状态流版
 */

#include <array>
#include <atomic>
#include <thread>
#include <vector>

#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 12: 线程化状态流读取", "read_robot_state 对齐版");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticNrt(robot, ec, 20, 5)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("1 启动状态流");
  using namespace RtSupportedFields;
  robot.startReceiveRobotState(std::chrono::milliseconds(8), {tcpPose_m, tau_m, jointPos_m});

  std::atomic_bool running{true};
  std::thread reader([&]() {
    int sample = 0;
    while (running && sample < 12) {
      std::array<double, 16> tcp_pose{};
      std::array<double, 6> joint_pos{};
      std::array<double, 6> tau{};
      robot.updateRobotState(std::chrono::milliseconds(8));
      os << "sample " << sample + 1 << ':' << std::endl;
      if (robot.getStateData(tcpPose_m, tcp_pose) == 0) {
        printArray("tcpPose_m", tcp_pose, 4);
      }
      if (robot.getStateData(jointPos_m, joint_pos) == 0) {
        printArray("jointPos_m", joint_pos, 4, " rad");
      }
      if (robot.getStateData(tau_m, tau) == 0) {
        printArray("tau_m", tau, 4, " Nm");
      }
      ++sample;
    }
  });

  printSection("2 运动线程");
  std::thread mover([&]() {
    error_code move_ec;
    robot.executeCommand(std::vector<MoveAbsJCommand>{MoveAbsJCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 20, 0),
                                                      MoveAbsJCommand({0.0, kPi / 6.0, kPi / 3.0, 0.0, kPi / 2.0, 0.0}, 20, 0)},
                         move_ec);
    if (move_ec) {
      std::cerr << "! executeCommand(threaded move) failed: " << move_ec.message() << std::endl;
    }
    running = false;
  });

  mover.join();
  reader.join();
  robot.stopReceiveRobotState();
  os << "state stream stopped" << std::endl;

  printSection("3 断开连接");
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
