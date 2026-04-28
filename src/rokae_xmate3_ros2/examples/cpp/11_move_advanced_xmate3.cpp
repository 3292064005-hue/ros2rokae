/**
 * @file 11_move_advanced_xmate3.cpp
 * @brief 外部 move_example 的 xMate3 运动特性整理版
 */

#include <array>
#include <thread>
#include <vector>

#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

namespace {

std::vector<double> toVector(const std::array<double, 6> &joints) {
  return std::vector<double>(joints.begin(), joints.end());
}

}  // namespace

int main() {
  printHeader("示例 11: xMate3 高级运动", "move_example 对齐版");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticNrt(robot, ec, 40, 5)) {
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

  printSection("1 运动到拖拽位姿");
  robot.executeCommand(std::vector<MoveAbsJCommand>{MoveAbsJCommand(toVector(kXMate3DragPose), 30, 0)}, ec);
  if (reportError("executeCommand(drag pose)", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("2 confData 与默认轴配置选解");
  CartesianPosition pose_a({0.40, 0.00, 0.50, kPi, 0.0, 0.0});
  CartesianPosition pose_b({0.40, 0.08, 0.48, kPi, 0.0, 0.0});
  pose_b.confData = {-1, 1, -1, 0, 1, 0, 0, 2};

  robot.setDefaultConfOpt(false, ec);
  if (reportError("setDefaultConfOpt(false)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.executeCommand(std::vector<MoveJCommand>{MoveJCommand(pose_a, 35, 5), MoveJCommand(pose_b, 35, 0)}, ec);
  if (reportError("executeCommand(MoveJ without forced conf)", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  robot.setDefaultConfOpt(true, ec);
  if (reportError("setDefaultConfOpt(true)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  const auto line_seed = robot.cartPosture(CoordinateType::endInRef, ec);
  if (reportError("cartPosture(line target)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  struct OffsetCandidate {
    double x;
    double y;
    double z;
  };
  const std::array<OffsetCandidate, 6> forced_conf_offsets{{
      {0.00, 0.01, -0.005},
      {0.00, 0.015, -0.005},
      {0.01, 0.00, -0.005},
      {-0.01, 0.01, 0.000},
      {0.00, -0.01, 0.005},
      {0.00, 0.02, 0.000},
  }};
  bool forced_conf_ok = false;
  for (const auto &offset : forced_conf_offsets) {
    auto line_target = line_seed;
    line_target.x += offset.x;
    line_target.y += offset.y;
    line_target.z += offset.z;
    line_target.confData.clear();
    ec.clear();
    robot.executeCommand(std::vector<MoveLCommand>{MoveLCommand(line_target, 20, 0)}, ec);
    if (!ec) {
      os << "forced default conf offset: ["
         << std::fixed << std::setprecision(4)
         << offset.x << ", " << offset.y << ", " << offset.z << "]" << std::endl;
      forced_conf_ok = true;
      break;
    }
  }
  if (!forced_conf_ok) {
    if (reportError("executeCommand(MoveL with forced default conf)", ec)) {
      cleanupRobot(robot);
      return 1;
    }
  }
  robot.setDefaultConfOpt(false, ec);
  if (reportError("setDefaultConfOpt(false)", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("3 笛卡尔偏移点位");
  const auto current_pose = robot.cartPosture(CoordinateType::endInRef, ec);
  if (reportError("cartPosture(endInRef)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  MoveLCommand move_l1(current_pose, 20, 0);
  MoveLCommand move_l2(current_pose, 20, 0);
  move_l2.offset = {CartesianPosition::Offset::offs, {0.0, 0.0, 0.05, 0.0, 0.0, 0.0}};

  MoveJCommand move_j1(current_pose, 25, 0);
  move_j1.offset = {CartesianPosition::Offset::relTool, {0.0, 0.0, 0.03, 0.0, 0.0, 0.0}};

  robot.executeCommand(std::vector<MoveLCommand>{move_l1, move_l2}, ec);
  if (reportError("executeCommand(offset MoveL)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.executeCommand(std::vector<MoveJCommand>{move_j1}, ec);
  if (reportError("executeCommand(offset MoveJ)", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("4 螺旋线 MoveSP");
  robot.moveReset(ec);
  if (reportError("moveReset(before MoveSP)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  std::string spiral_id;
  const auto spiral_pose = robot.cartPosture(CoordinateType::endInRef, ec);
  if (reportError("cartPosture(spiral)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  auto sp_target0 = spiral_pose;
  auto sp_target1 = spiral_pose;
  sp_target0.z += 0.03;
  sp_target1.z += 0.06;
  sp_target1.rx -= 0.05;
  sp_target1.ry += 0.03;
  sp_target1.rz -= 0.04;
  robot.moveAppend(std::vector<MoveAbsJCommand>{MoveAbsJCommand({0.0, 0.2215, 1.4780, 0.0, 1.2676, 0.0}, 30, 0)}, spiral_id, ec);
  if (reportError("moveAppend(MoveAbsJ before MoveSP)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  const std::vector<MoveSPCommand> spiral_cmds{
      MoveSPCommand(sp_target0, 0.004, 0.0001, kPi / 2.0, false, 45, 0),
      MoveSPCommand(sp_target1, 0.006, 0.0002, kPi, true, 35, 0),
  };
  robot.moveAppend(spiral_cmds, spiral_id, ec);
  if (reportError("moveAppend(MoveSP)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.moveStart(ec);
  if (reportError("moveStart(MoveSP)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  if (!waitForCommandOrIdle(robot, spiral_id, static_cast<int>(spiral_cmds.size()) - 1, ec)) {
    reportError("waitForCommandOrIdle(MoveSP)", ec);
    cleanupRobot(robot);
    return 1;
  }
  if (const auto spiral_ec = robot.lastErrorCode(); reportError("MoveSP lastErrorCode", spiral_ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("5 在线调整速度");
  robot.moveReset(ec);
  if (reportError("moveReset(before adjustSpeedOnline)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  std::string speed_id;
  std::vector<MoveAbsJCommand> speed_cmds{
      MoveAbsJCommand({0.0, 0.20, 1.45, 0.0, 1.30, 0.0}, 30, 0),
      MoveAbsJCommand({0.0, 0.35, 1.30, 0.0, 1.10, 0.0}, 30, 0),
      MoveAbsJCommand({0.0, 0.20, 1.45, 0.0, 1.30, 0.0}, 30, 0),
      MoveAbsJCommand({0.0, 0.35, 1.30, 0.0, 1.10, 0.0}, 30, 0),
  };
  robot.adjustSpeedOnline(0.30, ec);
  if (reportError("adjustSpeedOnline(0.30)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.moveAppend(speed_cmds, speed_id, ec);
  if (reportError("moveAppend(speed queue)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.moveStart(ec);
  if (reportError("moveStart(speed queue)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  std::thread speed_adjuster([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    error_code adjust_ec;
    robot.adjustSpeedOnline(0.60, adjust_ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    robot.adjustSpeedOnline(1.00, adjust_ec);
  });
  if (!waitForCommandOrIdle(robot, speed_id, static_cast<int>(speed_cmds.size()) - 1, ec)) {
    reportError("waitForCommandOrIdle(speed queue)", ec);
    speed_adjuster.join();
    cleanupRobot(robot);
    return 1;
  }
  speed_adjuster.join();

  printSection("6 断开连接");
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
