/**
 * @file 20_rt_joint_position.cpp
 * @brief 外部 rt/joint_position_control 的 xMate3 版
 */

#include <array>

#include "rokae/motion_control_rt.h"
#include "rokae/robot.h"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 20: RT 关节位置控制", "Gazebo simulated RT facade");
  os << "note: this is a simulation-only RT facade, not a hard real-time 1kHz controller" << std::endl;

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticRt(robot, ec, 20)) {
    return isSimulationOnlyCapabilityError(ec)
               ? skipExample(robot, "RT joint position facade unavailable in current simulation backend: " + ec.message())
               : (cleanupRobot(robot), 1);
  }

  auto rt = robot.getRtMotionController().lock();
  if (!rt) {
    return skipExample(robot, "RT joint position controller unavailable in current simulation backend");
  }

  printSection("1 MoveJ 到 RT 起始位");
  const auto current = robot.jointPos(ec);
  if (reportError("jointPos", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  rt->MoveJ(0.3, current, kXMate3DragPose);
  if (const auto movej_ec = robot.lastErrorCode(); movej_ec) {
    if (isSimulationOnlyCapabilityError(movej_ec)) {
      return skipExample(robot, "RT MoveJ unavailable in current simulation backend: " + movej_ec.message());
    }
    reportError("rt MoveJ", movej_ec);
    cleanupRobot(robot);
    return 1;
  }
  os << "rt MoveJ finished" << std::endl;

  printSection("2 启动 jointPosition 控制回调");
  robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m});
  std::array<double, 6> start_joints{};
  bool init = true;
  double time = 0.0;
  rt->startMove(RtControllerMode::jointPosition);
  rt->setControlLoop(std::function<JointPosition(void)>([&]() {
    if (init) {
      robot.updateRobotState(std::chrono::milliseconds(1));
      robot.getStateData(RtSupportedFields::jointPos_m, start_joints);
      init = false;
    }
    time += 0.001;
    const double delta = 0.08 * (1.0 - std::cos(kPi * time));
    JointPosition cmd(6);
    cmd.joints = {start_joints[0] + delta,
                  start_joints[1] + delta * 0.5,
                  start_joints[2] - delta * 0.5,
                  start_joints[3],
                  start_joints[4],
                  start_joints[5] + delta * 0.25};
    if (time > 0.25) {
      cmd.setFinished();
    }
    return cmd;
  }));
  rt->startLoop(true);
  if (const auto loop_ec = robot.lastErrorCode(); loop_ec) {
    robot.stopReceiveRobotState();
    if (isSimulationOnlyCapabilityError(loop_ec)) {
      return skipExample(robot, "RT joint position control loop unavailable in current simulation backend: " + loop_ec.message());
    }
    reportError("rt joint position loop", loop_ec);
    cleanupRobot(robot);
    return 1;
  }
  robot.stopReceiveRobotState();
  os << "rt joint position loop finished" << std::endl;

  printSection("3 恢复并断开连接");
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
