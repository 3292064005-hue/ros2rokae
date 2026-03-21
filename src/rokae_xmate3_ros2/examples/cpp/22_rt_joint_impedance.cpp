/**
 * @file 22_rt_joint_impedance.cpp
 * @brief 外部 rt/joint_impedance_control 的 xMate3 版
 */

#include <array>

#include "rokae/motion_control_rt.h"
#include "rokae/robot.h"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 22: RT 关节阻抗控制", "Gazebo simulated RT facade");
  os << "note: joint impedance behavior here is approximate and simulation-only" << std::endl;

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticRt(robot, ec, 20)) {
    cleanupRobot(robot);
    return 1;
  }

  auto rt = robot.getRtMotionController().lock();
  if (!ensurePtr(rt, "rt controller")) {
    cleanupRobot(robot);
    return 1;
  }

  robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m});
  std::array<double, 6> start_joints{};
  bool init = true;
  double time = 0.0;

  rt->MoveJ(0.3, robot.jointPos(ec), kXMate3DragPose);
  rt->setJointImpedance({500.0, 500.0, 500.0, 200.0, 100.0, 50.0}, ec);
  if (reportError("setJointImpedance", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  rt->startMove(RtControllerMode::jointImpedance);
  rt->setControlLoop(std::function<JointPosition(void)>([&]() {
    if (init) {
      robot.updateRobotState(std::chrono::milliseconds(1));
      robot.getStateData(RtSupportedFields::jointPos_m, start_joints);
      init = false;
    }
    time += 0.001;
    const double delta = 0.05 * (1.0 - std::cos(kPi * time));
    JointPosition cmd(6);
    cmd.joints = {start_joints[0] + delta,
                  start_joints[1],
                  start_joints[2] + delta * 0.5,
                  start_joints[3],
                  start_joints[4] - delta * 0.25,
                  start_joints[5]};
    if (time > 0.25) {
      cmd.setFinished();
    }
    return cmd;
  }));
  rt->startLoop(true);
  robot.stopReceiveRobotState();
  os << "rt joint impedance loop finished" << std::endl;

  printSection("2 恢复并断开连接");
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
