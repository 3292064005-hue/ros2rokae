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
  printHeader("示例 20: RT 关节位置控制", "joint_position_control 对齐版");

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

  printSection("1 MoveJ 到 RT 起始位");
  const auto current = robot.jointPos(ec);
  if (reportError("jointPos", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  rt->MoveJ(0.3, current, kXMate3DragPose);
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
  robot.stopReceiveRobotState();
  os << "rt joint position loop finished" << std::endl;

  printSection("3 恢复并断开连接");
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
