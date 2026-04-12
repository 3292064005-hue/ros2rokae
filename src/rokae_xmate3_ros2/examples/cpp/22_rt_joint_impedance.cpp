/**
 * @file 22_rt_joint_impedance.cpp
 * @brief 外部 rt/joint_impedance_control 的 xMate3 版
 */

#include <array>

#include "rokae/motion_control_rt.h"
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 22: RT 关节阻抗控制", "官方 SDK 风格");
  os << "note: 当前为仿真运行链路，接口调用方式与官方 SDK 一致" << std::endl;
  os << "rt profile: 1kHz control period (" << kRtControlPeriod.count() << " ms)" << std::endl;

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticRt(robot, ec, 20)) {
    return isSimulationOnlyCapabilityError(ec)
               ? skipExample(robot, "RT joint impedance facade unavailable in current simulation backend: " + ec.message())
               : (cleanupRobot(robot), 1);
  }

  auto rt = robot.getRtMotionController().lock();
  if (!rt) {
    return skipExample(robot, "RT joint impedance controller unavailable in current simulation backend");
  }

  robot.startReceiveRobotState(kRtControlPeriod, {RtSupportedFields::jointPos_m});
  std::array<double, 6> start_joints{};
  bool init = true;
  double time = 0.0;

  rt->MoveJ(0.3, robot.jointPos(ec), kXMate3DragPose);
  if (const auto movej_ec = robot.lastErrorCode(); movej_ec) {
    robot.stopReceiveRobotState();
    if (isSimulationOnlyCapabilityError(movej_ec)) {
      return skipExample(robot, "RT joint impedance MoveJ unavailable in current simulation backend: " + movej_ec.message());
    }
    reportError("rt MoveJ", movej_ec);
    cleanupRobot(robot);
    return 1;
  }
  rt->setJointImpedance({500.0, 500.0, 500.0, 200.0, 100.0, 50.0}, ec);
  if (reportError("setJointImpedance", ec)) {
    return isSimulationOnlyCapabilityError(ec)
               ? skipExample(robot, "Joint impedance configuration unavailable in current simulation backend: " + ec.message())
               : (cleanupRobot(robot), 1);
  }
  rt->startMove(RtControllerMode::jointImpedance);
  rt->setControlLoop(std::function<JointPosition(void)>([&]() {
    if (init) {
      robot.updateRobotState(kRtControlPeriod);
      robot.getStateData(RtSupportedFields::jointPos_m, start_joints);
      init = false;
    }
    time += kRtControlDtSec;
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
  if (const auto loop_ec = robot.lastErrorCode(); loop_ec) {
    robot.stopReceiveRobotState();
    if (isSimulationOnlyCapabilityError(loop_ec)) {
      return skipExample(robot, "RT joint impedance loop unavailable in current simulation backend: " + loop_ec.message());
    }
    reportError("rt joint impedance loop", loop_ec);
    cleanupRobot(robot);
    return 1;
  }
  robot.stopReceiveRobotState();
  os << "rt joint impedance loop finished" << std::endl;

  printSection("2 恢复并断开连接");
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
