/**
 * @file 23_rt_cartesian_impedance.cpp
 * @brief 外部 rt/cartesian_impedance_control 的 xMate3 版
 */

#include <array>

#include "rokae/motion_control_rt.h"
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 23: RT 笛卡尔阻抗控制", "Gazebo simulated RT facade");
  os << "note: cartesian impedance behavior here is approximate and simulation-only" << std::endl;

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

  printSection("1 配置阻抗参数");
  rt->MoveJ(0.3, robot.jointPos(ec), kXMate3DragPose);
  rt->setFcCoor({1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1},
                FrameType::tool,
                ec);
  if (reportError("setFcCoor", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  rt->setCartesianImpedance({1200.0, 1200.0, 200.0, 100.0, 100.0, 20.0}, ec);
  if (reportError("setCartesianImpedance", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  rt->setCartesianImpedanceDesiredTorque({3.0, 0.0, 3.0, 0.0, 0.0, 0.0}, ec);
  if (reportError("setCartesianImpedanceDesiredTorque", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("2 启动 cartesianImpedance 控制回调");
  std::array<double, 16> init_pose{};
  Utils::postureToTransArray(robot.posture(CoordinateType::endInRef, ec), init_pose);
  if (reportError("posture(endInRef)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  double time = 0.0;
  rt->startMove(RtControllerMode::cartesianImpedance);
  rt->setControlLoop(std::function<CartesianPosition(void)>([&]() {
    time += 0.001;
    CartesianPosition cmd;
    cmd.pos = init_pose;
    cmd.pos[11] += 0.04 * (1.0 - std::cos(kPi * time));
    cmd.syncComponentsFromMatrix();
    if (time > 0.25) {
      cmd.setFinished();
    }
    return cmd;
  }));
  rt->startLoop(true);
  os << "rt cartesian impedance loop finished" << std::endl;

  printSection("3 恢复并断开连接");
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
