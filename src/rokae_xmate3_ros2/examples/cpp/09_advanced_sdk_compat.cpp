/**
 * @file 09_advanced_sdk_compat.cpp
 * @brief 官方 SDK 风格 - 高级接口演示
 */

#include <array>
#include <chrono>
#include <vector>

#include "rokae/model.h"
#include "rokae/motion_control_rt.h"
#include "rokae/planner.h"
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 9: 高级 SDK 接口", "官方 SDK 风格");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticNrt(robot, ec, 20, 5)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("1 末端力矩与能力边界");
  robot.setAvoidSingularity(true, ec);
  if (ec) {
    os << "setAvoidSingularity: xMate6 当前实现返回 unsupported（符合机型能力边界） -> "
       << ec.message() << std::endl;
    ec.clear();
  }
  os << "skip getAvoidSingularity on xMate6 because the official manual scopes it to xMateCR/xMateSR only"
     << std::endl;

  std::array<double, 6> joint_tau{};
  std::array<double, 6> ext_tau{};
  std::array<double, 3> cart_tau{};
  std::array<double, 3> cart_force{};
  robot.getEndTorque(FrameType::tool, joint_tau, ext_tau, cart_tau, cart_force, ec);
  if (reportError("getEndTorque", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printArray("joint torque measured", joint_tau, 4, " Nm");
  printArray("external torque measured", ext_tau, 4, " Nm");
  printArray("cartesian torque", cart_tau, 4, " Nm");
  printArray("cartesian force", cart_force, 4, " N");

  printSection("2 状态流与事件接口");
  robot.setEventWatcher(Event::moveExecution,
                        [](const EventInfo &info) { printMoveEvent(info); },
                        ec);
  if (reportError("setEventWatcher(moveExecution)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.startReceiveRobotState(std::chrono::milliseconds(8),
                               {RtSupportedFields::jointPos_m,
                                RtSupportedFields::tau_m,
                                RtSupportedFields::tcpPose_m});
  robot.updateRobotState(std::chrono::milliseconds(8));
  std::array<double, 6> q_m{};
  std::array<double, 6> tau_m{};
  std::array<double, 16> tcp_pose_m{};
  if (robot.getStateData(RtSupportedFields::jointPos_m, q_m) == 0) {
    printArray("jointPos_m", q_m, 4, " rad");
  }
  if (robot.getStateData(RtSupportedFields::tau_m, tau_m) == 0) {
    printArray("tau_m", tau_m, 4, " Nm");
  }
  if (robot.getStateData(RtSupportedFields::tcpPose_m, tcp_pose_m) == 0) {
    printArray("tcpPose_m", tcp_pose_m, 4);
  }
  robot.stopReceiveRobotState();

  printSection("3 运行时选项与 public-lane 边界");
  float counter = 42.0F;
  robot.writeRegister("demo_counter", 0, counter, ec);
  if (ec) {
    if (isSimulationOnlyCapabilityError(ec)) {
      printCapabilityStatus("public-lane", "register workflow is internal/backend only on the install-facing xMate6 SDK lane");
      ec.clear();
    } else if (reportError("writeRegister(demo_counter)", ec)) {
      cleanupRobot(robot);
      return 1;
    }
  } else {
    counter = 0.0F;
    robot.readRegister("demo_counter", 0, counter, ec);
    if (reportError("readRegister(demo_counter)", ec)) {
      cleanupRobot(robot);
      return 1;
    }
    os << "demo_counter[0] = " << counter << std::endl;
  }
  robot.setRtNetworkTolerance(15, ec);
  if (reportError("setRtNetworkTolerance", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.useRciClient(false, ec);
  if (reportError("useRciClient(false)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "runtime options updated" << std::endl;

  printSection("4 模型、规划与 RT 控制器");
  auto model = robot.model();
  const auto q = robot.jointPos(ec);
  if (reportError("jointPos", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  const std::array<double, 6> dq{};
  const std::array<double, 6> ddq{};
  const auto gravity = model.getTorque(q, dq, ddq, TorqueType::gravity);
  printArray("gravity torque", gravity, 4, " Nm");
  const auto jac = model.jacobian(q);
  std::array<double, 6> jac_row0{};
  std::copy_n(jac.begin(), 6, jac_row0.begin());
  printArray("jacobian row0", jac_row0, 4);

  CartMotionGenerator cart_s(0.2, 0.10);
  cart_s.setMax(0.2, 0.4, 0.4);
  cart_s.calculateSynchronizedValues(0.0);
  double delta_s = 0.0;
  cart_s.calculateDesiredValues(0.1, &delta_s);
  os << "cartesian planner sample delta_s = " << delta_s << std::endl;

  if (auto rt = robot.getRtMotionController().lock()) {
    rt->setJointImpedance({600, 600, 600, 200, 100, 100}, ec);
    if (reportError("setJointImpedance", ec)) {
      if (isSimulationOnlyCapabilityError(ec)) {
        printCapabilityStatus("approximate", "RT joint impedance configuration skipped in simulation backend");
        ec.clear();
      } else {
        cleanupRobot(robot);
        return 1;
      }
    }
    rt->setFilterFrequency(50.0, 50.0, 50.0, ec);
    if (reportError("setFilterFrequency", ec)) {
      if (isSimulationOnlyCapabilityError(ec)) {
        printCapabilityStatus("approximate", "RT filter frequency configuration skipped in simulation backend");
        ec.clear();
      } else {
        cleanupRobot(robot);
        return 1;
      }
    }
    rt->setTorqueFilterCutOffFrequency(30.0, ec);
    if (reportError("setTorqueFilterCutOffFrequency", ec)) {
      if (isSimulationOnlyCapabilityError(ec)) {
        printCapabilityStatus("approximate", "RT torque filter configuration skipped in simulation backend");
        ec.clear();
      } else {
        cleanupRobot(robot);
        return 1;
      }
    }
    os << "rt controller configured" << std::endl;
  } else {
    printCapabilityStatus("approximate", "rt controller unavailable, skip RT controller configuration");
  }

  printSection("5 最近错误码与断开连接");
  const auto last_error = robot.lastErrorCode();
  os << "last error: " << (last_error ? last_error.message() : "ok") << std::endl;
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
