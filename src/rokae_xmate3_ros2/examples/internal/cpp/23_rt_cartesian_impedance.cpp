/**
 * @file 23_rt_cartesian_impedance.cpp
 * @brief 外部 rt/cartesian_impedance_control 的 xMate3 版
 */

#include <array>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "rokae/motion_control_rt.h"
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "rokae_xmate3_ros2/srv/get_runtime_diagnostics.hpp"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

namespace {

std::string queryRtStateSource() {
  if (!rclcpp::ok()) {
    return "unknown";
  }
  using Service = rokae_xmate3_ros2::srv::GetRuntimeDiagnostics;
  auto node = rclcpp::Node::make_shared("rt_cartesian_impedance_diagnostics_client");
  auto client = node->create_client<Service>("/xmate_er3/cobot/get_runtime_diagnostics");
  if (!client->wait_for_service(std::chrono::seconds(2))) {
    return "unknown";
  }
  auto request = std::make_shared<Service::Request>();
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(3)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return "unknown";
  }
  const auto response = future.get();
  if (!response || !response->success) {
    return "unknown";
  }
  return response->diagnostics.rt_state_source;
}

}  // namespace

int main() {
  printHeader("示例 23: RT 笛卡尔阻抗控制", "官方 SDK 风格");
  os << "note: 当前为仿真运行链路，接口调用方式与官方 SDK 一致" << std::endl;
  os << "rt profile: 1kHz control period (" << kRtControlPeriod.count() << " ms)" << std::endl;

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticRt(robot, ec, 20)) {
    return isSimulationOnlyCapabilityError(ec)
               ? skipExample(robot, "RT cartesian impedance facade unavailable in current simulation backend: " + ec.message())
               : (cleanupRobot(robot), 1);
  }

  auto rt = robot.getRtMotionController().lock();
  if (!rt) {
    return skipExample(robot, "RT cartesian impedance controller unavailable in current simulation backend");
  }
  robot.startReceiveRobotState(kRtControlPeriod, {RtSupportedFields::tcpPose_m});

  printSection("1 配置阻抗参数");
  rt->MoveJ(0.3, robot.jointPos(ec), kXMate3DragPose);
  if (const auto movej_ec = robot.lastErrorCode(); movej_ec) {
    robot.stopReceiveRobotState();
    if (isSimulationOnlyCapabilityError(movej_ec)) {
      return skipExample(robot, "RT cartesian impedance MoveJ unavailable in current simulation backend: " + movej_ec.message());
    }
    reportError("rt MoveJ", movej_ec);
    cleanupRobot(robot);
    return 1;
  }
  rt->setFcCoor({1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1},
                FrameType::tool,
                ec);
  if (reportError("setFcCoor", ec)) {
    robot.stopReceiveRobotState();
    return isSimulationOnlyCapabilityError(ec)
               ? skipExample(robot, "Cartesian force frame configuration unavailable in current simulation backend: " + ec.message())
               : (cleanupRobot(robot), 1);
  }
  rt->setCartesianImpedance({1200.0, 1200.0, 200.0, 100.0, 100.0, 20.0}, ec);
  if (reportError("setCartesianImpedance", ec)) {
    robot.stopReceiveRobotState();
    return isSimulationOnlyCapabilityError(ec)
               ? skipExample(robot, "Cartesian impedance configuration unavailable in current simulation backend: " + ec.message())
               : (cleanupRobot(robot), 1);
  }
  CartesianForceControlParam force_control;
  force_control.enabled = true;
  force_control.kp = {0.30, 0.30, 0.30, 0.06, 0.06, 0.06};
  force_control.ki = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  force_control.deadband = {0.30, 0.30, 0.30, 0.03, 0.03, 0.03};
  force_control.max_feedback_wrench = {6.0, 6.0, 6.0, 1.5, 1.5, 1.5};
  force_control.cutoff_frequency_hz = 20.0;
  force_control.integral_limit = {2.0, 2.0, 2.0, 0.5, 0.5, 0.5};
  rt->setCartesianForceControl(force_control, ec);
  if (reportError("setCartesianForceControl", ec)) {
    robot.stopReceiveRobotState();
    return isSimulationOnlyCapabilityError(ec)
               ? skipExample(robot, "Cartesian force control configuration unavailable in current simulation backend: " + ec.message())
               : (cleanupRobot(robot), 1);
  }
  rt->setCartesianImpedanceDesiredTorque({3.0, 0.0, 3.0, 0.0, 0.0, 0.0}, ec);
  if (reportError("setCartesianImpedanceDesiredTorque", ec)) {
    robot.stopReceiveRobotState();
    return isSimulationOnlyCapabilityError(ec)
               ? skipExample(robot, "Cartesian desired torque configuration unavailable in current simulation backend: " + ec.message())
               : (cleanupRobot(robot), 1);
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
    time += kRtControlDtSec;
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
  robot.stopReceiveRobotState();
  if (const auto loop_ec = robot.lastErrorCode(); loop_ec) {
    if (isSimulationOnlyCapabilityError(loop_ec)) {
      return skipExample(robot, "RT cartesian impedance loop unavailable in current simulation backend: " + loop_ec.message());
    }
    reportError("rt cartesian impedance loop", loop_ec);
    cleanupRobot(robot);
    return 1;
  }
  os << "rt cartesian impedance loop finished" << std::endl;
  os << "force control wrench source: " << queryRtStateSource() << std::endl;

  printSection("3 恢复并断开连接");
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
