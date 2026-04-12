/**
 * @file 26_rt_torque_control.cpp
 * @brief 外部 rt/torque_control 的 xMate3 版
 *
 * Gazebo shim 当前会确认并调度 torque loop，但不会像真机那样真实执行关节力矩闭环。
 * 因此这里把示例做成“短时 smoke + 模型/状态链路验证”版本：
 * 1. 运动到 RT 起始位；
 * 2. 读取状态并计算模型力矩；
 * 3. 发送短时 zero-torque loop；
 * 4. 发送短时 model-based torque loop。
 */

#include <array>
#include <cmath>
#include <functional>
#include <vector>

#include "rokae/model.h"
#include "rokae/motion_control_rt.h"
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

namespace {

constexpr double kZeroTorqueDuration = 0.15;
constexpr double kModelTorqueDuration = 0.25;
constexpr double kWaveAmplitude = 0.35;

using xMateRtController = RtMotionControl<WorkType::collaborative, 6>;

void fillStateSnapshot(xMateRobot &robot,
                       std::array<double, 6> &q,
                       std::array<double, 6> &dq,
                       std::array<double, 6> &ddq) {
  using namespace RtSupportedFields;
  robot.getStateData(jointPos_m, q);
  robot.getStateData(jointVel_m, dq);
  robot.getStateData(jointAcc_c, ddq);
}

template <typename Callback>
bool runTorqueLoop(xMateRobot &robot,
                   const std::shared_ptr<xMateRtController> &rt,
                   const std::string &label,
                   Callback callback,
                   bool use_state_data_in_loop) {
  rt->startMove(RtControllerMode::torque);
  if (const auto start_ec = robot.lastErrorCode(); reportError(label + " startMove", start_ec)) {
    return false;
  }

  rt->setControlLoop(std::function<Torque(void)>(callback), 0, use_state_data_in_loop);
  rt->startLoop(true);

  if (const auto loop_ec = robot.lastErrorCode(); reportError(label, loop_ec)) {
    return false;
  }

  os << label << " finished" << std::endl;
  return true;
}

}  // namespace

int main() {
  printHeader("示例 26: RT 力矩控制", "torque_control Gazebo smoke 版");
  os << "note: torque control in Gazebo is simulation-only and approximate" << std::endl;
  os << "rt profile: 1kHz control period (" << kRtControlPeriod.count() << " ms)" << std::endl;

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticRt(robot, ec, 20)) {
    return isSimulationOnlyCapabilityError(ec)
               ? skipExample(robot, "RT torque facade unavailable in current simulation backend: " + ec.message())
               : (cleanupRobot(robot), 1);
  }

  auto rt = robot.getRtMotionController().lock();
  if (!rt) {
    return skipExample(robot, "RT torque controller unavailable in current simulation backend");
  }

  printSection("1 MoveJ 到 RT 起始位");
  const auto current = robot.jointPos(ec);
  if (reportError("jointPos", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  rt->MoveJ(0.2, current, kXMate3DragPose);
  if (const auto move_ec = robot.lastErrorCode(); reportError("rt MoveJ", move_ec)) {
    return isSimulationOnlyCapabilityError(move_ec)
               ? skipExample(robot, "RT torque MoveJ unavailable in current simulation backend: " + move_ec.message())
               : (cleanupRobot(robot), 1);
  }
  os << "rt MoveJ finished" << std::endl;

  printSection("2 读取状态并计算模型力矩");
  robot.startReceiveRobotState(kRtControlPeriod,
                               {RtSupportedFields::jointPos_m,
                                RtSupportedFields::jointVel_m,
                                RtSupportedFields::jointAcc_c});
  robot.updateRobotState(kRtControlPeriod);

  auto model = robot.model();
  std::array<double, 6> q{};
  std::array<double, 6> dq{};
  std::array<double, 6> ddq{};
  fillStateSnapshot(robot, q, dq, ddq);
  const auto gravity = model.getTorque(q, dq, ddq, TorqueType::gravity);
  const auto friction = model.getTorque(q, dq, ddq, TorqueType::friction);
  printArray("jointPos_m", q, 4, " rad");
  printArray("gravity torque", gravity, 4, " Nm");
  printArray("friction torque", friction, 4, " Nm");

  printSection("3 Zero-torque loop smoke");
  if (!runTorqueLoop(robot, rt, "zero torque loop", [time = 0.0]() mutable {
        time += kRtControlDtSec;
        Torque cmd;
        if (time > kZeroTorqueDuration) {
          cmd.setFinished();
        }
        return cmd;
      }, false)) {
    robot.stopReceiveRobotState();
    const auto loop_ec = robot.lastErrorCode();
    if (isSimulationOnlyCapabilityError(loop_ec)) {
      return skipExample(robot, "RT zero-torque loop unavailable in current simulation backend: " + loop_ec.message());
    }
    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    cleanupRobot(robot);
    return 1;
  }

  printSection("4 Model-based torque loop smoke");
  std::array<double, 6> sample_gravity = gravity;
  if (!runTorqueLoop(robot, rt, "model torque loop", [&]() {
        static double time = 0.0;
        time += kRtControlDtSec;

        fillStateSnapshot(robot, q, dq, ddq);
        sample_gravity = model.getTorque(q, dq, ddq, TorqueType::gravity);

        Torque cmd;
        cmd.tau.assign(sample_gravity.begin(), sample_gravity.end());
        const double wave = kWaveAmplitude * std::sin(2.0 * kPi * 2.0 * time);
        cmd.tau[1] += wave;
        cmd.tau[2] -= 0.5 * wave;
        if (time > kModelTorqueDuration) {
          cmd.setFinished();
        }
        return cmd;
      }, true)) {
    robot.stopReceiveRobotState();
    const auto loop_ec = robot.lastErrorCode();
    if (isSimulationOnlyCapabilityError(loop_ec)) {
      return skipExample(robot, "RT model-based torque loop unavailable in current simulation backend: " + loop_ec.message());
    }
    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    cleanupRobot(robot);
    return 1;
  }
  printArray("last gravity torque sample", sample_gravity, 4, " Nm");
  os << "Gazebo shim: torque loop acknowledged, but no true torque physics is applied" << std::endl;

  printSection("5 恢复并断开连接");
  robot.stopReceiveRobotState();
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
