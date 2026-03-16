#include <array>
#include <iostream>

#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  error_code ec;
  robot.connectToRobot(ec);
  robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
  robot.startReceiveRobotState(std::chrono::milliseconds(1), {
      rokae::RtSupportedFields::jointPos_m,
      rokae::RtSupportedFields::jointVel_m,
      rokae::RtSupportedFields::tcpPose_m});
  robot.updateRobotState(std::chrono::milliseconds(1));

  std::array<double, 6> q{};
  robot.getStateData(rokae::RtSupportedFields::jointPos_m, q);

  auto rt = robot.getRtMotionController().lock();
  if (rt) {
    rt->startMove(rokae::RtControllerMode::jointPosition);
    rt->setJointImpedance({100, 100, 100, 50, 50, 50}, ec);
    rt->setCartesianImpedance({200, 200, 200, 20, 20, 20}, ec);
    rt->MoveJ(0.2, q, q);
    rt->stopMove();
  }

  auto model = robot.model();
  auto pose = model.getCartPose(q);
  auto jac = model.jacobian(q);
  auto torque = model.getTorque(q, q, q, rokae::TorqueType::full);

  rokae::JointMotionGenerator joint_planner(0.2, {0, 0, 0, 0, 0, 0, 0});
  joint_planner.calculateSynchronizedValues({0, 0, 0, 0, 0, 0, 0});
  std::array<double, 7> dq{};
  joint_planner.calculateDesiredValues(0.1, dq);

  rokae::CartMotionGenerator cart_planner(0.2, 0.1);
  double ds = 0.0;
  cart_planner.calculateDesiredValues(0.1, &ds);

  rokae::FollowPosition<6> follow(robot, model);
  follow.start(q);
  follow.setScale(0.4);
  follow.update(q);
  follow.stop();

  std::cout << "pose_x=" << pose[3] << ", jac00=" << jac[0] << ", torque0=" << torque[0] << ", ds=" << ds << "\n";
  return 0;
}
