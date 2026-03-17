/**
 * @file 24_rt_follow_position.cpp
 * @brief 外部 rt/follow_joint_position 的 xMate3 版
 */

#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "rokae/planner.h"
#include "rokae/robot.h"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

namespace {

const std::vector<std::array<double, 6>> kFollowPoints{
    {0.0000, 0.5236, 1.0472, 0.0000, 1.5708, 0.0000},
    {0.0532, 0.5260, 1.0436, 0.0000, 1.5721, 0.0532},
    {0.1062, 0.5373, 0.9440, 0.0000, 1.6604, 0.1062},
    {0.0532, 0.5425, 0.8532, 0.0000, 1.7460, 0.0532},
    {0.0000, 0.5274, 0.9593, 0.0000, 1.6549, 0.0000},
};

}  // namespace

int main() {
  printHeader("示例 24: RT 点位跟随", "follow_joint_position 对齐版");

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

  printSection("1 MoveJ 到跟随起始位");
  rt->MoveJ(0.3, robot.jointPos(ec), kXMate3DragPose);

  printSection("2 FollowPosition 更新点位");
  auto model = robot.model();
  FollowPosition<6> follow(robot, model);
  Eigen::Transform<double, 3, Eigen::Isometry> desired = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
  desired.rotate(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
  desired.pretranslate(Eigen::Vector3d(0.40, 0.00, 0.40));
  follow.start(desired);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  follow.setScale(2.0);
  for (const auto &point : kFollowPoints) {
    follow.update(point);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  for (auto it = kFollowPoints.rbegin(); it != kFollowPoints.rend(); ++it) {
    follow.update(*it);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  follow.stop();
  os << "follow position finished" << std::endl;

  printSection("3 恢复并断开连接");
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
