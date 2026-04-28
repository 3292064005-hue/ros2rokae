#include <chrono>
#include <system_error>

#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  std::error_code ec;

  try {
    robot.startReceiveRobotState(std::chrono::seconds(1),
                                 {rokae::RtSupportedFields::tcpPose_m,
                                  rokae::RtSupportedFields::tau_m,
                                  rokae::RtSupportedFields::jointPos_m});
    (void)robot.updateRobotState(std::chrono::seconds(1));

    std::array<double, 16> pose{};
    std::array<double, 6> joints{};
    (void)robot.getStateData(rokae::RtSupportedFields::tcpPose_m, pose);
    (void)robot.getStateData(rokae::RtSupportedFields::jointPos_m, joints);
    robot.stopReceiveRobotState();
  } catch (const rokae::Exception &) {
    robot.stopReceiveRobotState();
  }

  (void)ec;
  return 0;
}
