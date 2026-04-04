#include <chrono>
#include <system_error>
#include <thread>

#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  std::error_code ec;

  for (int i = 0; i < 40; ++i) {
    robot.connectToRobot("127.0.0.1", "", ec);
    if (!ec) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (ec) {
    return 1;
  }

  robot.setPowerState(true, ec);
  if (ec) {
    return 2;
  }
  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  if (ec) {
    return 3;
  }
  robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
  if (ec) {
    return 4;
  }
  robot.moveReset(ec);
  if (ec) {
    return 5;
  }
  const auto joints = robot.jointPos(ec);
  (void)joints;
  if (ec) {
    return 6;
  }
  return 0;
}
