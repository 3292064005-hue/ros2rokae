#include <system_error>
#include <vector>

#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  std::error_code ec;

  robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
  robot.moveReset(ec);
  robot.setDefaultSpeed(100, ec);
  robot.setDefaultZone(5, ec);

  rokae::MoveAbsJCommand absj;
  std::string id;
  robot.moveAppend(std::vector<rokae::MoveAbsJCommand>{absj}, id, ec);
  robot.moveStart(ec);
  robot.stop(ec);

  return 0;
}
