#include <system_error>

#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  std::error_code ec;
  (void)robot.powerState(ec);
  (void)robot.operateMode(ec);
  (void)robot.operationState(ec);
  robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
  robot.moveReset(ec);
  return 0;
}
