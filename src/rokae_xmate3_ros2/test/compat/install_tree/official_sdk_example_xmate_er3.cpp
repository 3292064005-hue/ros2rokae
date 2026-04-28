#include <system_error>

#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  std::error_code ec;

  (void)robot.robotInfo(ec);
  (void)robot.jointPos(ec);
  (void)robot.jointVel(ec);
  (void)robot.jointTorque(ec);
  (void)robot.posture(rokae::CoordinateType::endInRef, ec);
  (void)robot.flangePos(ec);

  rokae::Toolset toolset;
  robot.setToolset(toolset, ec);
  (void)robot.setToolset("tool0", "wobj0", ec);

  return 0;
}
