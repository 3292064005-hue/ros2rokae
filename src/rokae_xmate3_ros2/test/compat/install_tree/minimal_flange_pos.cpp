#include <system_error>

#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  std::error_code ec;
  (void)robot.flangePos(ec);
  return 0;
}
