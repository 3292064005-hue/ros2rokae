#include <string>
#include <system_error>

#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  std::error_code ec;
  bool bit_value = false;
  robot.readRegister("register2", 0, bit_value, ec);
  robot.writeRegister("register2", 0, true, ec);
  (void)robot.getDI(0, 0, ec);
  robot.setSimulationMode(true, ec);
  return 0;
}
