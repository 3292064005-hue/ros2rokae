#include <system_error>

#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  std::error_code ec;
  std::array<double[2], 6> limits{};
  (void)robot.getSoftLimit(limits, ec);
  robot.setSoftLimit(true, ec);
  robot.setSoftLimit(false, ec, limits);
  return 0;
}
