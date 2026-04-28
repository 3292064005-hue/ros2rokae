#include <system_error>

#include "rokae/robot.h"

int main() {
  std::error_code ec;
  rokae::xMateRobot robot("127.0.0.1");
  (void)robot.sdkVersion();
  return ec ? 1 : 0;
}
