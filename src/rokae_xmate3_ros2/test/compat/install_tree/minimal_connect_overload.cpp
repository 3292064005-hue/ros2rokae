#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  robot.connectToRobot("192.168.0.160");
  robot.connectToRobot("192.168.0.160", "192.168.0.22");
  return 0;
}
