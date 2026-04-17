#include <array>
#include "rokae/planner.h"

int main() {
  rokae::JointMotionGenerator planner(0.2, std::array<double, 6>{});
  planner.calculateSynchronizedValues(std::array<double, 6>{});
  std::array<double, 6> delta{};
  (void)planner.calculateDesiredValues(0.0, delta);
  return 0;
}
