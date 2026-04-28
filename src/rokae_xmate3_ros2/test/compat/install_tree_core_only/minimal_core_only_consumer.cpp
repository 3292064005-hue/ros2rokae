#include <array>
#include <system_error>

#include "rokae/model.h"
#include "rokae/planner.h"

int main() {
  std::error_code ec;
  rokae::xMateModel<6> model;
  (void)model.calcFk(std::array<double, 6>{}, ec);
  rokae::JointMotionGenerator planner(0.2, std::array<double, 6>{});
  planner.calculateSynchronizedValues(std::array<double, 6>{});
  std::array<double, 6> delta{};
  (void)planner.calculateDesiredValues(0.0, delta);
  return 0;
}
