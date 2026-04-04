#include <system_error>
#include "rokae/model.h"

int main() {
  std::error_code ec;
  rokae::xMateModel<6> model;
  (void)model.calcFk(std::array<double, 6>{}, ec);
  return 0;
}
