#ifndef ROKAE_XMATE3_ROS2_RUNTIME_RUNTIME_CATALOG_SERVICE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_RUNTIME_CATALOG_SERVICE_HPP

#include <array>
#include <string>
#include <vector>

#include "runtime/data_store_state.hpp"
#include "runtime/motion_options_state.hpp"
#include "runtime/program_state.hpp"
#include "runtime/session_state.hpp"
#include "runtime/tooling_state.hpp"

namespace rokae_xmate3_ros2::runtime {

struct RuntimeToolCatalogEntry {
  std::string name;
  std::string alias;
  bool robot_held = true;
  double mass = 0.0;
  std::array<double, 6> pose{};
  bool active = false;
  std::string source{"runtime"};
};


struct RuntimeOptionCatalogEntry {
  std::string name;
  std::string value;
  std::string mutability{"runtime"};
  std::string source{"runtime_options"};
};

struct RuntimeRegisterCatalogEntry {
  std::string name;
  std::string value_kind{"string"};
  bool indexed = false;
  std::size_t element_count = 1;
  bool readable = true;
  bool writable = true;
  bool simulation_only = false;
  std::string source{"runtime_register_bank"};
};

struct RuntimeProjectCatalogEntry {
  std::string name;
  bool is_running = false;
  double run_rate = 1.0;
  bool loop_mode = false;
  bool active = false;
  int current_episode = 0;
  std::string source{"runtime"};
};

[[nodiscard]] std::vector<RuntimeToolCatalogEntry> buildRuntimeToolCatalog(const ToolingState &tooling_state);
[[nodiscard]] std::vector<RuntimeToolCatalogEntry> buildRuntimeWobjCatalog(const ToolingState &tooling_state);
[[nodiscard]] std::vector<RuntimeProjectCatalogEntry> buildRuntimeProjectCatalog(const ProgramState &program_state);
[[nodiscard]] std::vector<RuntimeRegisterCatalogEntry> buildRuntimeRegisterCatalog(const DataStoreState &data_store_state);
[[nodiscard]] std::vector<RuntimeOptionCatalogEntry> buildRuntimeOptionCatalog(const MotionOptionsState &motion_options_state,
                                                                              const SessionState &session_state);
[[nodiscard]] std::string summarizeRuntimeOptionCatalog(const std::vector<RuntimeOptionCatalogEntry> &entries);

}  // namespace rokae_xmate3_ros2::runtime

#endif
