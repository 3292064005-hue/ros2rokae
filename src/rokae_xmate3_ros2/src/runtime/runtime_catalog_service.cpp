#include "runtime/runtime_catalog_service.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <map>
#include <sstream>

namespace rokae_xmate3_ros2::runtime {
namespace {

template <typename Entry>
void sort_catalog_with_active_last(std::vector<Entry> &entries) {
  std::sort(entries.begin(), entries.end(), [](const Entry &lhs, const Entry &rhs) {
    if (lhs.active != rhs.active) {
      return !lhs.active && rhs.active;
    }
    return lhs.name < rhs.name;
  });
}

std::string bool_text(bool value) {
  return value ? "true" : "false";
}

std::string double_text(double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

std::array<double, 6> frame_to_pose_array(const rokae::Frame &frame) {
  return {frame.x, frame.y, frame.z, frame.rx, frame.ry, frame.rz};
}

void ensure_active_pose_entry(std::vector<RuntimeToolCatalogEntry> &entries,
                              const std::string &name,
                              const std::vector<double> &pose,
                              bool robot_held,
                              double mass,
                              const std::string &source) {
  auto it = std::find_if(entries.begin(), entries.end(), [&](const RuntimeToolCatalogEntry &entry) {
    return entry.name == name;
  });
  if (it == entries.end()) {
    RuntimeToolCatalogEntry entry;
    entry.name = name;
    entry.alias = name;
    entry.robot_held = robot_held;
    entry.mass = mass;
    for (size_t i = 0; i < 6 && i < pose.size(); ++i) {
      entry.pose[i] = pose[i];
    }
    entry.active = true;
    entry.source = source;
    entries.push_back(entry);
    return;
  }
  it->active = true;
  it->robot_held = robot_held;
  it->mass = mass;
  for (size_t i = 0; i < 6 && i < pose.size(); ++i) {
    it->pose[i] = pose[i];
  }
  it->source = source;
}

}  // namespace

std::vector<RuntimeToolCatalogEntry> buildRuntimeToolCatalog(const ToolingState &tooling_state) {
  std::vector<RuntimeToolCatalogEntry> entries;
  const auto catalog = tooling_state.toolCatalog();
  entries.reserve(catalog.size() + 1);
  for (const auto &info : catalog) {
    RuntimeToolCatalogEntry entry;
    entry.name = info.name;
    entry.alias = info.alias.empty() ? info.name : info.alias;
    entry.robot_held = info.robotHeld;
    entry.mass = info.load.mass;
    entry.pose = frame_to_pose_array(info.pos);
    entry.source = "tool_catalog";
    entries.push_back(entry);
  }

  const auto toolset = tooling_state.toolset();
  ensure_active_pose_entry(entries,
                           toolset.tool_name.empty() ? std::string("tool0") : toolset.tool_name,
                           toolset.tool_pose,
                           true,
                           toolset.tool_mass,
                           "toolset_active");
  sort_catalog_with_active_last(entries);
  return entries;
}

std::vector<RuntimeToolCatalogEntry> buildRuntimeWobjCatalog(const ToolingState &tooling_state) {
  std::vector<RuntimeToolCatalogEntry> entries;
  const auto catalog = tooling_state.wobjCatalog();
  entries.reserve(catalog.size() + 1);
  for (const auto &info : catalog) {
    RuntimeToolCatalogEntry entry;
    entry.name = info.name;
    entry.alias = info.alias.empty() ? info.name : info.alias;
    entry.robot_held = info.robotHeld;
    entry.mass = 0.0;
    entry.pose = frame_to_pose_array(info.pos);
    entry.source = "wobj_catalog";
    entries.push_back(entry);
  }

  const auto toolset = tooling_state.toolset();
  ensure_active_pose_entry(entries,
                           toolset.wobj_name.empty() ? std::string("wobj0") : toolset.wobj_name,
                           toolset.wobj_pose,
                           false,
                           0.0,
                           "toolset_active");
  sort_catalog_with_active_last(entries);
  return entries;
}

std::vector<RuntimeProjectCatalogEntry> buildRuntimeProjectCatalog(const ProgramState &program_state) {
  std::vector<RuntimeProjectCatalogEntry> entries;
  const auto catalog = program_state.rlProjectCatalog();
  entries.reserve(catalog.size() + 1);
  for (const auto &info : catalog) {
    RuntimeProjectCatalogEntry entry;
    entry.name = info.name;
    entry.is_running = info.is_running;
    entry.run_rate = info.run_rate;
    entry.loop_mode = info.loop_mode;
    entry.source = "project_catalog";
    entries.push_back(entry);
  }

  const auto active_name = program_state.loadedRlProjectName();
  if (!active_name.empty()) {
    auto it = std::find_if(entries.begin(), entries.end(), [&](const RuntimeProjectCatalogEntry &entry) {
      return entry.name == active_name;
    });
    if (it == entries.end()) {
      RuntimeProjectCatalogEntry entry;
      entry.name = active_name;
      entry.is_running = program_state.rlProjectRunning();
      entry.run_rate = program_state.rlRunRate();
      entry.loop_mode = program_state.rlLoopMode();
      entry.active = true;
      entry.current_episode = program_state.rlCurrentEpisode();
      entry.source = program_state.rlProjectLoaded() ? "loaded_project" : "runtime_fallback";
      entries.push_back(entry);
    } else {
      it->active = true;
      it->is_running = program_state.rlProjectRunning();
      it->run_rate = program_state.rlRunRate();
      it->loop_mode = program_state.rlLoopMode();
      it->current_episode = program_state.rlCurrentEpisode();
      it->source = program_state.rlProjectLoaded() ? "loaded_project" : it->source;
    }
  }

  std::sort(entries.begin(), entries.end(), [](const auto &lhs, const auto &rhs) { return lhs.name < rhs.name; });
  return entries;
}

std::vector<RuntimeRegisterCatalogEntry> buildRuntimeRegisterCatalog(const DataStoreState &data_store_state) {
  std::map<std::string, RuntimeRegisterCatalogEntry> entries;
  for (const auto &key : data_store_state.registerKeys()) {
    const auto open = key.find('[');
    const auto close = key.rfind(']');
    if (open != std::string::npos && close == key.size() - 1 && close > open + 1) {
      const auto base = key.substr(0, open);
      const auto index_text = key.substr(open + 1, close - open - 1);
      const bool numeric_index = !index_text.empty() &&
          std::all_of(index_text.begin(), index_text.end(), [](unsigned char ch) { return std::isdigit(ch) != 0; });
      auto &entry = entries[base];
      entry.name = base;
      entry.indexed = numeric_index;
      entry.element_count = std::max<std::size_t>(entry.element_count, numeric_index ? static_cast<std::size_t>(std::stoul(index_text) + 1) : 1U);
      entry.simulation_only = true;
      continue;
    }
    auto &entry = entries[key];
    entry.name = key;
    entry.indexed = false;
    entry.element_count = std::max<std::size_t>(entry.element_count, 1U);
  }

  std::vector<RuntimeRegisterCatalogEntry> ordered;
  ordered.reserve(entries.size());
  for (const auto &entry : entries) {
    ordered.push_back(entry.second);
  }
  return ordered;
}


std::vector<RuntimeOptionCatalogEntry> buildRuntimeOptionCatalog(const MotionOptionsState &motion_options_state,
                                                                 const SessionState &session_state) {
  std::vector<RuntimeOptionCatalogEntry> entries;
  entries.reserve(9);

  RuntimeOptionCatalogEntry speed;
  speed.name = "default_speed";
  speed.value = double_text(motion_options_state.defaultSpeed());
  entries.push_back(speed);

  RuntimeOptionCatalogEntry zone;
  zone.name = "default_zone";
  zone.value = std::to_string(motion_options_state.defaultZone());
  entries.push_back(zone);

  RuntimeOptionCatalogEntry scale;
  scale.name = "speed_scale";
  scale.value = double_text(motion_options_state.speedScale());
  entries.push_back(scale);

  RuntimeOptionCatalogEntry strict_conf;
  strict_conf.name = "strict_conf";
  strict_conf.value = bool_text(motion_options_state.defaultConfOptForced());
  entries.push_back(strict_conf);

  RuntimeOptionCatalogEntry avoid_singularity;
  avoid_singularity.name = "avoid_singularity";
  avoid_singularity.value = bool_text(motion_options_state.avoidSingularityEnabled());
  entries.push_back(avoid_singularity);

  const auto soft_limit = motion_options_state.softLimit();
  RuntimeOptionCatalogEntry soft_limit_entry;
  soft_limit_entry.name = "soft_limit_enabled";
  soft_limit_entry.value = bool_text(soft_limit.enabled);
  entries.push_back(soft_limit_entry);

  RuntimeOptionCatalogEntry sim_mode;
  sim_mode.name = "simulation_mode";
  sim_mode.value = bool_text(session_state.simulationMode());
  entries.push_back(sim_mode);

  RuntimeOptionCatalogEntry motion_mode;
  motion_mode.name = "motion_mode";
  motion_mode.value = std::to_string(session_state.motionMode());
  motion_mode.mutability = "session";
  motion_mode.source = "session_state";
  entries.push_back(motion_mode);

  RuntimeOptionCatalogEntry rt_mode;
  rt_mode.name = "rt_control_mode";
  rt_mode.value = std::to_string(session_state.rtControlMode());
  rt_mode.mutability = "session";
  rt_mode.source = "session_state";
  entries.push_back(rt_mode);

  return entries;
}

std::string summarizeRuntimeOptionCatalog(const std::vector<RuntimeOptionCatalogEntry> &entries) {
  std::ostringstream stream;
  bool first = true;
  for (const auto &entry : entries) {
    if (!first) {
      stream << "; ";
    }
    first = false;
    stream << entry.name << '=' << entry.value;
  }
  return stream.str();
}

}  // namespace rokae_xmate3_ros2::runtime
