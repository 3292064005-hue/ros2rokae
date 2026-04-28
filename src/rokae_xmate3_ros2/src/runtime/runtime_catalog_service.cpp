#include "runtime/runtime_catalog_service.hpp"

#include "rokae_xmate3_ros2/runtime/rt_semantic_topics.hpp"

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
                                                                 const SessionState &session_state,
                                                                 const DataStoreState &data_store_state) {
  return buildRuntimeOptionCatalog(
      motion_options_state,
      session_state,
      data_store_state.rtControlSnapshot(),
      data_store_state.rtSemanticSnapshot());
}

std::vector<RuntimeOptionCatalogEntry> buildRuntimeOptionCatalog(const MotionOptionsState &motion_options_state,
                                                                 const SessionState &session_state,
                                                                 const DataStoreState::RtControlSnapshot &rt_snapshot,
                                                                 const DataStoreState::RtSemanticSnapshot &semantic_snapshot) {
  std::vector<RuntimeOptionCatalogEntry> entries;
  entries.reserve(18);

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

  const auto push_snapshot = [&](const std::string &name,
                                 const std::string &value,
                                 bool configured,
                                 const std::string &source = "rt_snapshot") {
    if (!configured) {
      return;
    }
    RuntimeOptionCatalogEntry option;
    option.name = name;
    option.value = value;
    option.mutability = "runtime";
    option.source = source;
    entries.push_back(std::move(option));
  };

  push_snapshot("rt_network_tolerance",
                double_text(std::clamp((rt_snapshot.rt_command_timeout_sec - 0.05) / (2.0 - 0.05) * 100.0, 0.0, 100.0)),
                rt_snapshot.rt_network_tolerance_configured);
  push_snapshot("use_rci_client", bool_text(rt_snapshot.use_rci_client), rt_snapshot.use_rci_client_configured, "rt_snapshot");

  if (rt_snapshot.joint_impedance_configured) {
    RuntimeOptionCatalogEntry option;
    option.name = "joint_impedance";
    option.value = double_text(rt_snapshot.joint_impedance[0]) + "," + double_text(rt_snapshot.joint_impedance[1]) + "," +
                   double_text(rt_snapshot.joint_impedance[2]) + "," + double_text(rt_snapshot.joint_impedance[3]) + "," +
                   double_text(rt_snapshot.joint_impedance[4]) + "," + double_text(rt_snapshot.joint_impedance[5]);
    option.mutability = "runtime";
    option.source = "rt_snapshot";
    entries.push_back(std::move(option));
  }
  if (rt_snapshot.cartesian_impedance_configured) {
    RuntimeOptionCatalogEntry option;
    option.name = "cartesian_impedance";
    option.value = double_text(rt_snapshot.cartesian_impedance[0]) + "," + double_text(rt_snapshot.cartesian_impedance[1]) + "," +
                   double_text(rt_snapshot.cartesian_impedance[2]) + "," + double_text(rt_snapshot.cartesian_impedance[3]) + "," +
                   double_text(rt_snapshot.cartesian_impedance[4]) + "," + double_text(rt_snapshot.cartesian_impedance[5]);
    option.mutability = "runtime";
    option.source = "rt_snapshot";
    entries.push_back(std::move(option));
  }
  if (rt_snapshot.filter_frequency_configured) {
    RuntimeOptionCatalogEntry option;
    option.name = "filter_frequency";
    option.value = double_text(rt_snapshot.filter_frequency[0]) + "," + double_text(rt_snapshot.filter_frequency[1]) + "," +
                   double_text(rt_snapshot.filter_frequency[2]) + "," + double_text(rt_snapshot.filter_frequency[3]) + "," +
                   double_text(rt_snapshot.filter_frequency[4]) + "," + double_text(rt_snapshot.filter_frequency[5]);
    option.mutability = "runtime";
    option.source = "rt_snapshot";
    entries.push_back(std::move(option));
  }
  if (rt_snapshot.filter_limit_configured) {
    RuntimeOptionCatalogEntry option;
    option.name = "filter_limit";
    option.value = std::string{"enabled="} + bool_text(rt_snapshot.filter_limit_enabled) +
                   ",cutoff=" + double_text(rt_snapshot.filter_limit_cutoff_frequency);
    option.mutability = "runtime";
    option.source = "rt_snapshot";
    entries.push_back(std::move(option));
  }
  if (rt_snapshot.cartesian_desired_wrench_configured) {
    RuntimeOptionCatalogEntry option;
    option.name = "cartesian_desired_wrench";
    option.value = double_text(rt_snapshot.cartesian_desired_wrench[0]) + "," + double_text(rt_snapshot.cartesian_desired_wrench[1]) + "," +
                   double_text(rt_snapshot.cartesian_desired_wrench[2]) + "," + double_text(rt_snapshot.cartesian_desired_wrench[3]) + "," +
                   double_text(rt_snapshot.cartesian_desired_wrench[4]) + "," + double_text(rt_snapshot.cartesian_desired_wrench[5]);
    option.mutability = "runtime";
    option.source = "rt_snapshot";
    entries.push_back(std::move(option));
  }
  if (rt_snapshot.collision_thresholds_configured) {
    RuntimeOptionCatalogEntry option;
    option.name = "collision_behaviour_thresholds";
    option.value = double_text(rt_snapshot.collision_thresholds[0]) + "," + double_text(rt_snapshot.collision_thresholds[1]) + "," +
                   double_text(rt_snapshot.collision_thresholds[2]) + "," + double_text(rt_snapshot.collision_thresholds[3]) + "," +
                   double_text(rt_snapshot.collision_thresholds[4]) + "," + double_text(rt_snapshot.collision_thresholds[5]);
    option.mutability = "runtime";
    option.source = "rt_snapshot";
    entries.push_back(std::move(option));
  }
  push_snapshot("torque_cutoff_frequency",
                double_text(rt_snapshot.torque_cutoff_frequency),
                rt_snapshot.torque_cutoff_frequency_configured);
  push_snapshot("force_control_frame",
                std::string{"type="} + std::to_string(static_cast<int>(rt_snapshot.force_control_frame.type)),
                rt_snapshot.force_control_frame.configured,
                "rt_snapshot");
  push_snapshot("rt_dispatch_mode", semantic_snapshot.dispatch_mode, !semantic_snapshot.dispatch_mode.empty(), "rt_semantic");

  return entries;
}

std::string summarizeCatalogProvenance(const DataStoreState &data_store_state) {
  return summarizeCatalogProvenance(data_store_state.rtSemanticSnapshot());
}

std::string summarizeCatalogProvenance(const DataStoreState::RtSemanticSnapshot &semantic_snapshot) {
  return semantic_snapshot.catalog_provenance.empty() ? std::string{"runtime_authoritative"}
                                                      : semantic_snapshot.catalog_provenance;
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
