#include "runtime/service_facade.hpp"

#include "runtime/runtime_catalog_service.hpp"

namespace rokae_xmate3_ros2::runtime {

void QueryFacade::handleGetRlProjectInfo(const rokae_xmate3_ros2::srv::GetRlProjectInfo::Request &req,
                                         rokae_xmate3_ros2::srv::GetRlProjectInfo::Response &res) const {
  (void)req;
  const auto projects = buildRuntimeProjectCatalog(program_state_);
  for (const auto &entry : projects) {
    res.project_names.push_back(entry.name);
    res.is_running.push_back(entry.is_running);
    res.run_rates.push_back(entry.run_rate);
    res.loop_modes.push_back(entry.loop_mode);
    if (entry.active) {
      res.active_project_name = entry.name;
      res.current_episode = entry.current_episode;
    }
  }
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void QueryFacade::handleGetToolCatalog(const rokae_xmate3_ros2::srv::GetToolCatalog::Request &req,
                                       rokae_xmate3_ros2::srv::GetToolCatalog::Response &res) const {
  (void)req;
  const auto tools = buildRuntimeToolCatalog(tooling_state_);
  for (const auto &entry : tools) {
    res.names.push_back(entry.name);
    res.aliases.push_back(entry.alias);
    res.robot_held.push_back(entry.robot_held);
    res.masses.push_back(entry.mass);
    for (double value : entry.pose) {
      res.pose_flattened.push_back(value);
    }
  }
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void QueryFacade::handleGetWobjCatalog(const rokae_xmate3_ros2::srv::GetWobjCatalog::Request &req,
                                       rokae_xmate3_ros2::srv::GetWobjCatalog::Response &res) const {
  (void)req;
  const auto wobjs = buildRuntimeWobjCatalog(tooling_state_);
  for (const auto &entry : wobjs) {
    res.names.push_back(entry.name);
    res.aliases.push_back(entry.alias);
    res.robot_held.push_back(entry.robot_held);
    for (double value : entry.pose) {
      res.pose_flattened.push_back(value);
    }
  }
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

}  // namespace rokae_xmate3_ros2::runtime
