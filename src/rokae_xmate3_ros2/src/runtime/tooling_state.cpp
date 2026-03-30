#include "runtime/runtime_state.hpp"

#include <algorithm>

#include "runtime/runtime_state_utils.hpp"

namespace rokae_xmate3_ros2::runtime {

void ToolingState::setToolset(const std::string &tool_name,
                              const std::string &wobj_name,
                              const std::vector<double> &tool_pose,
                              const std::vector<double> &wobj_pose) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_tool_name_ = tool_name.empty() ? "tool0" : tool_name;
  current_wobj_name_ = wobj_name.empty() ? "wobj0" : wobj_name;
  current_tool_pose_ = detail::normalize_runtime_pose(tool_pose);
  current_wobj_pose_ = detail::normalize_runtime_pose(wobj_pose);
  tool_registry_[current_tool_name_] = current_tool_pose_;
  wobj_registry_[current_wobj_name_] = current_wobj_pose_;
  if (tool_mass_registry_.find(current_tool_name_) == tool_mass_registry_.end()) {
    tool_mass_registry_[current_tool_name_] = 0.0;
  }
  if (tool_com_registry_.find(current_tool_name_) == tool_com_registry_.end()) {
    tool_com_registry_[current_tool_name_] = {0.0, 0.0, 0.0};
  }
  current_tool_mass_ = tool_mass_registry_[current_tool_name_];
  current_tool_com_ = tool_com_registry_[current_tool_name_];
  base_pose_ = current_wobj_pose_;
}

bool ToolingState::setToolsetByName(const std::string &tool_name, const std::string &wobj_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto resolved_tool_name = tool_name.empty() ? current_tool_name_ : tool_name;
  const auto resolved_wobj_name = wobj_name.empty() ? current_wobj_name_ : wobj_name;
  const auto tool_it = tool_registry_.find(resolved_tool_name);
  const auto wobj_it = wobj_registry_.find(resolved_wobj_name);
  if (tool_it == tool_registry_.end() || wobj_it == wobj_registry_.end()) {
    return false;
  }
  current_tool_name_ = resolved_tool_name;
  current_wobj_name_ = resolved_wobj_name;
  current_tool_pose_ = tool_it->second;
  current_wobj_pose_ = wobj_it->second;
  current_tool_mass_ = tool_mass_registry_[current_tool_name_];
  current_tool_com_ = tool_com_registry_[current_tool_name_];
  base_pose_ = current_wobj_pose_;
  return true;
}

void ToolingState::setToolDynamics(const std::string &tool_name,
                                   double mass,
                                   const std::array<double, 3> &com) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string resolved_tool_name = tool_name.empty() ? current_tool_name_ : tool_name;
  tool_mass_registry_[resolved_tool_name] = std::max(0.0, mass);
  tool_com_registry_[resolved_tool_name] = com;
  if (resolved_tool_name == current_tool_name_) {
    current_tool_mass_ = tool_mass_registry_[resolved_tool_name];
    current_tool_com_ = tool_com_registry_[resolved_tool_name];
  }
}

ToolsetSnapshot ToolingState::toolset() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ToolsetSnapshot{
      current_tool_name_,
      current_wobj_name_,
      current_tool_pose_,
      current_wobj_pose_,
      base_pose_,
      current_tool_mass_,
      current_tool_com_,
  };
}

void ToolingState::setBaseFrame(const std::vector<double> &base_pose) {
  std::lock_guard<std::mutex> lock(mutex_);
  base_pose_ = detail::normalize_runtime_pose(base_pose);
}

std::vector<double> ToolingState::baseFrame() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return base_pose_;
}


std::vector<rokae::WorkToolInfo> ToolingState::toolCatalog() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rokae::WorkToolInfo> tools;
  tools.reserve(tool_registry_.size());
  for (const auto &entry : tool_registry_) {
    rokae::WorkToolInfo info;
    info.name = entry.first;
    info.alias = entry.first;
    info.robotHeld = true;
    info.pos = rokae::Frame({entry.second[0], entry.second[1], entry.second[2], entry.second[3], entry.second[4], entry.second[5]});
    info.load.mass = tool_mass_registry_.count(entry.first) ? tool_mass_registry_.at(entry.first) : 0.0;
    if (tool_com_registry_.count(entry.first)) {
      info.load.cog = tool_com_registry_.at(entry.first);
    }
    tools.push_back(info);
  }
  return tools;
}

std::vector<rokae::WorkToolInfo> ToolingState::wobjCatalog() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rokae::WorkToolInfo> wobjs;
  wobjs.reserve(wobj_registry_.size());
  for (const auto &entry : wobj_registry_) {
    rokae::WorkToolInfo info;
    info.name = entry.first;
    info.alias = entry.first;
    info.robotHeld = false;
    info.pos = rokae::Frame({entry.second[0], entry.second[1], entry.second[2], entry.second[3], entry.second[4], entry.second[5]});
    wobjs.push_back(info);
  }
  return wobjs;
}


}  // namespace rokae_xmate3_ros2::runtime
