#ifndef ROKAE_XMATE3_ROS2_RUNTIME_TOOLING_STATE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_TOOLING_STATE_HPP

#include <array>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "runtime/runtime_snapshots.hpp"
#include "rokae_xmate3_ros2/types.hpp"

namespace rokae_xmate3_ros2::runtime {

class ToolingState {
 public:
  void setToolset(const std::string &tool_name,
                  const std::string &wobj_name,
                  const std::vector<double> &tool_pose,
                  const std::vector<double> &wobj_pose);
  [[nodiscard]] bool setToolsetByName(const std::string &tool_name, const std::string &wobj_name);
  void setToolDynamics(const std::string &tool_name, double mass, const std::array<double, 3> &com);
  [[nodiscard]] ToolsetSnapshot toolset() const;
  void setBaseFrame(const std::vector<double> &base_pose);
  [[nodiscard]] std::vector<double> baseFrame() const;
  [[nodiscard]] std::vector<rokae::WorkToolInfo> toolCatalog() const;
  [[nodiscard]] std::vector<rokae::WorkToolInfo> wobjCatalog() const;

 private:
  mutable std::mutex mutex_;
  std::string current_tool_name_{"tool0"};
  std::string current_wobj_name_{"wobj0"};
  std::vector<double> current_tool_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> current_wobj_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> base_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double current_tool_mass_ = 0.0;
  std::array<double, 3> current_tool_com_{{0.0, 0.0, 0.0}};
  std::map<std::string, std::vector<double>> tool_registry_{{"tool0", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}};
  std::map<std::string, std::vector<double>> wobj_registry_{{"wobj0", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}};
  std::map<std::string, double> tool_mass_registry_{{"tool0", 0.0}};
  std::map<std::string, std::array<double, 3>> tool_com_registry_{{"tool0", {{0.0, 0.0, 0.0}}}};
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
