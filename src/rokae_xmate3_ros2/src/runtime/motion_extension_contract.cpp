#include "runtime/motion_extension_contract.hpp"

#include <algorithm>
#include <sstream>

namespace rokae_xmate3_ros2::runtime {
namespace {
const char *kindName(MotionKind kind) noexcept {
  switch (kind) {
    case MotionKind::move_absj: return "MoveAbsJ";
    case MotionKind::move_j: return "MoveJ";
    case MotionKind::move_l: return "MoveL";
    case MotionKind::move_c: return "MoveC";
    case MotionKind::move_cf: return "MoveCF";
    case MotionKind::move_sp: return "MoveSP";
    case MotionKind::none:
    default: return "none";
  }
}
}  // namespace

std::vector<MotionExtensionContract> buildMotionExtensionContracts() {
  return {
      {MotionKind::move_absj, "MoveAppend/executeCommand", "joint_trajectory", "trajectory_goal_builder", true, false},
      {MotionKind::move_j, "MoveAppend/executeCommand", "cartesian_to_joint_plan", "trajectory_goal_builder", true, false},
      {MotionKind::move_l, "MoveAppend/executeCommand", "cartesian_line_plan", "trajectory_goal_builder", true, false},
      {MotionKind::move_c, "MoveAppend/executeCommand", "cartesian_arc_plan", "trajectory_goal_builder", true, false},
      {MotionKind::move_cf, "MoveAppend/executeCommand", "continuous_circle_plan", "trajectory_goal_builder", true, false},
      {MotionKind::move_sp, "MoveAppend/executeCommand", "spiral_plan", "trajectory_goal_builder", false, true},
  };
}

std::string summarizeMotionExtensionContracts(const std::vector<MotionExtensionContract> &contracts) {
  std::ostringstream stream;
  stream << "extension_contracts[";
  bool first = true;
  for (const auto &entry : contracts) {
    if (!first) stream << ',';
    first = false;
    stream << kindName(entry.kind)
           << "=surface:" << entry.request_surface
           << "|planner:" << entry.planner_output
           << "|backend:" << entry.backend_capability
           << "|public:" << (entry.public_xmate6 ? "true" : "false")
           << "|experimental:" << (entry.experimental ? "true" : "false");
  }
  stream << ']';
  return stream.str();
}

bool validateMotionExtensionContracts(const std::vector<MotionExtensionContract> &contracts, std::string &error) {
  const MotionKind required_public[] = {
      MotionKind::move_absj, MotionKind::move_j, MotionKind::move_l, MotionKind::move_c, MotionKind::move_cf};
  for (const auto kind : required_public) {
    const auto it = std::find_if(contracts.begin(), contracts.end(), [kind](const MotionExtensionContract &entry) {
      return entry.kind == kind;
    });
    if (it == contracts.end()) {
      error = std::string{"missing motion extension contract for "} + kindName(kind);
      return false;
    }
    if (!it->public_xmate6) {
      error = std::string{"required public motion contract hidden for "} + kindName(kind);
      return false;
    }
    if (it->experimental) {
      error = std::string{"required public motion contract marked experimental for "} + kindName(kind);
      return false;
    }
  }
  return true;
}

}  // namespace rokae_xmate3_ros2::runtime
