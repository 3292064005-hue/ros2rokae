#include "runtime/planner_trace.hpp"

namespace rokae_xmate3_ros2::runtime {

void appendPlannerTrace(std::vector<std::string> &notes,
                        const PlannerPreflightReport &report,
                        const std::string &request_id) {
  if (!request_id.empty()) {
    notes.push_back("planner_request_id=" + request_id);
  }
  if (!report.request_profile.empty()) {
    notes.push_back("planner_request_profile=" + report.request_profile);
  }
  if (!report.primary_backend.empty()) {
    notes.push_back("planner_primary_backend=" + report.primary_backend);
  }
  if (!report.auxiliary_backend.empty()) {
    notes.push_back("planner_aux_backend=" + report.auxiliary_backend);
  }
  if (!report.retimer_family.empty()) {
    notes.push_back("planner_retimer_family=" + report.retimer_family);
  }
  if (!report.branch_policy.empty()) {
    notes.push_back("planner_branch_policy=" + report.branch_policy);
  }
  if (!report.dominant_motion_kind.empty()) {
    notes.push_back("planner_motion_kind=" + report.dominant_motion_kind);
  }
  notes.push_back(std::string("planner_command_count=") + std::to_string(report.command_count));
  notes.push_back(std::string("planner_strict_conf=") + (report.strict_conf ? "true" : "false"));
  notes.push_back(std::string("planner_avoid_singularity=") + (report.avoid_singularity ? "true" : "false"));
  notes.push_back(std::string("planner_soft_limit_enabled=") + (report.soft_limit_enabled ? "true" : "false"));
  notes.push_back(std::string("planner_fallback_permitted=") + (report.fallback_permitted ? "true" : "false"));
  notes.insert(notes.end(), report.notes.begin(), report.notes.end());
}

}  // namespace rokae_xmate3_ros2::runtime
