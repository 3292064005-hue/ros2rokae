#ifndef ROKAE_XMATE3_ROS2_RUNTIME_PLANNER_TRACE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_PLANNER_TRACE_HPP

#include <string>
#include <vector>

#include "runtime/planner_preflight.hpp"

namespace rokae_xmate3_ros2::runtime {

void appendPlannerTrace(std::vector<std::string> &notes,
                        const PlannerPreflightReport &report,
                        const std::string &request_id);

}  // namespace rokae_xmate3_ros2::runtime

#endif
