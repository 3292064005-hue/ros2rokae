#include "runtime/rt_subscription_plan.hpp"

#include <cmath>
#include <set>
#include <sstream>

#include "runtime/rt_field_registry.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr std::size_t kMaxRtPayloadBytes = 1024;

}  // namespace

std::string RtSubscriptionPlan::summary() const {
  std::ostringstream stream;
  stream << status << " bytes=" << total_bytes << " interval_ms=" << interval_ms << " fields=";
  for (std::size_t i = 0; i < accepted_fields.size(); ++i) {
    if (i != 0) {
      stream << ',';
    }
    stream << accepted_fields[i];
  }
  if (!rejected_fields.empty()) {
    stream << " rejected=";
    for (std::size_t i = 0; i < rejected_fields.size(); ++i) {
      if (i != 0) {
        stream << ',';
      }
      stream << rejected_fields[i];
    }
  }
  return stream.str();
}

RtSubscriptionPlan buildRtSubscriptionPlan(const std::vector<std::string> &requested_fields,
                                           std::chrono::steady_clock::duration interval,
                                           bool use_state_data_in_loop) {
  RtSubscriptionPlan plan;
  plan.use_state_data_in_loop = use_state_data_in_loop;
  plan.interval_ms = std::chrono::duration<double, std::milli>(interval).count();

  if (requested_fields.empty()) {
    plan.status = "no_fields";
    plan.notes.push_back("rt.plan=no_fields");
    return plan;
  }

  std::set<std::string> seen;
  for (const auto &field : requested_fields) {
    if (!seen.insert(field).second) {
      continue;
    }
    const auto *descriptor = findRtFieldDescriptor(field);
    if (descriptor == nullptr) {
      plan.rejected_fields.push_back(field);
      continue;
    }
    plan.accepted_fields.push_back(field);
    plan.total_bytes += descriptor->bytes;
  }

  if (plan.accepted_fields.empty()) {
    plan.status = "unsupported_fields";
    plan.notes.push_back("rt.plan=unsupported_fields");
    return plan;
  }
  if (!plan.rejected_fields.empty()) {
    plan.status = "unsupported_fields";
    plan.notes.push_back("rt.plan=rejected_fields_present");
    return plan;
  }
  if (plan.total_bytes > kMaxRtPayloadBytes) {
    plan.status = "payload_budget_exceeded";
    plan.notes.push_back("rt.plan=payload_budget_exceeded");
    return plan;
  }
  if (use_state_data_in_loop && std::fabs(plan.interval_ms - 1.0) > 1e-6) {
    plan.status = "in_loop_requires_1ms";
    plan.notes.push_back("rt.plan=in_loop_requires_1ms");
    return plan;
  }

  plan.ok = true;
  plan.status = use_state_data_in_loop ? "armed_in_loop" : "armed_polled";
  plan.notes.push_back("rt.plan=" + plan.status);
  plan.notes.push_back("rt.plan.summary=" + summarizeRtFieldSet(plan.accepted_fields));
  return plan;
}

}  // namespace rokae_xmate3_ros2::runtime
