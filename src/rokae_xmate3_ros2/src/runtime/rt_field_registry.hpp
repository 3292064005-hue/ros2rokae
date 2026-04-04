#ifndef ROKAE_XMATE3_ROS2_RUNTIME_RT_FIELD_REGISTRY_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_RT_FIELD_REGISTRY_HPP

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace rokae_xmate3_ros2::runtime {

enum class RtFieldValueKind {
  scalar,
  boolean,
  array6,
  matrix16,
};

struct RtFieldDescriptor {
  std::string_view name;
  RtFieldValueKind value_kind = RtFieldValueKind::scalar;
  std::size_t bytes = 0;
  bool in_loop_safe = true;
  bool strict_in_loop_supported = true;
  bool derived = false;
  bool experimental = false;
  std::string_view source_kind;
  std::string_view category;
};

[[nodiscard]] bool isStrictRtInLoopFieldSupported(std::string_view field_name);

[[nodiscard]] const RtFieldDescriptor *findRtFieldDescriptor(std::string_view field_name);
[[nodiscard]] bool isRtFieldSupported(std::string_view field_name);
[[nodiscard]] std::size_t rtFieldBytes(std::string_view field_name);
[[nodiscard]] std::vector<std::string> defaultRtFieldSet();
[[nodiscard]] std::vector<RtFieldDescriptor> supportedRtFields();
[[nodiscard]] std::string summarizeRtFieldSet(const std::vector<std::string> &fields);
[[nodiscard]] std::string summarizeRtFieldPolicy(std::string_view field_name);
[[nodiscard]] std::string summarizeRtFieldPolicies(const std::vector<std::string> &fields);
[[nodiscard]] std::string rtFieldPolicySource();

}  // namespace rokae_xmate3_ros2::runtime

#endif
