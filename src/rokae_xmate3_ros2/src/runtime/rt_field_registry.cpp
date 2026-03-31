#include "runtime/rt_field_registry.hpp"

#include <algorithm>
#include <sstream>

#include "rokae_xmate3_ros2/types.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr RtFieldDescriptor kDescriptors[] = {
    {rokae::RtSupportedFields::jointPos_m, RtFieldValueKind::array6, sizeof(double) * 6, true, false, false, "joint"},
    {rokae::RtSupportedFields::jointPos_c, RtFieldValueKind::array6, sizeof(double) * 6, true, true, true, "joint"},
    {rokae::RtSupportedFields::jointVel_m, RtFieldValueKind::array6, sizeof(double) * 6, true, false, false, "joint"},
    {rokae::RtSupportedFields::jointVel_c, RtFieldValueKind::array6, sizeof(double) * 6, true, true, true, "joint"},
    {rokae::RtSupportedFields::jointAcc_c, RtFieldValueKind::array6, sizeof(double) * 6, true, true, true, "joint"},
    {rokae::RtSupportedFields::tcpPose_m, RtFieldValueKind::matrix16, sizeof(double) * 16, true, true, true, "cartesian"},
    {rokae::RtSupportedFields::tcpPoseAbc_m, RtFieldValueKind::array6, sizeof(double) * 6, true, true, true, "cartesian"},
    {rokae::RtSupportedFields::tcpPose_c, RtFieldValueKind::matrix16, sizeof(double) * 16, true, true, true, "cartesian"},
    {rokae::RtSupportedFields::tcpVel_c, RtFieldValueKind::array6, sizeof(double) * 6, true, true, true, "cartesian"},
    {rokae::RtSupportedFields::tcpAcc_c, RtFieldValueKind::array6, sizeof(double) * 6, true, true, true, "cartesian"},
    {rokae::RtSupportedFields::elbow_m, RtFieldValueKind::scalar, sizeof(double), true, true, true, "elbow"},
    {rokae::RtSupportedFields::elbow_c, RtFieldValueKind::scalar, sizeof(double), true, true, true, "elbow"},
    {rokae::RtSupportedFields::elbowVel_c, RtFieldValueKind::scalar, sizeof(double), true, true, true, "elbow"},
    {rokae::RtSupportedFields::elbowAcc_c, RtFieldValueKind::scalar, sizeof(double), true, true, true, "elbow"},
    {rokae::RtSupportedFields::tau_m, RtFieldValueKind::array6, sizeof(double) * 6, true, false, false, "torque"},
    {rokae::RtSupportedFields::tau_c, RtFieldValueKind::array6, sizeof(double) * 6, true, true, true, "torque"},
    {rokae::RtSupportedFields::tauFiltered_m, RtFieldValueKind::array6, sizeof(double) * 6, true, true, true, "torque"},
    {rokae::RtSupportedFields::tauVel_c, RtFieldValueKind::array6, sizeof(double) * 6, true, true, true, "torque"},
    {rokae::RtSupportedFields::tauExt_inBase, RtFieldValueKind::array6, sizeof(double) * 6, true, true, true, "torque"},
    {rokae::RtSupportedFields::tauExt_inStiff, RtFieldValueKind::array6, sizeof(double) * 6, true, true, true, "torque"},
    {rokae::RtSupportedFields::theta_m, RtFieldValueKind::array6, sizeof(double) * 6, true, true, true, "motor"},
    {rokae::RtSupportedFields::thetaVel_m, RtFieldValueKind::array6, sizeof(double) * 6, true, true, true, "motor"},
    {rokae::RtSupportedFields::motorTau, RtFieldValueKind::array6, sizeof(double) * 6, true, true, true, "motor"},
    {rokae::RtSupportedFields::motorTauFiltered, RtFieldValueKind::array6, sizeof(double) * 6, true, true, true, "motor"},
};

}  // namespace

const RtFieldDescriptor *findRtFieldDescriptor(std::string_view field_name) {
  const auto *begin = std::begin(kDescriptors);
  const auto *end = std::end(kDescriptors);
  const auto it = std::find_if(begin, end, [&](const RtFieldDescriptor &descriptor) {
    return descriptor.name == field_name;
  });
  return it == end ? nullptr : it;
}

bool isRtFieldSupported(std::string_view field_name) {
  return findRtFieldDescriptor(field_name) != nullptr;
}

std::size_t rtFieldBytes(std::string_view field_name) {
  const auto *descriptor = findRtFieldDescriptor(field_name);
  return descriptor == nullptr ? 0U : descriptor->bytes;
}

std::vector<std::string> defaultRtFieldSet() {
  return {rokae::RtSupportedFields::jointPos_m,
          rokae::RtSupportedFields::jointVel_m,
          rokae::RtSupportedFields::tau_m};
}

std::vector<RtFieldDescriptor> supportedRtFields() {
  return {std::begin(kDescriptors), std::end(kDescriptors)};
}

std::string summarizeRtFieldSet(const std::vector<std::string> &fields) {
  std::ostringstream stream;
  stream << "fields=";
  bool first = true;
  std::size_t total_bytes = 0;
  for (const auto &field : fields) {
    if (!first) {
      stream << ',';
    }
    first = false;
    stream << field;
    total_bytes += rtFieldBytes(field);
  }
  stream << " bytes=" << total_bytes;
  return stream.str();
}

}  // namespace rokae_xmate3_ros2::runtime
