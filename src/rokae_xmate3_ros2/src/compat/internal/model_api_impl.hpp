#ifndef ROKAE_COMPAT_INTERNAL_MODEL_API_IMPL_HPP
#define ROKAE_COMPAT_INTERNAL_MODEL_API_IMPL_HPP

#include <array>
#include <functional>
#include <memory>

#include "rokae/model.h"
#include "rokae/utility.h"
#include "rokae_xmate3_ros2/gazebo/model_facade.hpp"

namespace rokae {

namespace compat_internal {
inline std::array<double, 16> identity_matrix16() {
  return {1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0};
}
}  // namespace compat_internal

template <>
struct xMateModel<6>::Impl {
  gazebo::xMateER3Kinematics kinematics;
  Load load{};
  std::array<double, 16> f_t_ee{compat_internal::identity_matrix16()};
  std::array<double, 16> ee_t_k{compat_internal::identity_matrix16()};
  std::function<std::array<double, 6>(void)> current_seed_provider;
  std::function<void(const Load &)> sync_load;
  std::function<void(const std::array<double, 16> &, const std::array<double, 16> &)> sync_tcp;

  [[nodiscard]] rokae_xmate3_ros2::gazebo_model::ModelFacade facade() const {
    std::array<double, 6> tool_pose{};
    std::array<double, 6> posture{};
    Utils::transArrayToPosture(f_t_ee, posture);
    tool_pose = posture;
    return rokae_xmate3_ros2::gazebo_model::makeModelFacade(
        const_cast<gazebo::xMateER3Kinematics &>(kinematics), tool_pose, {load.mass, load.cog});
  }

  [[nodiscard]] std::array<double, 6> current_seed() const {
    if (current_seed_provider) {
      return current_seed_provider();
    }
    return {};
  }
};

}  // namespace rokae

#endif  // ROKAE_COMPAT_INTERNAL_MODEL_API_IMPL_HPP
