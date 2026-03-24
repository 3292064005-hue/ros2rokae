#ifndef ROKAE_XMATE3_ROS2_RUNTIME_OWNER_ARBITER_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_OWNER_ARBITER_HPP

#include <string>
#include <utility>

#include "runtime/runtime_types.hpp"

namespace rokae_xmate3_ros2::runtime {

class OwnerArbiter {
 public:
  struct TransitionResult {
    bool accepted = false;
    ControlOwner owner = ControlOwner::none;
    std::string reason;
  };

  [[nodiscard]] ControlOwner current() const noexcept { return current_owner_; }
  [[nodiscard]] const std::string &reason() const noexcept { return last_reason_; }
  [[nodiscard]] bool faulted() const noexcept { return faulted_; }

  [[nodiscard]] bool canTransition(ControlOwner desired_owner) const noexcept {
    if (desired_owner == current_owner_) {
      return true;
    }
    if (desired_owner == ControlOwner::none) {
      return true;
    }
    if (faulted_) {
      return false;
    }
    if (current_owner_ == ControlOwner::none) {
      return true;
    }
    return false;
  }

  [[nodiscard]] TransitionResult clear(const std::string &reason) {
    faulted_ = false;
    return transitionTo(ControlOwner::none, reason);
  }

  [[nodiscard]] TransitionResult claimTrajectory(const std::string &reason) {
    return transitionTo(ControlOwner::trajectory, reason);
  }

  [[nodiscard]] TransitionResult claimEffort(const std::string &reason) {
    return transitionTo(ControlOwner::effort, reason);
  }

  [[nodiscard]] TransitionResult fault(const std::string &reason) {
    faulted_ = true;
    current_owner_ = ControlOwner::none;
    last_reason_ = reason.empty() ? "fault" : reason;
    return {false, current_owner_, last_reason_};
  }

 private:
  [[nodiscard]] TransitionResult transitionTo(ControlOwner desired_owner, const std::string &reason) {
    const auto normalized_reason = reason.empty() ? std::string{"owner transition"} : reason;
    if (!canTransition(desired_owner)) {
      const std::string rejected_reason =
          std::string{"owner transition rejected: "} + to_string(current_owner_) + " -> " +
          to_string(desired_owner) + " (" + normalized_reason + ")";
      return {false,
              current_owner_,
              rejected_reason};
    }
    current_owner_ = desired_owner;
    last_reason_ = normalized_reason;
    return {true, current_owner_, last_reason_};
  }

  ControlOwner current_owner_ = ControlOwner::none;
  std::string last_reason_{"idle"};
  bool faulted_ = false;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif  // ROKAE_XMATE3_ROS2_RUNTIME_OWNER_ARBITER_HPP
