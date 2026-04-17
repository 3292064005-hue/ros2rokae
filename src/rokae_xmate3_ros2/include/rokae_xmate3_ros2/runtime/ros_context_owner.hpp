#ifndef ROKAE_XMATE3_ROS2_RUNTIME_ROS_CONTEXT_OWNER_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_ROS_CONTEXT_OWNER_HPP

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace rokae_xmate3_ros2::runtime {

/**
 * @brief Process-wide owner for the default ROS2 context.
 *
 * Library objects and the Gazebo plugin share the same default ROS context.
 * This helper centralises ownership accounting so that individual wrappers do
 * not call global rclcpp::shutdown() from their destructors.
 */
class RosContextOwner {
 public:
  class Lease {
   public:
    Lease() = default;
    Lease(const Lease &) = delete;
    Lease &operator=(const Lease &) = delete;
    Lease(Lease &&) noexcept = default;
    Lease &operator=(Lease &&) noexcept = default;
    ~Lease() {
      if (owner_ != nullptr) {
        owner_->release();
      }
    }

    [[nodiscard]] bool initialized_ros() const noexcept { return initialized_ros_; }

   private:
    friend class RosContextOwner;
    explicit Lease(RosContextOwner *owner, bool initialized_ros)
        : owner_(owner), initialized_ros_(initialized_ros) {}

    RosContextOwner *owner_ = nullptr;
    bool initialized_ros_ = false;
  };

  /**
   * @brief Acquire shared ownership of the process ROS context.
   *
   * @param reason Diagnostic label used only for logging/debugging.
   * @return Shared lease; releasing the last lease does not automatically shut
   *         ROS down, preserving compatibility with embedded callers.
   */
  [[nodiscard]] static std::shared_ptr<Lease> acquire(const std::string &reason = {}) {
    return instance().acquireImpl(reason);
  }

  [[nodiscard]] static std::size_t activeLeaseCount() noexcept {
    return instance().activeLeaseCountImpl();
  }

 private:
  static RosContextOwner &instance() {
    static RosContextOwner owner;
    return owner;
  }

  [[nodiscard]] std::shared_ptr<Lease> acquireImpl(const std::string &) {
    std::lock_guard<std::mutex> lock(mutex_);
    bool initialized_ros = false;
    if (!rclcpp::ok()) {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
      initialized_ros = true;
      initialized_by_owner_ = true;
    }
    ++lease_count_;
    return std::shared_ptr<Lease>(new Lease(this, initialized_ros));
  }

  void release() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (lease_count_ > 0) {
      --lease_count_;
    }
  }

  [[nodiscard]] std::size_t activeLeaseCountImpl() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return lease_count_;
  }

  mutable std::mutex mutex_;
  std::size_t lease_count_ = 0;
  bool initialized_by_owner_ = false;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif  // ROKAE_XMATE3_ROS2_RUNTIME_ROS_CONTEXT_OWNER_HPP
