#ifndef ROKAE_SDK_SHIM_PLANNER_HPP
#define ROKAE_SDK_SHIM_PLANNER_HPP

#include "rokae/detail/sdk_shim_core.hpp"
#include "rokae_xmate3_ros2/runtime/strict_jerk_profile.hpp"

namespace rokae {
namespace detail {

using StrictJerkLimitedProfile = rokae_xmate3_ros2::runtime::StrictJerkLimitedScalarProfile;

}  // namespace detail

class XCORE_API CartMotionGenerator {
public:
  CartMotionGenerator(double speed_factor, double s_goal)
      : speed_factor_(speed_factor), s_goal_(s_goal) {}

  /**
   * @brief Configure scalar Cartesian arc-length limits.
   * @param ds_max Maximum path velocity.
   * @param dds_max_start Maximum start acceleration.
   * @param dds_max_end Maximum end deceleration.
   */
  void setMax(double ds_max, double dds_max_start, double dds_max_end) {
    ds_max_ = ds_max;
    dds_max_start_ = dds_max_start;
    dds_max_end_ = dds_max_end;
    profile_dirty_ = true;
  }

  [[nodiscard]] double getTime() {
    refreshProfile();
    return profile_.total_time();
  }

  /**
   * @brief Sample arc-length delta using a strict piecewise-jerk-limited scalar profile.
   * @param t Time since profile start.
   * @param delta_s_d Output travelled distance relative to the synchronized start.
   * @return true once the commanded profile duration has elapsed.
   */
  bool calculateDesiredValues(double t, double *delta_s_d) const {
    const_cast<CartMotionGenerator *>(this)->refreshProfile();
    if (delta_s_d) {
      *delta_s_d = profile_.position(t);
    }
    return t >= profile_.total_time();
  }

  void calculateSynchronizedValues(double s_init) {
    s_init_ = s_init;
    (void)s_init_;
    profile_dirty_ = true;
  }

private:
  void refreshProfile() {
    if (!profile_dirty_) {
      return;
    }
    profile_.configure(s_goal_ - s_init_,
                       ds_max_ * std::max(speed_factor_, 1e-6),
                       dds_max_start_ * std::max(speed_factor_, 1e-6),
                       dds_max_end_ * std::max(speed_factor_, 1e-6));
    profile_dirty_ = false;
  }

  double speed_factor_ = 1.0;
  double s_goal_ = 0.0;
  double s_init_ = 0.0;
  double ds_max_ = 0.5;
  double dds_max_start_ = 0.5;
  double dds_max_end_ = 0.5;
  bool profile_dirty_ = true;
  detail::StrictJerkLimitedProfile profile_{};
};

class XCORE_API JointMotionGenerator {
public:
  JointMotionGenerator(double speed_factor, std::array<double, 6> q_goal)
      : speed_factor_(speed_factor), q_goal_(q_goal) {}

  /**
   * @brief Configure per-axis velocity/acceleration/deceleration limits.
   * @param dq_max Maximum joint velocities.
   * @param ddq_max_start Maximum joint accelerations.
   * @param ddq_max_end Maximum joint decelerations.
   */
  void setMax(const std::array<double, 6> &dq_max,
              const std::array<double, 6> &ddq_max_start,
              const std::array<double, 6> &ddq_max_end) {
    dq_max_ = dq_max;
    ddq_max_start_ = ddq_max_start;
    ddq_max_end_ = ddq_max_end;
    profile_dirty_ = true;
  }

  [[nodiscard]] double getTime() {
    refreshProfiles();
    return total_time_;
  }

  /**
   * @brief Sample joint delta using synchronized strict piecewise-jerk-limited scalar profiles.
   * @param t Time since profile start.
   * @param delta_q_d Output joint deltas relative to q_init.
   * @return true once the synchronized profile has completed.
   */
  bool calculateDesiredValues(double t, std::array<double, 6> &delta_q_d) const {
    const_cast<JointMotionGenerator *>(this)->refreshProfiles();
    for (size_t i = 0; i < delta_q_d.size(); ++i) {
      if (profiles_[i].distance <= 1e-12 || total_time_ <= 1e-12) {
        delta_q_d[i] = q_goal_[i] - q_init_[i];
        continue;
      }
      const double normalized = std::clamp(t / total_time_, 0.0, 1.0);
      const double sample_time = normalized * profiles_[i].total_time;
      delta_q_d[i] = profiles_[i].position(sample_time);
    }
    return t >= total_time_;
  }

  void calculateSynchronizedValues(const std::array<double, 6> &q_init) {
    q_init_ = q_init;
    profile_dirty_ = true;
  }

private:
  void refreshProfiles() {
    if (!profile_dirty_) {
      return;
    }
    total_time_ = 0.0;
    for (size_t i = 0; i < profiles_.size(); ++i) {
      profiles_[i].configure(q_goal_[i] - q_init_[i],
                             dq_max_[i] * std::max(speed_factor_, 1e-6),
                             ddq_max_start_[i] * std::max(speed_factor_, 1e-6),
                             ddq_max_end_[i] * std::max(speed_factor_, 1e-6));
      total_time_ = std::max(total_time_, profiles_[i].total_time());
    }
    profile_dirty_ = false;
  }

  double speed_factor_ = 1.0;
  std::array<double, 6> q_goal_{};
  std::array<double, 6> q_init_{};
  std::array<double, 6> dq_max_{{1, 1, 1, 1, 1, 1}};
  std::array<double, 6> ddq_max_start_{{1, 1, 1, 1, 1, 1}};
  std::array<double, 6> ddq_max_end_{{1, 1, 1, 1, 1, 1}};
  std::array<detail::StrictJerkLimitedProfile, 6> profiles_{};
  double total_time_ = 0.0;
  bool profile_dirty_ = true;
};

template <unsigned short DoF>
class XCORE_API FollowPosition {
public:
  FollowPosition() = default;

  FollowPosition(Cobot<DoF> &robot,
                 xMateModel<DoF> &model,
                 const Eigen::Transform<double, 3, Eigen::Isometry> &endInFlange = Eigen::Transform<double, 3, Eigen::Isometry>::Identity())
      : robot_(&robot), model_(&model), end_in_flange_(endInFlange) {}

  /**
   * @brief Bind the follower to a robot/model pair after default construction.
   * @param robot Robot facade used for RT command emission.
   * @param model Model facade used for Cartesian-to-joint conversion.
   */
  void init(Cobot<DoF> &robot, xMateModel<DoF> &model) {
    std::lock_guard<std::mutex> lock(mutex_);
    robot_ = &robot;
    model_ = &model;
  }

  /**
   * @brief Start continuous following towards a Cartesian TCP target.
   * @param bMe_desire Desired TCP pose in base coordinates.
   * @throws RealtimeControlException when the RT control loop cannot be armed.
   */
  void start(const Eigen::Transform<double, 3, Eigen::Isometry> &bMe_desire) {
    ensureLoopRunning();
    update(bMe_desire);
  }

  /**
   * @brief Start continuous following towards a joint target.
   * @param jnt_desire Desired joint position in radians.
   * @throws RealtimeControlException when the RT control loop cannot be armed.
   */
  void start(const std::array<double, DoF> &jnt_desire) {
    ensureLoopRunning();
    update(jnt_desire);
  }

  /**
   * @brief Stop the follower and tear down the RT control loop.
   * @details The follower emits a finished RT joint command to leave the runtime in a clean state.
   */
  void stop() {
    std::shared_ptr<RtMotionControlCobot<DoF>> controller;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_requested_ = true;
      active_ = false;
      controller = rt_controller_;
      rt_controller_.reset();
    }
    if (controller) {
      try {
        controller->stopLoop();
      } catch (...) {
      }
      try {
        controller->stopMove();
      } catch (...) {
      }
    }
  }

  /**
   * @brief Update the Cartesian target followed by the continuous RT loop.
   * @param bMe_desire Desired TCP pose in base coordinates.
   * @details The pose is converted to a joint target using the current follower state as IK seed.
   */
  void update(const Eigen::Transform<double, 3, Eigen::Isometry> &bMe_desire) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || !robot_ || !model_) {
      return;
    }
    const Eigen::Transform<double, 3, Eigen::Isometry> flange_target =
        bMe_desire * end_in_flange_.inverse();
    std::array<double, 16> cart{};
    const auto matrix = flange_target.matrix();
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        cart[static_cast<std::size_t>(row * 4 + col)] = matrix(row, col);
      }
    }
    std::array<double, DoF> solved{};
    if (model_->getJointPos(cart, 0.0, desired_joints_, solved) != 0) {
      return;
    }
    desired_joints_ = solved;
  }

  /**
   * @brief Update the joint target followed by the continuous RT loop.
   * @param jnt_desired Desired joint configuration.
   */
  void update(const std::array<double, DoF> &jnt_desired) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || !robot_) {
      return;
    }
    desired_joints_ = jnt_desired;
  }

  /**
   * @brief Set the follower speed scale.
   * @param scale Normalised scale in [0.01, 1.0].
   * @details The scale directly limits the maximum per-cycle joint travel.
   */
  void setScale(double scale) {
    std::lock_guard<std::mutex> lock(mutex_);
    scale_ = std::clamp(scale, 0.01, 1.0);
  }

private:
  void ensureLoopRunning() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!robot_ || !model_) {
      throw RealtimeControlException("FollowPosition is not initialized");
    }
    if (active_) {
      return;
    }

    error_code ec;
    robot_->setOperateMode(OperateMode::automatic, ec);
    if (ec) {
      throw RealtimeControlException("FollowPosition failed to set automatic mode");
    }
    if (robot_->powerState(ec) != PowerState::on) {
      robot_->setPowerState(true, ec);
      if (ec) {
        throw RealtimeControlException("FollowPosition failed to power on robot");
      }
    }
    robot_->setMotionControlMode(MotionControlMode::RtCommand, ec);
    if (ec) {
      throw RealtimeControlException("FollowPosition failed to enable RT command mode");
    }

    const auto controller = robot_->getRtMotionController().lock();
    if (!controller) {
      throw RealtimeControlException("FollowPosition failed to acquire RT controller");
    }

    desired_joints_ = robot_->jointPos(ec);
    if (ec) {
      throw RealtimeControlException("FollowPosition failed to read initial joint state");
    }
    command_joints_ = desired_joints_;
    rt_controller_ = controller;
    stop_requested_ = false;
    active_ = true;

    try {
      controller->startMove(RtControllerMode::jointPosition);
      controller->setControlLoop<JointPosition>([this]() {
        std::lock_guard<std::mutex> inner_lock(mutex_);
        JointPosition command;
        command.joints.assign(command_joints_.begin(), command_joints_.end());
        const auto limits = rokae_xmate3_ros2::spec::xmate3::kJointVelocityLimit;
        bool converged = true;
        for (std::size_t axis = 0; axis < DoF; ++axis) {
          const double max_delta = std::max(1e-5, limits[axis] * scale_ * 0.001);
          const double error = desired_joints_[axis] - command_joints_[axis];
          const double step = std::clamp(error, -max_delta, max_delta);
          command_joints_[axis] += step;
          command.joints[axis] = command_joints_[axis];
          if (std::fabs(error) > 5e-4) {
            converged = false;
          }
        }
        if (stop_requested_ || (!active_ && converged)) {
          command.setFinished();
        }
        return command;
      }, 0, false);
      controller->startLoop(false);
    } catch (...) {
      active_ = false;
      stop_requested_ = true;
      rt_controller_.reset();
      try {
        controller->stopLoop();
      } catch (...) {
      }
      try {
        controller->stopMove();
      } catch (...) {
      }
      throw;
    }
  }

  Cobot<DoF> *robot_ = nullptr;
  xMateModel<DoF> *model_ = nullptr;
  Eigen::Transform<double, 3, Eigen::Isometry> end_in_flange_ = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
  std::shared_ptr<RtMotionControlCobot<DoF>> rt_controller_;
  std::mutex mutex_;
  bool active_ = false;
  bool stop_requested_ = false;
  double scale_ = 0.5;
  std::array<double, DoF> desired_joints_{};
  std::array<double, DoF> command_joints_{};
};

}  // namespace rokae

#endif  // ROKAE_SDK_SHIM_PLANNER_HPP
