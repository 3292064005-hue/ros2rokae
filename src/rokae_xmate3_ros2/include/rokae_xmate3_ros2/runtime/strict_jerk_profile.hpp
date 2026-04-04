#ifndef ROKAE_XMATE3_ROS2_RUNTIME_STRICT_JERK_PROFILE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_STRICT_JERK_PROFILE_HPP

#include <algorithm>
#include <cmath>

namespace rokae_xmate3_ros2::runtime {

struct StrictJerkSample {
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

/**
 * @brief Symmetric piecewise-constant-jerk motion profile with optional cruise phase.
 * @details The profile uses a two-ramp jerk-limited acceleration phase, an optional constant-velocity
 *          cruise, and a mirrored two-ramp deceleration phase. Acceleration is continuous and jerk is
 *          piecewise constant across all dynamic phases.
 */
class StrictJerkLimitedScalarProfile {
public:
  void configure(double commanded_distance,
                 double velocity_max,
                 double accel_max,
                 double decel_max) {
    distance_ = std::abs(commanded_distance);
    sign_ = commanded_distance < 0.0 ? -1.0 : 1.0;
    velocity_limit_ = std::max(std::abs(velocity_max), 1e-6);
    accel_limit_ = std::max(std::abs(accel_max), 1e-6);
    decel_limit_ = std::max(std::abs(decel_max), 1e-6);
    peak_velocity_ = 0.0;
    jerk_accel_ = 0.0;
    jerk_decel_ = 0.0;
    accel_ramp_time_ = 0.0;
    decel_ramp_time_ = 0.0;
    cruise_time_ = 0.0;
    accel_distance_ = 0.0;
    decel_distance_ = 0.0;
    total_time_ = 0.0;

    if (distance_ <= 1e-12) {
      return;
    }

    const double denom = (1.0 / accel_limit_) + (1.0 / decel_limit_);
    const double velocity_to_hit_limits = std::sqrt(distance_ / std::max(denom, 1e-9));
    peak_velocity_ = std::min(velocity_limit_, velocity_to_hit_limits);
    peak_velocity_ = std::max(peak_velocity_, 1e-6);

    accel_ramp_time_ = peak_velocity_ / accel_limit_;
    decel_ramp_time_ = peak_velocity_ / decel_limit_;
    jerk_accel_ = accel_limit_ / std::max(accel_ramp_time_, 1e-9);
    jerk_decel_ = decel_limit_ / std::max(decel_ramp_time_, 1e-9);

    accel_distance_ = peak_velocity_ * peak_velocity_ / accel_limit_;
    decel_distance_ = peak_velocity_ * peak_velocity_ / decel_limit_;
    const double cruise_distance = std::max(0.0, distance_ - accel_distance_ - decel_distance_);
    cruise_time_ = cruise_distance / std::max(peak_velocity_, 1e-9);
    total_time_ = 2.0 * accel_ramp_time_ + cruise_time_ + 2.0 * decel_ramp_time_;
  }

  [[nodiscard]] StrictJerkSample sample(double t) const {
    StrictJerkSample sample_out{};
    if (distance_ <= 1e-12 || total_time_ <= 1e-12) {
      return sample_out;
    }

    const double clamped_t = std::clamp(t, 0.0, total_time_);
    const double t1 = accel_ramp_time_;
    const double t2 = 2.0 * accel_ramp_time_;
    const double t3 = t2 + cruise_time_;
    const double t4 = t3 + decel_ramp_time_;

    const double v_half_accel = 0.5 * peak_velocity_;
    const double s_phase1 = peak_velocity_ * peak_velocity_ / (6.0 * accel_limit_);
    const double s_accel = accel_distance_;
    const double s_cruise_end = s_accel + peak_velocity_ * cruise_time_;
    const double s_mid_decel = s_cruise_end + peak_velocity_ * peak_velocity_ / (3.0 * decel_limit_);

    auto apply_sign = [&](double position, double velocity, double acceleration) {
      sample_out.position = sign_ * std::clamp(position, 0.0, distance_);
      sample_out.velocity = sign_ * velocity;
      sample_out.acceleration = sign_ * acceleration;
    };

    if (clamped_t <= t1) {
      const double tau = clamped_t;
      apply_sign(jerk_accel_ * tau * tau * tau / 6.0,
                 0.5 * jerk_accel_ * tau * tau,
                 jerk_accel_ * tau);
      return sample_out;
    }
    if (clamped_t <= t2) {
      const double tau = clamped_t - t1;
      apply_sign(s_phase1 + v_half_accel * tau + 0.5 * accel_limit_ * tau * tau - jerk_accel_ * tau * tau * tau / 6.0,
                 v_half_accel + accel_limit_ * tau - 0.5 * jerk_accel_ * tau * tau,
                 accel_limit_ - jerk_accel_ * tau);
      return sample_out;
    }
    if (clamped_t <= t3) {
      const double tau = clamped_t - t2;
      apply_sign(s_accel + peak_velocity_ * tau, peak_velocity_, 0.0);
      return sample_out;
    }
    if (clamped_t <= t4) {
      const double tau = clamped_t - t3;
      apply_sign(s_cruise_end + peak_velocity_ * tau - jerk_decel_ * tau * tau * tau / 6.0,
                 peak_velocity_ - 0.5 * jerk_decel_ * tau * tau,
                 -jerk_decel_ * tau);
      return sample_out;
    }

    const double tau = clamped_t - t4;
    const double v_half_decel = 0.5 * peak_velocity_;
    apply_sign(s_mid_decel + v_half_decel * tau - 0.5 * decel_limit_ * tau * tau + jerk_decel_ * tau * tau * tau / 6.0,
               v_half_decel - decel_limit_ * tau + 0.5 * jerk_decel_ * tau * tau,
               -decel_limit_ + jerk_decel_ * tau);
    return sample_out;
  }

  [[nodiscard]] double position(double t) const { return sample(t).position; }
  [[nodiscard]] double velocity(double t) const { return sample(t).velocity; }
  [[nodiscard]] double acceleration(double t) const { return sample(t).acceleration; }
  [[nodiscard]] double total_time() const { return total_time_; }
  [[nodiscard]] double distance() const { return distance_; }

private:
  double distance_{0.0};
  double sign_{1.0};
  double velocity_limit_{1.0};
  double accel_limit_{1.0};
  double decel_limit_{1.0};
  double peak_velocity_{0.0};
  double jerk_accel_{0.0};
  double jerk_decel_{0.0};
  double accel_ramp_time_{0.0};
  double decel_ramp_time_{0.0};
  double cruise_time_{0.0};
  double accel_distance_{0.0};
  double decel_distance_{0.0};
  double total_time_{0.0};
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
