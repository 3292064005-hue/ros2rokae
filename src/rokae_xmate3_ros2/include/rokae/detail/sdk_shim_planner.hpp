#ifndef ROKAE_SDK_SHIM_PLANNER_HPP
#define ROKAE_SDK_SHIM_PLANNER_HPP

#include "rokae/detail/sdk_shim_core.hpp"

namespace rokae {

class XCORE_API CartMotionGenerator {
public:
  CartMotionGenerator(double speed_factor, double s_goal)
      : speed_factor_(speed_factor), s_goal_(s_goal) {}

  void setMax(double ds_max, double dds_max_start, double dds_max_end) {
    ds_max_ = ds_max;
    dds_max_start_ = dds_max_start;
    dds_max_end_ = dds_max_end;
  }

  double getTime() {
    return std::abs(s_goal_) / std::max(ds_max_ * std::max(speed_factor_, 1e-6), 1e-6);
  }

  bool calculateDesiredValues(double t, double *delta_s_d) const {
    const double total = std::abs(s_goal_) / std::max(ds_max_ * std::max(speed_factor_, 1e-6), 1e-6);
    if (delta_s_d) {
      *delta_s_d = std::clamp(t / std::max(total, 1e-6), 0.0, 1.0) * s_goal_;
    }
    return t >= total;
  }

  void calculateSynchronizedValues(double s_init) {
    s_init_ = s_init;
  }

private:
  double speed_factor_ = 1.0;
  double s_goal_ = 0.0;
  double s_init_ = 0.0;
  double ds_max_ = 0.5;
  double dds_max_start_ = 0.5;
  double dds_max_end_ = 0.5;
};

class XCORE_API JointMotionGenerator {
public:
  JointMotionGenerator(double speed_factor, std::array<double, 6> q_goal)
      : speed_factor_(speed_factor), q_goal_(q_goal) {}

  void setMax(const std::array<double, 6> &dq_max,
              const std::array<double, 6> &ddq_max_start,
              const std::array<double, 6> &ddq_max_end) {
    dq_max_ = dq_max;
    ddq_max_start_ = ddq_max_start;
    ddq_max_end_ = ddq_max_end;
  }

  double getTime() {
    double max_delta = 0.0;
    for (size_t i = 0; i < q_goal_.size(); ++i) {
      max_delta = std::max(max_delta, std::abs(q_goal_[i] - q_init_[i]));
    }
    const double max_speed = std::max(1e-6, *std::max_element(dq_max_.begin(), dq_max_.end()) * std::max(speed_factor_, 1e-6));
    return max_delta / max_speed;
  }

  bool calculateDesiredValues(double t, std::array<double, 6> &delta_q_d) const {
    const double total = std::max(getTimeCached(), 1e-6);
    const double alpha = std::clamp(t / total, 0.0, 1.0);
    for (size_t i = 0; i < delta_q_d.size(); ++i) {
      delta_q_d[i] = (q_goal_[i] - q_init_[i]) * alpha;
    }
    return t >= total;
  }

  void calculateSynchronizedValues(const std::array<double, 6> &q_init) {
    q_init_ = q_init;
  }

private:
  double getTimeCached() const {
    double max_delta = 0.0;
    for (size_t i = 0; i < q_goal_.size(); ++i) {
      max_delta = std::max(max_delta, std::abs(q_goal_[i] - q_init_[i]));
    }
    const double max_speed = std::max(1e-6, *std::max_element(dq_max_.begin(), dq_max_.end()) * std::max(speed_factor_, 1e-6));
    return max_delta / max_speed;
  }

  double speed_factor_ = 1.0;
  std::array<double, 6> q_goal_{};
  std::array<double, 6> q_init_{};
  std::array<double, 6> dq_max_{{1, 1, 1, 1, 1, 1}};
  std::array<double, 6> ddq_max_start_{{1, 1, 1, 1, 1, 1}};
  std::array<double, 6> ddq_max_end_{{1, 1, 1, 1, 1, 1}};
};

template <unsigned short DoF>
class XCORE_API FollowPosition {
public:
  FollowPosition() = default;

  FollowPosition(Cobot<DoF> &robot,
                 xMateModel<DoF> &model,
                 const Eigen::Transform<double, 3, Eigen::Isometry> &endInFlange = Eigen::Transform<double, 3, Eigen::Isometry>::Identity())
      : robot_(&robot), model_(&model), end_in_flange_(endInFlange) {}

  void init(Cobot<DoF> &robot, XMateModel<DoF> &model) {
    robot_ = &robot;
    model_ = &model;
  }

  void start(const Eigen::Transform<double, 3, Eigen::Isometry> &bMe_desire) {
    active_ = true;
    update(bMe_desire);
  }

  void start(const std::array<double, DoF> &jnt_desire) {
    active_ = true;
    update(jnt_desire);
  }

  void stop() {
    active_ = false;
    if (robot_) {
      error_code ec;
      robot_->stop(ec);
    }
  }

  void update(const Eigen::Transform<double, 3, Eigen::Isometry> &bMe_desire) {
    if (!active_ || !robot_) {
      return;
    }
    const Eigen::Vector3d trans = bMe_desire.translation();
    const Eigen::Vector3d rpy = bMe_desire.rotation().eulerAngles(0, 1, 2);
    CartesianPosition pose({trans.x(), trans.y(), trans.z(), rpy.x(), rpy.y(), rpy.z()});
    MoveLCommand cmd(pose);
    error_code ec;
    robot_->executeCommand({cmd}, ec);
  }

  void update(const std::array<double, DoF> &jnt_desired) {
    if (!active_ || !robot_) {
      return;
    }
    MoveAbsJCommand cmd;
    cmd.target = JointPosition(std::vector<double>(jnt_desired.begin(), jnt_desired.end()));
    error_code ec;
    robot_->executeCommand({cmd}, ec);
  }

  void setScale(double scale) {
    scale_ = scale;
  }

private:
  Cobot<DoF> *robot_ = nullptr;
  xMateModel<DoF> *model_ = nullptr;
  Eigen::Transform<double, 3, Eigen::Isometry> end_in_flange_ = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
  bool active_ = false;
  double scale_ = 0.5;
};

}  // namespace rokae

#endif  // ROKAE_SDK_SHIM_PLANNER_HPP
