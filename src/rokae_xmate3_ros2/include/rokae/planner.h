#ifndef ROKAE_PLANNER_H
#define ROKAE_PLANNER_H

#include <array>
#include <memory>
#include <mutex>

#include <Eigen/Geometry>

#include "rokae/base.h"
#include "rokae/model.h"

namespace rokae {

class BaseCobot;
template <unsigned short DoF>
class Cobot;

template <unsigned short DoF>
class FollowPosition;

/**
 * @brief S-profile Cartesian arc-length generator for the xMate6 compatibility lane.
 */
class XCORE_API CartMotionGenerator {
 public:
  CartMotionGenerator(double speed_factor, double s_goal);
  ~CartMotionGenerator();

  void setMax(double ds_max, double dds_max_start, double dds_max_end);
  double getTime() const;
  bool calculateDesiredValues(double t, double *delta_s_d) const;
  void calculateSynchronizedValues(double s_init);

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

/**
 * @brief S-profile joint generator for the xMate6 compatibility lane.
 */
class XCORE_API JointMotionGenerator {
 public:
  JointMotionGenerator(double speed_factor, std::array<double, 6> q_goal);
  ~JointMotionGenerator();

  void setMax(const std::array<double, 6> &dq_max,
              const std::array<double, 6> &ddq_max_start,
              const std::array<double, 6> &ddq_max_end);
  double getTime() const;
  bool calculateDesiredValues(double t, std::array<double, 6> &delta_q_d) const;
  void calculateSynchronizedValues(const std::array<double, 6> &q_init);

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

template <>
class XCORE_API FollowPosition<6> {
 public:
  FollowPosition();
  FollowPosition(Cobot<6> &robot,
                 xMateModel<6> &model,
                 const Eigen::Transform<double, 3, Eigen::Isometry> &endInFlange =
                     Eigen::Transform<double, 3, Eigen::Isometry>::Identity());
  ~FollowPosition();

  void init(Cobot<6> &robot, xMateModel<6> &model);
  void start(const Eigen::Transform<double, 3, Eigen::Isometry> &bMe_desire);
  void start(const std::array<double, 6> &jnt_desire);
  void stop();
  void update(const Eigen::Transform<double, 3, Eigen::Isometry> &bMe_desire);
  void update(const std::array<double, 6> &jnt_desired);
  void setScale(double scale);

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace rokae

#endif  // ROKAE_PLANNER_H
