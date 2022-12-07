// Copyright 2022 Yifeng Zhu

#include "base_traj_interpolator.h"
#include <Eigen/Dense>
#ifndef UTILS_TRAJ_INTERPOLATORS_MIN_JERK_POSE_TRAJ_INTERPOLATOR_H_
#define UTILS_TRAJ_INTERPOLATORS_MIN_JERK_POSE_TRAJ_INTERPOLATOR_H_

namespace traj_utils {
class MinJerkPoseTrajInterpolator : public BaseTrajInterpolator {
private:
  Eigen::Vector3d p_start_;
  Eigen::Vector3d p_goal_;
  Eigen::Vector3d last_p_t_;

  Eigen::Quaterniond q_start_;
  Eigen::Quaterniond q_goal_;
  Eigen::Quaterniond last_q_t_;

  double dt_;
  double last_time_;
  double max_time_;
  double start_time_;
  bool start_;

public:
  inline MinJerkPoseTrajInterpolator()
      : dt_(0.), last_time_(0.), max_time_(1.), start_time_(0.),
        start_(false){};

  inline ~MinJerkPoseTrajInterpolator(){};

  inline void Reset(const double &time_sec, const Eigen::Vector3d &p_start,
                    const Eigen::Quaterniond &q_start,
                    const Eigen::Vector3d &p_goal,
                    const Eigen::Quaterniond &q_goal, const int &policy_rate,
                    const int &rate,
                    const double &traj_interpolator_time_fraction) {
    dt_ = 1. / static_cast<double>(rate);
    last_time_ = time_sec;
    max_time_ =
        1. / static_cast<double>(policy_rate) * traj_interpolator_time_fraction;
    start_time_ = time_sec;

    start_ = false;

    p_start_ = p_start;
    q_start_ = q_start;

    // Flip the sign if the dot product of quaternions is negative
    if (q_goal_.coeffs().dot(q_start_.coeffs()) < 0.0) {
      q_start_.coeffs() << -q_start_.coeffs();
    }
    p_goal_ = p_goal;
    q_goal_ = q_goal;
  };

  inline void GetNextStep(const double &time_sec, Eigen::Vector3d &p_t,
                          Eigen::Quaterniond &q_t) {
    if (!start_) {
      start_time_ = time_sec;
      last_p_t_ = p_start_;
      last_q_t_ = q_start_;
      start_ = true;
    }

    if (last_time_ + dt_ <= time_sec) {
      double t =
          std::min(std::max((time_sec - start_time_) / max_time_, 0.), 1.);
      double transformed_t =
          10 * std::pow(t, 3) - 15 * std::pow(t, 4) + 6 * std::pow(t, 5);

      last_p_t_ = p_start_ + transformed_t * (p_goal_ - p_start_);
      last_q_t_ = q_start_.slerp(transformed_t, q_goal_);
      last_time_ = time_sec;
    }
    p_t = last_p_t_;
    q_t = last_q_t_;
  };
};

} // namespace traj_utils

#endif
