// Copyright 2022 Yifeng Zhu

#include "base_traj_interpolator.h"
#include <Eigen/Dense>
#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_POSE_TRAJ_INTERPOLATOR_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_POSE_TRAJ_INTERPOLATOR_H_

namespace traj_utils {
class LinearPoseTrajInterpolator : public BaseTrajInterpolator {
private:
  Eigen::Vector3d p_start_;
  Eigen::Vector3d p_goal_;
  Eigen::Vector3d last_p_t_;
  Eigen::Vector3d prev_p_goal_;

  Eigen::Quaterniond q_start_;
  Eigen::Quaterniond q_goal_;
  Eigen::Quaterniond last_q_t_;
  Eigen::Quaterniond prev_q_goal_;

  double dt_;
  double last_time_;
  double max_time_;
  double start_time_;
  bool start_;
  bool first_goal_;

public:
  inline LinearPoseTrajInterpolator()
      : dt_(0.), last_time_(0.), max_time_(1.), start_time_(0.), start_(false),
        first_goal_(true){};

  inline ~LinearPoseTrajInterpolator(){};

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

    if (first_goal_) {
      p_start_ = p_start;
      q_start_ = q_start;

      prev_p_goal_ = p_start;
      prev_q_goal_ = q_start;
      first_goal_ = false;
      // std::cout << "First goal" << std::endl;
    } else {
      // If the goal is already set, use prev goal as the starting point of
      // interpolation.
      prev_p_goal_ = p_goal_;
      prev_q_goal_ = q_goal_;

      p_start_ = prev_p_goal_;
      q_start_ = prev_q_goal_;
    }

    p_goal_ = p_goal;
    q_goal_ = q_goal;

    // Flip the sign if the dot product of quaternions is negative
    if (q_goal_.coeffs().dot(q_start_.coeffs()) < 0.0) {
      q_start_.coeffs() << -q_start_.coeffs();
    }
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
      last_p_t_ = p_start_ + t * (p_goal_ - p_start_);
      last_q_t_ = q_start_.slerp(t, q_goal_);
      last_time_ = time_sec;
    }
    p_t = last_p_t_;
    q_t = last_q_t_;
  };
};
} // namespace traj_utils
#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_POSE_TRAJ_INTERPOLATOR_H_
