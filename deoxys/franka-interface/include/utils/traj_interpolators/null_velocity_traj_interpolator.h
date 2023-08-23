// Copyright 2023 Yifeng Zhu

#include "base_traj_interpolator.h"
#include <Eigen/Dense>
#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_NULL_VELOCITY_TRAJ_INTERPOLATOR_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_NULL_VELOCITY_TRAJ_INTERPOLATOR_H_

namespace traj_utils {
class NullVelocityTrajInterpolator : public BaseTrajInterpolator {
private:

  Eigen::Vector3d twist_trans_start_;
  Eigen::Vector3d twist_trans_goal_;
  Eigen::Vector3d last_twist_trans_t_;

  Eigen::Vector3d twist_rot_start_;
  Eigen::Vector3d twist_rot_goal_;
  Eigen::Vector3d last_twist_rot_t_;

  Eigen::Vector3d prev_twist_trans_goal_;
  Eigen::Vector3d prev_twist_rot_goal_;

  double dt_;
  double last_time_;
  double max_time_;
  double start_time_;
  bool start_;
  bool first_goal_;

  double interpolation_fraction_; // fraction of actual interpolation within an
                                  // interval

public:
  inline NullVelocityTrajInterpolator()
      : dt_(0.), last_time_(0.), max_time_(1.), start_time_(0.), start_(false),
        first_goal_(true){};

  inline ~NullVelocityTrajInterpolator(){};

  inline void Reset(const double &time_sec,
                    const Eigen::Vector3d &twist_trans_start,
                    const Eigen::Vector3d &twist_rot_start,
                    const Eigen::Vector3d &twist_trans_goal,
                    const Eigen::Vector3d &twist_rot_goal,
                    const int &policy_rate, const int &rate,
                    const double &traj_interpolator_time_fraction) {
    // Note that twist_*_start will be ignored in null Velocityocity trajectory interpolator
    dt_ = 1. / static_cast<double>(rate);
    last_time_ = time_sec;

    max_time_ =
        1. / static_cast<double>(policy_rate) * traj_interpolator_time_fraction;
    start_time_ = time_sec;

    start_ = false;

    if (first_goal_) {
      twist_trans_start_ = twist_trans_start;
      twist_rot_start_ = twist_rot_start;
      prev_twist_trans_goal_ = twist_trans_start_;
      prev_twist_rot_goal_ = twist_rot_start_;
      first_goal_ = false;
    } else {
      // prev_twist_trans_goal_ = twist_trans_goal_;
      // prev_twist_rot_goal_ = twist_rot_goal_;

      prev_twist_trans_goal_ = twist_trans_start;
      prev_twist_trans_goal_ = twist_rot_start;

      twist_trans_start_ = prev_twist_trans_goal_;
      twist_rot_start_ = prev_twist_rot_goal_;
    }
    twist_trans_goal_ = twist_trans_goal;
    twist_rot_goal_ = twist_rot_goal;

    std::cout << " ** twist trans goal: " << twist_trans_goal_.transpose() << std::endl;
    std::cout << " ** twist trans start: " << twist_trans_start_.transpose() << std::endl; 

    // std::cout << twist_trans_goal_.transpose() << std::endl;
  };

  inline void GetNextStep(const double &time_sec, Eigen::Vector3d &twist_trans_t, Eigen::Vector3d &twist_rot_t) {
    if (!start_) {
      start_time_ = time_sec;
      start_ = true;
    }

    // Yifeng: This doesn't really anything in this trajectory interpolator. But this implementation logic is kept for consistency and for future use.
    if (last_time_ + dt_ <= time_sec) {
      double t =
          std::min(std::max((time_sec - start_time_) / max_time_, 0.), 1.);
      double transformed_t = t;  // 30 * std::pow(t, 2) - 60 * std::pow(t, 3) + 30 * std::pow(t, 4);
      last_time_ = time_sec;
      
      // std::cout << " -- twist trans goal: " << twist_trans_goal_.transpose() << std::endl;
      // std::cout << " -- twist trans start: " << twist_trans_start_.transpose() << std::endl; 

      // std::cout << transformed_t << " : " << twist_trans_goal_ << std::endl;
      // std::cout << transformed_t << " : " << twist_trans_start_ << std::endl;
      last_twist_trans_t_ = twist_trans_start_ + transformed_t * (twist_trans_goal_ - twist_trans_start_);
      last_twist_rot_t_ = twist_rot_start_ + transformed_t * (twist_rot_goal_- twist_rot_start_);
      
      // last_twist_trans_t_ = twist_trans_start_;
      // last_twist_rot_t_ = twist_rot_start_;
    }
    twist_trans_t = last_twist_trans_t_;
    twist_rot_t = last_twist_rot_t_;
    // std::cout << "Last twist: " << last_twist_trans_t_.transpose() << std::endl;

  };
};
} // namespace traj_utils

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_NULL_VELOCITY_TRAJ_INTERPOLATOR_H_
