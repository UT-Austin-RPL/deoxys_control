// Copyright 2022 Yifeng Zhu

#include "base_traj_interpolator.h"
#include <Eigen/Dense>
#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_POSITION_TRAJ_INTERPOLATOR_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_POSITION_TRAJ_INTERPOLATOR_H_

namespace traj_utils {
class LinearPositionTrajInterpolator : public BaseTrajInterpolator {
private:
  Eigen::Vector3d p_start_;
  Eigen::Vector3d p_goal_;
  Eigen::Vector3d last_p_t_;

  double dt_;
  double last_time_;
  double max_time_;
  double start_time_;
  bool start_;

public:
  inline LinearPositionTrajInterpolator()
      : dt_(0.), last_time_(0.), max_time_(1.), start_time_(0.),
        start_(false){};

  inline ~LinearPositionTrajInterpolator(){};

  inline void Reset(const double &time_sec, const Eigen::Vector3d &p_start,
                    const Eigen::Vector3d &p_goal, const int &policy_freq,
                    const int &freq,
                    const double &traj_interpolator_time_fraction) {
    dt_ = 1. / static_cast<double>(freq);
    last_time_ = time_sec;
    max_time_ =
        1. / static_cast<double>(policy_freq) * traj_interpolator_time_fraction;
    start_time_ = time_sec;

    start_ = false;

    p_start_ = p_start;
    p_goal_ = p_goal;
  };

  inline void GetNextStep(const double &time_sec, Eigen::Vector3d &p_t) {

    if (!start_) {
      start_time_ = time_sec;
      last_p_t_ = p_start_;
      start_ = true;
    }

    if (last_time_ + dt_ <= time_sec) {
      double t =
          std::min(std::max((time_sec - start_time_) / max_time_, 0.), 1.);
      last_p_t_ = p_start_ + t * (p_goal_ - p_start_);
      last_time_ = time_sec;
    }
    p_t = last_p_t_;
  };
};
} // namespace traj_utils
#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_POSITION_TRAJ_INTERPOLATOR_H_
