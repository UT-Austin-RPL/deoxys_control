// Copyright 2022 Yifeng Zhu

#include <Eigen/Dense>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wreturn-type"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_BASE_TRAJ_INTERPOLATOR_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_BASE_TRAJ_INTERPOLATOR_H_

namespace traj_utils {
class BaseTrajInterpolator {
public:
  inline BaseTrajInterpolator(){};
  inline virtual ~BaseTrajInterpolator(){};

  // For pose
  inline virtual void
  Reset(const double &time_sec, const Eigen::Vector3d &p_start,
        const Eigen::Quaterniond &q_start, const Eigen::Vector3d &p_goal,
        const Eigen::Quaterniond &q_goal, const int &policy_rate,
        const int &rate, const double &traj_interpolator_time_fraction){};
  inline virtual void GetNextStep(const double &time_sec, Eigen::Vector3d &p_t,
                                  Eigen::Quaterniond &q_t){};

  // For cartesian velocity
  inline virtual void Reset(const double &time_sec, 
                            const Eigen::Vector3d &twist_trans_start,
                            const Eigen::Vector3d &twist_rot_start,
                            const Eigen::Vector3d &twist_trans_goal,
                            const Eigen::Vector3d &twist_rot_goal,
                            const int &policy_rate, const int &rate,
                            const double &traj_interpolator_time_fraction) {};
  inline virtual void GetNextStep(const double &time_sec, Eigen::Vector3d &twist_trans_t, Eigen::Vector3d &twist_rot_t) {};

  // For joints
  inline virtual void Reset(const double &time_sec,
                            const Eigen::Matrix<double, 7, 1> &j_start,
                            const Eigen::Matrix<double, 7, 1> &j_goal,
                            const int &policy_rate, const int &rate,
                            const double &traj_interpolator_time_fraction){};
  inline virtual void GetNextStep(const double &time_sec,
                                  Eigen::Matrix<double, 7, 1> &joints){};
};
} // namespace traj_utils

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_BASE_TRAJ_INTERPOLATOR_H_
#pragma GCC diagnostic pop
