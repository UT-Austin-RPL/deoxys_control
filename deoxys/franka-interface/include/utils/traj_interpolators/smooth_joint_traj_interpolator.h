// Copyright 2022 Yifeng Zhu

#include "base_traj_interpolator.h"
#include <Eigen/Dense>
#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_SMOOTH_JOINT_TRAJ_INTERPOLATOR_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_SMOOTH_JOINT_TRAJ_INTERPOLATOR_H_

namespace traj_utils {
class SmoothJointTrajInterpolator : public BaseTrajInterpolator {
private:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector7i = Eigen::Matrix<int, 7, 1>;

  Vector7d q_start_;
  Vector7d q_goal_;
  Vector7d last_q_t_;
  Vector7d prev_q_goal_;

  Vector7d delta_q_;
  Vector7d dq_max_sync_;
  Vector7d t_1_sync_;
  Vector7d t_2_sync_;
  Vector7d t_f_sync_;
  Vector7d q_1_;

  Vector7d dq_max_ =
      (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
  Vector7d ddq_max_start_ = (Vector7d() << 1, 1, 1, 1, 1, 1, 1).finished();
  Vector7d ddq_max_goal_ = (Vector7d() << 1, 1, 1, 1, 1, 1, 1).finished();

  static constexpr double kDeltaQMotionFinished = 1e-6;

  double dt_;
  double max_time_;
  double start_time_;
  bool start_;
  bool first_goal_;
  bool speed_factor_;

public:
  inline SmoothJointTrajInterpolator()
      : dt_(0.), max_time_(1.), start_time_(0.), start_(false),
        first_goal_(true), speed_factor_(0.01) {
    dq_max_ *= speed_factor_;
    ddq_max_start_ *= speed_factor_;
    ddq_max_goal_ *= speed_factor_;
  };

  inline ~SmoothJointTrajInterpolator(){};

  inline void Reset(const double &time_sec,
                    const Eigen::Matrix<double, 7, 1> &q_start,
                    const Eigen::Matrix<double, 7, 1> &q_goal,
                    const int &policy_rate, const int &rate,
                    const double &traj_interpolator_time_fraction) {
    bool goal_changed = false;
    for (size_t i = 0; i < 7; i++) {
      if (std::abs(q_goal[i] - prev_q_goal_[i]) > kDeltaQMotionFinished) {
        goal_changed = true;
        break;
      }
    }

    if (goal_changed || !start_) {
      q_start_ = q_start;
      q_goal_ = q_goal;
      delta_q_ = q_goal_ - q_start_;
      start_ = false;

      Vector7d dq_max_reach(dq_max_);
      Vector7d t_f = Vector7d::Zero();
      Vector7d delta_t_2 = Vector7d::Zero();
      Vector7d t_1 = Vector7d::Zero();
      Vector7d delta_t_2_sync = Vector7d::Zero();
      Vector7i sign_delta_q;
      sign_delta_q << delta_q_.cwiseSign().cast<int>();

      for (size_t i = 0; i < 7; i++) {
        if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
          if (std::abs(delta_q_[i]) <
              (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
               3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i]))) {
            dq_max_reach[i] =
                std::sqrt(4.0 / 3.0 * delta_q_[i] * sign_delta_q[i] *
                          (ddq_max_start_[i] * ddq_max_goal_[i]) /
                          (ddq_max_start_[i] + ddq_max_goal_[i]));
          }
          t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
          delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
          t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 +
                   std::abs(delta_q_[i]) / dq_max_reach[i];
        }
      }
      double max_t_f = t_f.maxCoeff();
      for (size_t i = 0; i < 7; i++) {
        if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
          double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
          double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
          double c =
              std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
          double delta = b * b - 4.0 * a * c;
          if (delta < 0.0) {
            delta = 0.0;
          }
          dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
          t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
          delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
          t_f_sync_[i] = (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 +
                         std::abs(delta_q_[i] / dq_max_sync_[i]);
          t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
          q_1_[i] =
              (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
        }
      }
    }

    prev_q_goal_ = q_goal_;
    // std::cout << " Interpolator start  : " << q_start_.transpose() << " | "
    // << q_start.transpose() << std::endl; std::cout << "Goal: " <<
    // q_goal_.transpose() << std::endl;
  };

  inline void GetNextStep(const double &time_sec,
                          Eigen::Matrix<double, 7, 1> &joints) {
    if (!start_) {
      start_time_ = time_sec;
      last_q_t_ = q_start_;
      start_ = true;
    }

    // Compute relative to the starting time.
    double t = time_sec - start_time_;

    Vector7i sign_delta_q;
    sign_delta_q << delta_q_.cwiseSign().cast<int>();
    Vector7d t_d = t_2_sync_ - t_1_sync_;
    Vector7d delta_t_2_sync = t_f_sync_ - t_2_sync_;

    Vector7d delta_q;
    for (size_t i = 0; i < 7; i++) {
      if (std::abs(delta_q_[i]) < kDeltaQMotionFinished) {
        delta_q[i] = 0;
      } else {
        if (t < t_1_sync_[i]) {
          delta_q[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] *
                       sign_delta_q[i] * (0.5 * t - t_1_sync_[i]) *
                       std::pow(t, 3.0);
        } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
          delta_q[i] =
              q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
        } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
          delta_q[i] =
              delta_q_[i] +
              0.5 *
                  (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                       (t - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                       std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
                   (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] -
                    2.0 * t_d[i])) *
                  dq_max_sync_[i] * sign_delta_q[i];
        } else {
          delta_q[i] = delta_q_[i];
        }
      }
    }
    joints = q_start_ + delta_q;
  };
};
} // namespace traj_utils

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_SMOOTH_JOINT_TRAJ_INTERPOLATOR_H_
