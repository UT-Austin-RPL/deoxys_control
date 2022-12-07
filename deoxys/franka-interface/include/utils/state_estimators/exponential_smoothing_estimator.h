// Copyright 2022 Yifeng Zhu

#include <Eigen/Dense>

#include "franka_controller.pb.h"
#include "utils/state_estimators/base_state_estimator.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_STATE_ESTIMATORS_EXPONENTIAL_SMOOTHING_ESTIMATOR_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_STATE_ESTIMATORS_EXPONENTIAL_SMOOTHING_ESTIMATOR_H_

namespace estimator_utils {
class ExponentialSmoothingEstimator : public BaseStateEstimator {
protected:
  double alpha_q_;
  double alpha_dq_;
  double alpha_eef_;
  double alpha_eef_vel_;
  ExponentialSmoothingConfig state_estimator_config_;

public:
  inline ExponentialSmoothingEstimator() {
    is_estimation_ = false;
    is_first_state_ = true;
  };
  inline ~ExponentialSmoothingEstimator(){};

  inline void Reset() { is_first_state_ = true; }

  inline bool ParseMessage(const FrankaStateEstimatorMessage &msg) {
    if (!msg.config().UnpackTo(&state_estimator_config_)) {
      return false;
    }
    is_estimation_ = msg.is_estimation();

    // parse weights for exponential smoothing over joint positions and joint
    // velocities
    alpha_q_ = state_estimator_config_.alpha_q();
    alpha_dq_ = state_estimator_config_.alpha_dq();

    // parse weights for exponential smoothing over end effector displacements
    alpha_eef_ = state_estimator_config_.alpha_eef();
    // (TODO) placeholder for exponential smoothing over end effector twists
    alpha_eef_vel_ = state_estimator_config_.alpha_eef_vel();
    return true;
  };

  inline void Step(const Eigen::Matrix<double, 7, 1> &q,
                   const Eigen::Matrix<double, 7, 1> &dq) {
    if (is_estimation_) {
      // If estimating states, do exponential smoothing on joint positions and
      // joint velocities respectively
      raw_current_q_ << q;
      estimated_current_q_ << alpha_q_ * raw_current_q_ +
                                  (1 - alpha_q_) * estimated_current_q_;
      raw_current_dq_ << (estimated_current_q_ - estimated_prev_q_) / 0.001;
      raw_current_dq_ = raw_current_dq_.unaryExpr(
          [](double x) { return (abs(x) < 5e-2) ? 0. : x; });
      estimated_current_dq_ << alpha_dq_ * raw_current_dq_ +
                                   (1 - alpha_dq_) * estimated_current_dq_;
      estimated_current_dq_ = estimated_current_dq_.unaryExpr(
          [](double x) { return (abs(x) < 5e-2) ? 0. : x; });
    } else {
      // do not estimate, onlly save raw data
      raw_current_q_ << q;
      estimated_current_q_ << raw_current_q_;
      raw_current_dq_ << dq;
      estimated_current_dq_ << raw_current_dq_;
    }
    estimated_prev_q_ = estimated_current_q_;
  };

  inline void Step(const Eigen::Vector3d &eef_pos,
                   const Eigen::Quaterniond &eef_quat) {
    raw_current_eef_pos_ << eef_pos;
    raw_current_eef_quat_ = eef_quat;
    if (is_estimation_) {
      // If estimating states, do exponential smoothing on position and
      // orientation respectively
      estimated_current_eef_pos_
          << alpha_eef_ * raw_current_eef_pos_ +
                 (1 - alpha_eef_) * estimated_current_eef_pos_;
      if (eef_quat.coeffs().dot(estimated_current_eef_quat_.coeffs()) < 0.0) {
        estimated_current_eef_quat_.coeffs()
            << -estimated_current_eef_quat_.coeffs();
      }
      estimated_current_eef_quat_ =
          estimated_current_eef_quat_.slerp(alpha_eef_, eef_quat);
    } else {
      // do not estimate, onlly save raw data
      estimated_current_eef_pos_ << raw_current_eef_pos_;
      estimated_current_eef_quat_ = raw_current_eef_quat_;
    }
  };
};
} // namespace estimator_utils

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_STATE_ESTIMATORS_EXPONENTIAL_SMOOTHING_ESTIMATOR_H_