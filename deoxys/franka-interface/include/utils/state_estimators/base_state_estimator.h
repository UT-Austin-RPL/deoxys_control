// Copyright 2022 Yifeng Zhu

#include <Eigen/Dense>

#include "franka_controller.pb.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wreturn-type"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_STATE_ESTIMATORS_BASE_STATE_ESTIMATOR_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_STATE_ESTIMATORS_BASE_STATE_ESTIMATOR_H_

namespace estimator_utils {
class BaseStateEstimator {
protected:
  Eigen::Matrix<double, 7, 1> estimated_current_q_, estimated_prev_q_,
      estimated_current_dq_;
  Eigen::Matrix<double, 7, 1> raw_current_q_, raw_current_dq_;

  Eigen::Matrix<double, 3, 1> estimated_current_eef_pos_,
      estimated_prev_eef_pos_;
  Eigen::Quaterniond estimated_current_eef_quat_;

  Eigen::Matrix<double, 3, 1> raw_current_eef_pos_;
  Eigen::Quaterniond raw_current_eef_quat_;

  bool is_first_state_;
  bool is_estimation_;

public:
  inline BaseStateEstimator(){};
  inline virtual ~BaseStateEstimator(){};
  inline bool IsFirstState() { return is_first_state_; };
  inline virtual void Reset(){};

  inline virtual bool ParseMessage(const FrankaStateEstimatorMessage &msg){};

  inline virtual void Step(const Eigen::Matrix<double, 7, 1> &q,
                           const Eigen::Matrix<double, 7, 1> &dq){};
  inline virtual void Step(const Eigen::Vector3d &eef_pos,
                           const Eigen::Quaterniond &eef_quat){};

  inline void Initialize(const Eigen::Matrix<double, 7, 1> &q,
                         const Eigen::Matrix<double, 7, 1> &dq) {
    raw_current_q_ = q;
    raw_current_dq_ = dq;
    estimated_current_q_ = q;
    estimated_prev_q_ = q;
    is_first_state_ = false;
  };

  inline void Initialize(const Eigen::Vector3d &eef_pos,
                         const Eigen::Quaterniond &eef_quat) {
    raw_current_eef_pos_ = eef_pos;
    raw_current_eef_quat_ = eef_quat;
    estimated_current_eef_pos_ = eef_pos;
    estimated_current_eef_quat_ = eef_quat;
    estimated_prev_eef_pos_ = eef_pos;
    is_first_state_ = false;
  };

  inline void Initialize(const Eigen::Matrix<double, 7, 1> &q,
                         const Eigen::Matrix<double, 7, 1> &dq,
                         const Eigen::Vector3d &eef_pos,
                         const Eigen::Quaterniond &eef_quat) {
    this->Initialize(q, dq);
    this->Initialize(eef_pos, eef_quat);
  };

  inline void Initialize(const Eigen::Vector3d &eef_pos,
                         const Eigen::Quaterniond &eef_quat,
                         const Eigen::Matrix<double, 7, 1> &q,
                         const Eigen::Matrix<double, 7, 1> &dq) {
    this->Initialize(q, dq, eef_pos, eef_quat);
  };

  inline void Update(const Eigen::Matrix<double, 7, 1> &q,
                     const Eigen::Matrix<double, 7, 1> &dq) {
    this->Step(q, dq);
  };

  // Make sure that your estimated quat and the current observation quat are not
  // in the flipped representation. Otherwise could cause big issue
  inline void Update(const Eigen::Vector3d &eef_pos,
                     const Eigen::Quaterniond &eef_quat) {
    this->Step(eef_pos, eef_quat);
  };

  inline void Update(const Eigen::Vector3d &eef_pos,
                     const Eigen::Quaterniond &eef_quat,
                     const Eigen::Matrix<double, 7, 1> &q,
                     const Eigen::Matrix<double, 7, 1> &dq) {
    this->Step(eef_pos, eef_quat);
    this->Step(q, dq);
  }
  inline void Update(const Eigen::Matrix<double, 7, 1> &q,
                     const Eigen::Matrix<double, 7, 1> &dq,
                     const Eigen::Vector3d &eef_pos,
                     const Eigen::Quaterniond &eef_quat) {
    this->Update(eef_pos, eef_quat, q, dq);
  }

  inline Eigen::Matrix<double, 7, 1> GetCurrentJointPos() {
    if (is_estimation_)
      return estimated_current_q_;
    else
      return raw_current_q_;
  };
  inline Eigen::Matrix<double, 7, 1> GetCurrentJointVel() {
    if (is_estimation_)
      return estimated_current_dq_;
    else
      return raw_current_dq_;
  };
  inline Eigen::Vector3d GetCurrentEEFPos() {
    if (is_estimation_)
      return estimated_current_eef_pos_;
    else
      return raw_current_eef_pos_;
  };

  inline Eigen::Quaterniond GetCurrentEEFQuat() {
    if (is_estimation_)
      return estimated_current_eef_quat_;
    else
      return raw_current_eef_quat_;
  };
};
} // NAMESPACE estimator_utils

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_STATE_ESTIMATORS_BASE_STATE_ESTIMATOR_H_
#pragma GCC diagnostic pop