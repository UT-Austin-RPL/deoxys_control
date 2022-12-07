// Copyright 2022 Yifeng Zhu

#include <Eigen/Dense>

#include "controllers/base_controller.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_OSC_IMPEDANCE_LOW_FREQ_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_OSC_IMPEDANCE_LOW_FREQ_H_

namespace controller {
class LowFreqOSCImpedanceController : public BaseController {
protected:
  FrankaOSCPoseControllerMessage control_msg_;
  Eigen::Matrix<double, 3, 3> Kp_p, Kp_r, Kd_p, Kd_r;

  Eigen::Matrix<double, 7, 1> M_;
  Eigen::Matrix<double, 7, 7> M_inv_;
  Eigen::Matrix<double, 6, 7> jacobian_;
  Eigen::Matrix<double, 7, 1> coriolis_;

  int counter_;

public:
  OSCImpedanceController();
  OSCImpedanceController(franka::Model &model);

  ~OSCImpedanceController();

  bool ParseMessage(const FrankaControlMessage &msg);

  void ComputeGoal(const Eigen::Vector3d &, const Eigen::Quaterniond &,
                   Eigen::Vector3d &, Eigen::Quaterniond &);

  std::array<double, 7> Step(const franka::RobotState &,
                             const Eigen::Vector3d &,
                             const Eigen::Quaterniond &);
};
} // namespace controller

#endif //  DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_OSC_IMPEDANCE_LOW_FREQ_H_
