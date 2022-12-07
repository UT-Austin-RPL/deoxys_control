#include <Eigen/Dense>

#include "controllers/base_controller.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_OSC_YAW_IMPEDANCE_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_OSC_YAW_IMPEDANCE_H_

namespace controller {
class OSCYawImpedanceController : public BaseController {
protected:
  FrankaOSCPoseControllerMessage control_msg_;
  Eigen::Matrix<double, 3, 3> Kp_p, Kp_r, Kd_p, Kd_r;

  Eigen::Matrix<double, 7, 1> static_q_task_;

  Eigen::Vector3d prev_goal_pos_EE_in_base_frame;

  Eigen::Matrix<double, 7, 1> residual_mass_vec_;

public:
  OSCYawImpedanceController();
  OSCYawImpedanceController(franka::Model &model);

  ~OSCYawImpedanceController();

  bool ParseMessage(const FrankaControlMessage &msg);

  // void ComputeGoal(const Eigen::Vector3d&, const Eigen::Quaterniond&,
  // Eigen::Vector3d&, Eigen::Quaterniond&);
  void ComputeGoal(const std::shared_ptr<StateInfo> &,
                   std::shared_ptr<StateInfo> &);

  std::array<double, 7> Step(const franka::RobotState &,
                             const Eigen::Vector3d &,
                             const Eigen::Quaterniond &);
};
} // namespace controller

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_OSC_YAW_IMPEDANCE_H_
