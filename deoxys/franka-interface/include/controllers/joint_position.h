// Copyright 2022 Yifeng Zhu

#include <Eigen/Dense>

#include "controllers/base_controller.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_JOINT_POSITION_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_JOINT_POSITION_H_

namespace controller {
class JointPositionController : public BaseController {
protected:
  FrankaJointPositionControllerMessage control_msg_;

  double kp, kd;
  double speed_factor_;

public:
  JointPositionController();
  JointPositionController(franka::Model &model);

  ~JointPositionController();

  bool ParseMessage(const FrankaControlMessage &msg);

  // void ComputeGoal(const Eigen::Matrix<double, 7, 1>&, Eigen::Matrix<double,
  // 7, 1>&);
  void ComputeGoal(const std::shared_ptr<StateInfo> &state_info,
                   std::shared_ptr<StateInfo> &goal_info);

  std::array<double, 7> Step(const franka::RobotState &,
                             const Eigen::Matrix<double, 7, 1> &);
  bool checkFinished(const double &, const Eigen::Matrix<double, 7, 1> &);
};
} // namespace controller

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_JOINT_POSITION_H_
