// Copyright 2023 Yifeng Zhu

#include "controllers/base_controller.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_CARTESIAN_VELOCITY_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_CARTESIAN_VELOCITY_H_

namespace controller {
class CartesianVelocityController : public BaseController {
protected:
  FrankaCartesianVelocityControllerMessage control_msg_;

  double speed_factor_;

public:
  CartesianVelocityController();
  CartesianVelocityController(franka::Model &model);

  ~CartesianVelocityController();

  bool ParseMessage(const FrankaControlMessage &msg);

  void ComputeGoal(const std::shared_ptr<StateInfo> &state_info,
                   std::shared_ptr<StateInfo> &goal_info);

  std::array<double, 6> Step(const franka::RobotState &,
                             const Eigen::Vector3d &,
                             const Eigen::Vector3d &);
  void EstimateVelocities(const franka::RobotState &robot_state,
                          std::shared_ptr<StateInfo> &);
};

} // namespace controller

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_CARTESIAN_VELOCITY_H_



