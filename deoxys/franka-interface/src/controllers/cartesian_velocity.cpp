// Copyright 2023 Yifeng Zhu

#include "franka_controller.pb.h"
#include "franka_robot_state.pb.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "utils/common_utils.h"
#include "utils/control_utils.h"
#include "utils/robot_utils.h"

#include "controllers/cartesian_velocity.h"

#include <memory>

namespace controller {
CartesianVelocityController::CartesianVelocityController() {}
CartesianVelocityController::~CartesianVelocityController() {}

CartesianVelocityController::CartesianVelocityController(franka::Model &model) {
  model_ = &model;
}

bool CartesianVelocityController::ParseMessage(const FrankaControlMessage &msg) {

  if (!msg.control_msg().UnpackTo(&control_msg_)) {
    return false;
  }

  speed_factor_ = control_msg_.speed_factor();

  this->state_estimator_ptr_->ParseMessage(msg.state_estimator_msg());

  return true;

}

void CartesianVelocityController::ComputeGoal(const std::shared_ptr<StateInfo> &state_info,
                std::shared_ptr<StateInfo> &goal_state_info) {
  goal_state_info->twist_trans_EE_in_base_frame = Eigen::Vector3d(control_msg_.goal().x(), control_msg_.goal().y(), control_msg_.goal().z());
  goal_state_info->twist_rot_EE_in_base_frame = Eigen::Vector3d(control_msg_.goal().ax(), control_msg_.goal().ay(), control_msg_.goal().az());
}

void CartesianVelocityController::EstimateVelocities(const franka::RobotState &robot_state, std::shared_ptr<StateInfo>& current_state_info) {
    Eigen::Matrix<double, 7, 1> current_q, current_dq;
    // Get state from a specified state estimator
    current_q = this->state_estimator_ptr_->GetCurrentJointPos();
    current_dq = this->state_estimator_ptr_->GetCurrentJointVel();

    // Estimate the current end effector velocities
    std::array<double, 42> jacobian_array =
        model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

    Eigen::MatrixXd jacobian_pos(3, 7);
    Eigen::MatrixXd jacobian_ori(3, 7);
    jacobian_pos << jacobian.block(0, 0, 3, 7);
    jacobian_ori << jacobian.block(3, 0, 3, 7);

    current_state_info->twist_trans_EE_in_base_frame << jacobian_pos * current_dq;
    current_state_info->twist_rot_EE_in_base_frame << jacobian_ori * current_dq;
  
}

std::array<double, 6> CartesianVelocityController::Step(const franka::RobotState & robot_state,
                            const Eigen::Vector3d & desired_twist_trans_EE_in_base_frame,
                            const Eigen::Vector3d & desired_twist_rot_EE_in_base_frame) {
  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();

  Eigen::Matrix<double, 6, 1> target_v;

  target_v << desired_twist_trans_EE_in_base_frame, desired_twist_rot_EE_in_base_frame;
  
  // TODO: Scale the target command with speed_factor

  std::array<double, 6> vel_d_array{};
  Eigen::VectorXd::Map(&vel_d_array[0], 6) = target_v;


  std::chrono::high_resolution_clock::time_point t2 =
      std::chrono::high_resolution_clock::now();
  return vel_d_array;
}

} // namespace controller