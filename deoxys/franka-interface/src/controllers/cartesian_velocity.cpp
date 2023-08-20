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