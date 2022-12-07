// Copyright 2022 Yifeng Zhu

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

#include "controllers/joint_position.h"

#include <memory>

namespace controller {
JointPositionController::JointPositionController() {}
JointPositionController::~JointPositionController() {}

JointPositionController::JointPositionController(franka::Model &model) {
  model_ = &model;
}

bool JointPositionController::ParseMessage(const FrankaControlMessage &msg) {

  if (!msg.control_msg().UnpackTo(&control_msg_)) {
    return false;
  }

  speed_factor_ = control_msg_.speed_factor();
  return true;
}

void JointPositionController::ComputeGoal(
    const std::shared_ptr<StateInfo> &current_state_info,
    std::shared_ptr<StateInfo> &goal_state_info) {
  if (control_msg_.goal().is_delta()) {
    Eigen::Matrix<double, 7, 1> delta_joint_position;
    delta_joint_position << control_msg_.goal().q1(), control_msg_.goal().q2(),
        control_msg_.goal().q3(), control_msg_.goal().q4(),
        control_msg_.goal().q5(), control_msg_.goal().q6(),
        control_msg_.goal().q7();
    goal_state_info->joint_positions =
        current_state_info->joint_positions + delta_joint_position;
  } else {
    goal_state_info->joint_positions << control_msg_.goal().q1(),
        control_msg_.goal().q2(), control_msg_.goal().q3(),
        control_msg_.goal().q4(), control_msg_.goal().q5(),
        control_msg_.goal().q6(), control_msg_.goal().q7();
  }

  // goal_state_info->joint_positions << control_msg_.goal().q1(),
  // control_msg_.goal().q2(), control_msg_.goal().q3(),
  // control_msg_.goal().q4(), control_msg_.goal().q5(),
  // control_msg_.goal().q6(), control_msg_.goal().q7();
}

std::array<double, 7>
JointPositionController::Step(const franka::RobotState &robot_state,
                              const Eigen::Matrix<double, 7, 1> &desired_q_) {

  std::array<double, 7> q_d_array{};
  Eigen::VectorXd::Map(&q_d_array[0], 7) = desired_q_;
  return q_d_array;
}

bool checkFinished(const double &time,
                   const Eigen::Matrix<double, 7, 1> &delta_q_d) {
  // Check if joint position control is finished
}

} // namespace controller
