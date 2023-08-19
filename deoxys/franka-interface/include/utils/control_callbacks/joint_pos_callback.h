// Copyright 2022 Yifeng Zhu

#include <chrono>
#include <franka/model.h>
#include <franka/robot.h>

#include "utils/control_utils.h"
#include "utils/robot_utils.h"
#include "utils/shared_memory.h"
#include "utils/shared_state.h"

#ifndef UTILS_CONTROL_CALLBACKS_JOINT_POS_CALLBACK_H_
#define UTILS_CONTROL_CALLBACKS_JOINT_POS_CALLBACK_H_

namespace control_callbacks {
std::function<franka::JointPositions(const franka::RobotState &,
                                     franka::Duration)>
CreateJointPositionCallback(
    const std::shared_ptr<SharedMemory> &global_handler,
    const std::shared_ptr<robot_utils::StatePublisher> state_publisher,
    const franka::Model &model, std::shared_ptr<StateInfo> &current_state_info,
    std::shared_ptr<StateInfo> &goal_state_info, const int &policy_rate,
    const int &traj_rate) {
  return [&global_handler, &state_publisher, &model, &current_state_info,
          &goal_state_info, &policy_rate,
          &traj_rate](const franka::RobotState &robot_state,
                      franka::Duration period) -> franka::JointPositions {
    std::chrono::high_resolution_clock::time_point t1 =
        std::chrono::high_resolution_clock::now();
    current_state_info->joint_positions =
        Eigen::VectorXd::Map(robot_state.q_d.data(), 7);
    Eigen::Affine3d current_T_EE_in_base_frame(
        Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    current_state_info->pos_EE_in_base_frame
        << current_T_EE_in_base_frame.translation();
    current_state_info->quat_EE_in_base_frame =
        Eigen::Quaterniond(current_T_EE_in_base_frame.linear());

    if (!global_handler->running) {
      std::array<double, 7> finished_joint_positions;
      Eigen::VectorXd::Map(&finished_joint_positions[0], 7) =
          current_state_info->joint_positions;

      franka::JointPositions target_joint_positions(finished_joint_positions);
      return franka::MotionFinished(target_joint_positions);
    }

    if (global_handler->time == 0.0) {
      global_handler->traj_interpolator_ptr->Reset(
          global_handler->time, current_state_info->joint_positions,
          goal_state_info->joint_positions, policy_rate, traj_rate,
          global_handler->traj_interpolator_time_fraction);
    }
    global_handler->time += period.toSec();
    Eigen::Matrix<double, 7, 1> desired_q;

    global_handler->traj_interpolator_ptr->GetNextStep(global_handler->time,
                                                       desired_q);

    state_publisher->UpdateNewState(robot_state, &model);

    std::array<double, 7> joint_positions;
    joint_positions =
        global_handler->controller_ptr->Step(robot_state, desired_q);
    // TODO: Regularize delta q d
    // std::cout << "Desired: " << desired_q.transpose() << std::endl;
    // std::cout << " Current : " << current_joint_positions.transpose() <<
    // std::endl; std::array<double, 7> joint_positions;
    // Eigen::VectorXd::Map(&joint_positions[0], 7) = desired_q;
    franka::JointPositions output(joint_positions);
    std::chrono::high_resolution_clock::time_point t2 =
        std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    // global_handler->logger->debug("{0} microseconds" , time.count());
    return output;
  };
}

} // NAMESPACE control_callbacks

#endif // UTILS_CONTROL_CALLBACKS_JOINT_POS_CALLBACK_H_
