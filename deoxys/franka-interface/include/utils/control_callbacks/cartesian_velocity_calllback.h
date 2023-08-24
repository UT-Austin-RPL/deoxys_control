// Copyright 2023 Yifeng Zhu

#include <chrono>
#include <franka/model.h>
#include <franka/robot.h>

#include "utils/control_utils.h"
#include "utils/robot_utils.h"
#include "utils/shared_memory.h"
#include "utils/shared_state.h"


#ifndef UTILS_CONTROL_CALLBACKS_CARTESIAN_VELOCITY_CALLBACK_H_
#define UTILS_CONTROL_CALLBACKS_CARTESIAN_VELOCITY_CALLBACK_H_


namespace control_callbacks {
    
    std::function<franka::CartesianVelocities(const franka::RobotState &,
                                               franka::Duration)>
    CreateCartesianVelocitiesCallback(
        const std::shared_ptr<SharedMemory> &global_handler,
        const std::shared_ptr<robot_utils::StatePublisher> state_publisher,
        const franka::Model &model, std::shared_ptr<StateInfo> &current_state_info,
        std::shared_ptr<StateInfo> &goal_state_info, const int &policy_rate,
        const int &traj_rate) {
            return [&global_handler, &state_publisher, &model, &current_state_info, &goal_state_info, &policy_rate, &traj_rate](const franka::RobotState &robot_state,
            franka::Duration period) -> franka::CartesianVelocities {
                std::chrono::high_resolution_clock::time_point t1 =
                    std::chrono::high_resolution_clock::now();

                current_state_info->joint_positions =
                    Eigen::VectorXd::Map(robot_state.q.data(), 7);
                Eigen::Affine3d current_T_EE_in_base_frame(
                    Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));

                global_handler->controller_ptr->EstimateVelocities(robot_state, current_state_info);

                current_state_info->pos_EE_in_base_frame
                    << current_T_EE_in_base_frame.translation();
                current_state_info->quat_EE_in_base_frame =
                    Eigen::Quaterniond(current_T_EE_in_base_frame.linear());
                
                if (!global_handler->running) {
                franka::CartesianVelocities zero_velocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
                return franka::MotionFinished(zero_velocities);
                }

                std::array<double, 6> vel_d_array{};
                if (global_handler->time == 0.) {
                global_handler->traj_interpolator_ptr->Reset(
                    0., current_state_info->twist_trans_EE_in_base_frame,
                    current_state_info->twist_rot_EE_in_base_frame,
                    goal_state_info->twist_trans_EE_in_base_frame,
                    goal_state_info->twist_rot_EE_in_base_frame, policy_rate, traj_rate,
                    global_handler->traj_interpolator_time_fraction);
                }
                global_handler->time += period.toSec();

                Eigen::Vector3d desired_twist_trans_EE_in_base_frame;
                Eigen::Vector3d desired_twist_rot_EE_in_base_frame;

                global_handler->traj_interpolator_ptr->GetNextStep(
                    global_handler->time, desired_twist_trans_EE_in_base_frame,
                    desired_twist_rot_EE_in_base_frame);

                state_publisher->UpdateNewState(robot_state, &model);

                vel_d_array = global_handler->controller_ptr->Step(
                    robot_state, desired_twist_trans_EE_in_base_frame,
                    desired_twist_rot_EE_in_base_frame);
                control_utils::CartesianVelocitySafetyGuardFn(vel_d_array,
                                                                global_handler->min_trans_speed,
                                                                global_handler->max_trans_speed,
                                                                global_handler->min_rot_speed,
                                                                global_handler->max_rot_speed
                );
                std::chrono::high_resolution_clock::time_point t2 =
                    std::chrono::high_resolution_clock::now();
                auto time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
                global_handler->logger->debug("{0} microseconds" , time.count());
                return vel_d_array;
            };
        }
} // namespace control_callbacks

#endif // UTILS_CONTROL_CALLBACKS_CARTESIAN_VELOCITY_CALLBACK_H_

