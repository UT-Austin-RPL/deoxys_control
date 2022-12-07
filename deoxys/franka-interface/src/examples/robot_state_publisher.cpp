// Copyright 2022 Yifeng Zhu

#include "franka_robot_state.pb.h"
#include <Eigen/Dense>
#include <atomic>
#include <bitset>
#include <chrono>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <zmqpp/zmqpp.hpp>

// TODO: add initialization method from yaml files
class FrankaRobotStatePublisher {
public:
  FrankaRobotStatePublisher(){};
  ~FrankaRobotStatePublisher(){};
  void LoadErrorState(FrankaRobotStateMessage::Errors &error_msg,
                      const franka::Errors &robot_state_error);
  void LoadRobotState(FrankaRobotStateMessage &robot_state_msg,
                      const franka::RobotState &robot_state);
};

void FrankaRobotStatePublisher::LoadErrorState(
    FrankaRobotStateMessage::Errors &error_msg,
    const franka::Errors &robot_state_error) {
  error_msg.set_joint_position_limits_violation(
      robot_state_error.joint_position_limits_violation);
  error_msg.set_cartesian_position_limits_violation(
      robot_state_error.cartesian_position_limits_violation);
  error_msg.set_self_collision_avoidance_violation(
      robot_state_error.self_collision_avoidance_violation);
  error_msg.set_joint_velocity_violation(
      robot_state_error.joint_velocity_violation);
  error_msg.set_cartesian_velocity_violation(
      robot_state_error.cartesian_velocity_violation);
  error_msg.set_force_control_safety_violation(
      robot_state_error.force_control_safety_violation);
  error_msg.set_joint_reflex(robot_state_error.joint_reflex);
  error_msg.set_cartesian_reflex(robot_state_error.cartesian_reflex);
  error_msg.set_max_goal_pose_deviation_violation(
      robot_state_error.max_goal_pose_deviation_violation);
  error_msg.set_max_path_pose_deviation_violation(
      robot_state_error.max_path_pose_deviation_violation);
  error_msg.set_cartesian_velocity_profile_safety_violation(
      robot_state_error.cartesian_velocity_profile_safety_violation);
  error_msg.set_joint_position_motion_generator_start_pose_invalid(
      robot_state_error.joint_position_motion_generator_start_pose_invalid);
  error_msg.set_joint_motion_generator_position_limits_violation(
      robot_state_error.joint_motion_generator_position_limits_violation);
  error_msg.set_joint_motion_generator_velocity_limits_violation(
      robot_state_error.joint_motion_generator_velocity_limits_violation);
  error_msg.set_joint_motion_generator_velocity_discontinuity(
      robot_state_error.joint_motion_generator_velocity_discontinuity);
  error_msg.set_joint_motion_generator_acceleration_discontinuity(
      robot_state_error.joint_motion_generator_acceleration_discontinuity);
  error_msg.set_cartesian_position_motion_generator_start_pose_invalid(
      robot_state_error.cartesian_position_motion_generator_start_pose_invalid);
  error_msg.set_cartesian_motion_generator_elbow_limit_violation(
      robot_state_error.cartesian_motion_generator_elbow_limit_violation);
  error_msg.set_cartesian_motion_generator_velocity_limits_violation(
      robot_state_error.cartesian_motion_generator_velocity_limits_violation);
  error_msg.set_cartesian_motion_generator_velocity_discontinuity(
      robot_state_error.cartesian_motion_generator_velocity_discontinuity);
  error_msg.set_cartesian_motion_generator_acceleration_discontinuity(
      robot_state_error.cartesian_motion_generator_acceleration_discontinuity);
  error_msg.set_cartesian_motion_generator_elbow_sign_inconsistent(
      robot_state_error.cartesian_motion_generator_elbow_sign_inconsistent);
  error_msg.set_cartesian_motion_generator_start_elbow_invalid(
      robot_state_error.cartesian_motion_generator_start_elbow_invalid);
  error_msg.set_cartesian_motion_generator_joint_position_limits_violation(
      robot_state_error
          .cartesian_motion_generator_joint_position_limits_violation);
  error_msg.set_cartesian_motion_generator_joint_velocity_limits_violation(
      robot_state_error
          .cartesian_motion_generator_joint_velocity_limits_violation);
  error_msg.set_cartesian_motion_generator_joint_velocity_discontinuity(
      robot_state_error
          .cartesian_motion_generator_joint_velocity_discontinuity);
  error_msg.set_cartesian_motion_generator_joint_acceleration_discontinuity(
      robot_state_error
          .cartesian_motion_generator_joint_acceleration_discontinuity);
  error_msg.set_cartesian_position_motion_generator_invalid_frame(
      robot_state_error.cartesian_position_motion_generator_invalid_frame);
  error_msg.set_force_controller_desired_force_tolerance_violation(
      robot_state_error.force_controller_desired_force_tolerance_violation);
  error_msg.set_controller_torque_discontinuity(
      robot_state_error.controller_torque_discontinuity);
  error_msg.set_start_elbow_sign_inconsistent(
      robot_state_error.start_elbow_sign_inconsistent);
  error_msg.set_communication_constraints_violation(
      robot_state_error.communication_constraints_violation);
  error_msg.set_power_limit_violation(robot_state_error.power_limit_violation);
  error_msg.set_joint_p2p_insufficient_torque_for_planning(
      robot_state_error.joint_p2p_insufficient_torque_for_planning);
  error_msg.set_tau_j_range_violation(robot_state_error.tau_j_range_violation);
  error_msg.set_instability_detected(robot_state_error.instability_detected);
  error_msg.set_joint_move_in_wrong_direction(
      robot_state_error.joint_move_in_wrong_direction);
}

void FrankaRobotStatePublisher::LoadRobotState(
    FrankaRobotStateMessage &robot_state_msg,
    const franka::RobotState &robot_state) {
  robot_state_msg.mutable_o_t_ee()->Add(robot_state.O_T_EE.begin(),
                                        robot_state.O_T_EE.end());
  robot_state_msg.mutable_o_t_ee_d()->Add(robot_state.O_T_EE_d.begin(),
                                          robot_state.O_T_EE_d.end());

  robot_state_msg.mutable_f_t_ee()->Add(robot_state.F_T_EE.begin(),
                                        robot_state.F_T_EE.end());
  robot_state_msg.mutable_f_t_ne()->Add(robot_state.F_T_NE.begin(),
                                        robot_state.F_T_NE.end());
  robot_state_msg.mutable_ne_t_ee()->Add(robot_state.NE_T_EE.begin(),
                                         robot_state.NE_T_EE.end());
  robot_state_msg.mutable_ee_t_k()->Add(robot_state.EE_T_K.begin(),
                                        robot_state.EE_T_K.end());
  robot_state_msg.set_m_ee(robot_state.m_ee);
  robot_state_msg.mutable_i_ee()->Add(robot_state.I_ee.begin(),
                                      robot_state.I_ee.end());
  robot_state_msg.mutable_f_x_cee()->Add(robot_state.F_x_Cee.begin(),
                                         robot_state.F_x_Cee.end());
  robot_state_msg.set_m_load(robot_state.m_load);
  robot_state_msg.mutable_i_load()->Add(robot_state.I_load.begin(),
                                        robot_state.I_load.end());
  robot_state_msg.mutable_f_x_cload()->Add(robot_state.F_x_Cload.begin(),
                                           robot_state.F_x_Cload.end());
  robot_state_msg.set_m_total(robot_state.m_total);
  robot_state_msg.mutable_i_total()->Add(robot_state.I_total.begin(),
                                         robot_state.I_total.end());
  robot_state_msg.mutable_f_x_ctotal()->Add(robot_state.F_x_Ctotal.begin(),
                                            robot_state.F_x_Ctotal.end());
  robot_state_msg.mutable_elbow()->Add(robot_state.elbow.begin(),
                                       robot_state.elbow.end());
  robot_state_msg.mutable_elbow_d()->Add(robot_state.elbow_d.begin(),
                                         robot_state.elbow_d.end());
  robot_state_msg.mutable_elbow_c()->Add(robot_state.elbow_c.begin(),
                                         robot_state.elbow_c.end());
  robot_state_msg.mutable_delbow_c()->Add(robot_state.delbow_c.begin(),
                                          robot_state.delbow_c.end());
  robot_state_msg.mutable_ddelbow_c()->Add(robot_state.ddelbow_c.begin(),
                                           robot_state.ddelbow_c.end());
  robot_state_msg.mutable_tau_j()->Add(robot_state.tau_J.begin(),
                                       robot_state.tau_J.end());
  robot_state_msg.mutable_tau_j_d()->Add(robot_state.tau_J_d.begin(),
                                         robot_state.tau_J_d.end());
  robot_state_msg.mutable_dtau_j()->Add(robot_state.dtau_J.begin(),
                                        robot_state.dtau_J.end());
  robot_state_msg.mutable_q()->Add(robot_state.q.begin(), robot_state.q.end());
  robot_state_msg.mutable_q_d()->Add(robot_state.q_d.begin(),
                                     robot_state.q_d.end());
  robot_state_msg.mutable_dq()->Add(robot_state.dq.begin(),
                                    robot_state.dq.end());
  robot_state_msg.mutable_dq_d()->Add(robot_state.dq_d.begin(),
                                      robot_state.dq_d.end());
  robot_state_msg.mutable_ddq_d()->Add(robot_state.ddq_d.begin(),
                                       robot_state.ddq_d.end());
  robot_state_msg.mutable_joint_contact()->Add(
      robot_state.joint_contact.begin(), robot_state.joint_contact.end());
  robot_state_msg.mutable_cartesian_contact()->Add(
      robot_state.cartesian_contact.begin(),
      robot_state.cartesian_contact.end());
  robot_state_msg.mutable_joint_collision()->Add(
      robot_state.joint_collision.begin(), robot_state.joint_collision.end());
  robot_state_msg.mutable_cartesian_collision()->Add(
      robot_state.cartesian_collision.begin(),
      robot_state.cartesian_collision.end());
  robot_state_msg.mutable_tau_ext_hat_filtered()->Add(
      robot_state.tau_ext_hat_filtered.begin(),
      robot_state.tau_ext_hat_filtered.end());
  robot_state_msg.mutable_o_f_ext_hat_k()->Add(
      robot_state.O_F_ext_hat_K.begin(), robot_state.O_F_ext_hat_K.end());
  robot_state_msg.mutable_k_f_ext_hat_k()->Add(
      robot_state.K_F_ext_hat_K.begin(), robot_state.K_F_ext_hat_K.end());
  robot_state_msg.mutable_o_dp_ee_d()->Add(robot_state.O_dP_EE_d.begin(),
                                           robot_state.O_dP_EE_d.end());
  robot_state_msg.mutable_o_t_ee_c()->Add(robot_state.O_T_EE_c.begin(),
                                          robot_state.O_T_EE_c.end());
  robot_state_msg.mutable_o_dp_ee_c()->Add(robot_state.O_dP_EE_c.begin(),
                                           robot_state.O_dP_EE_c.end());
  robot_state_msg.mutable_o_ddp_ee_c()->Add(robot_state.O_ddP_EE_c.begin(),
                                            robot_state.O_ddP_EE_c.end());
  robot_state_msg.mutable_theta()->Add(robot_state.theta.begin(),
                                       robot_state.theta.end());
  robot_state_msg.mutable_dtheta()->Add(robot_state.dtheta.begin(),
                                        robot_state.dtheta.end());

  // Error
  // FrankaRobotStateMessage::Errors current_errors;
  FrankaRobotStateMessage::Errors *current_error_ptr =
      new FrankaRobotStateMessage::Errors();
  this->LoadErrorState(*current_error_ptr, robot_state.current_errors);
  // FrankaRobotStateMessage::Errors last_motion_errors;
  FrankaRobotStateMessage::Errors *last_motion_error_ptr =
      new FrankaRobotStateMessage::Errors();
  this->LoadErrorState(*last_motion_error_ptr, robot_state.last_motion_errors);

  robot_state_msg.set_allocated_current_errors(current_error_ptr);
  robot_state_msg.set_allocated_last_motion_errors(last_motion_error_ptr);

  robot_state_msg.set_control_command_success_rate(
      robot_state.control_command_success_rate);

  FrankaRobotStateMessage::Duration *time =
      new FrankaRobotStateMessage::Duration();
  time->set_tosec(robot_state.time.toSec());
  time->set_tomsec(robot_state.time.toMSec());

  robot_state_msg.set_allocated_time(time);

  // set robot mode

  switch (robot_state.robot_mode) {
  case franka::RobotMode::kOther:
    robot_state_msg.set_robot_mode(FrankaRobotStateMessage_RobotMode_Other);
    break;
  case franka::RobotMode::kIdle:
    robot_state_msg.set_robot_mode(FrankaRobotStateMessage_RobotMode_Idle);
    break;
  case franka::RobotMode::kMove:
    robot_state_msg.set_robot_mode(FrankaRobotStateMessage_RobotMode_Move);
    break;
  case franka::RobotMode::kGuiding:
    robot_state_msg.set_robot_mode(FrankaRobotStateMessage_RobotMode_Guiding);
    break;
  case franka::RobotMode::kReflex:
    robot_state_msg.set_robot_mode(FrankaRobotStateMessage_RobotMode_Reflex);
    break;
  case franka::RobotMode::kUserStopped:
    robot_state_msg.set_robot_mode(
        FrankaRobotStateMessage_RobotMode_UserStopped);
    break;
  case franka::RobotMode::kAutomaticErrorRecovery:
    robot_state_msg.set_robot_mode(
        FrankaRobotStateMessage_RobotMode_AutomaticErrorRecovery);
    break;
  }
}

int main(int argc, char **argv) {

  const double robot_state_rate = 100.0;

  struct {
    std::mutex mutex;
  } state_pub{};
  std::atomic_bool running{true};

  const std::string pub_host = "tcp://*:5556/";
  zmqpp::context context;
  zmqpp::socket_type type = zmqpp::socket_type::publish;
  zmqpp::socket socket(context, type);
  socket.bind(pub_host);

  FrankaRobotStatePublisher robot_state_publisher;

  try {
    franka::Robot robot(argv[1]);

    std::thread state_publish_thread([robot_state_rate, &robot, &socket,
                                      &robot_state_publisher, &state_pub,
                                      &running]() {
      while (running) {
        FrankaRobotStateMessage robot_state_msg;

        std::this_thread::sleep_for(std::chrono::milliseconds(
            static_cast<int>(1.0 / robot_state_rate * 1000.0)));

        if (state_pub.mutex.try_lock()) {
          std::chrono::high_resolution_clock::time_point start_t =
              std::chrono::high_resolution_clock::now();

          std::cout << 1 << std::endl;
          franka::RobotState current_robot_state;
          robot.read(
              [&current_robot_state](const franka::RobotState &robot_state) {
                current_robot_state = robot_state;
                return 0;
              });
          robot_state_publisher.LoadRobotState(robot_state_msg,
                                               current_robot_state);

          std::string serialized_robot_state_msg;
          robot_state_msg.SerializeToString(&serialized_robot_state_msg);
          for (size_t i = 0; i < 7; i++) {
            std::cout << current_robot_state.O_F_ext_hat_K[i] << " ";
          }
          std::cout << robot_state_msg.time().tosec() << std::endl;
          std::cout << std::endl;
          socket.send(serialized_robot_state_msg);
          std::chrono::high_resolution_clock::time_point end_t =
              std::chrono::high_resolution_clock::now();
          std::cout << "Time: "
                    << std::chrono::duration_cast<std::chrono::microseconds>(
                           end_t - start_t)
                           .count()
                    << std::endl;
          state_pub.mutex.unlock();
        }
      }
    });

    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (state_publish_thread.joinable()) {
      state_publish_thread.join();
    }
  } catch (franka::Exception const &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  socket.disconnect(pub_host);

  return 0;
}