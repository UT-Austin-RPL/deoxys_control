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
#include "utils/traj_interpolators/pose_traj_interpolator.h"
#include "utils/traj_interpolators/position_traj_interpolator.h"

#include "controllers/osc_impedance.h"

#include <memory>

using namespace controller;

int main(int argc, char **argv) {

  YAML::Node config = YAML::LoadFile(argv[1]);

  const std::string pub_port = config["NUC"]["PUB_PORT"].as<std::string>();
  const std::string subscriber_ip = config["PC"]["IP"].as<std::string>();
  const std::string sub_port = config["NUC"]["SUB_PORT"].as<std::string>();
  const std::string robot_ip = config["ROBOT"]["IP"].as<std::string>();

  double translation_stiffness = 150.0;
  double rotation_stiffness = 10.0;

  double max_torque = 5.0;
  double min_torque = -5.0;

  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  damping.setZero();
  stiffness.topLeftCorner(3, 3)
      << translation_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3)
      << rotation_stiffness * Eigen::MatrixXd::Identity(3, 3);
  // critical damping example
  damping.topLeftCorner(3, 3)
      << 2.0 * sqrt(translation_stiffness) * Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3)
      << 2.0 * sqrt(rotation_stiffness) * Eigen::MatrixXd::Identity(3, 3);
  Eigen::MatrixXd t_stiffness(3, 3), r_stiffness(3, 3), t_damping(3, 3),
      r_damping(3, 3);
  t_stiffness << stiffness.topLeftCorner(3, 3);
  r_stiffness << stiffness.bottomRightCorner(3, 3);
  t_damping << damping.topLeftCorner(3, 3);
  r_damping << damping.bottomRightCorner(3, 3);

  Eigen::Vector3d relative_pos;
  // axis angle
  Eigen::Vector3d relative_ori;

  relative_pos << 0.1, 0.0, 0.0;
  relative_ori << 0.0, 0.0, 0.0;

  try {
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();

    Eigen::Affine3d init_T_EE_in_base_frame(
        Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d goal_pos_EE_in_base_frame(
        init_T_EE_in_base_frame.translation() + relative_pos);

    Eigen::AngleAxisd relative_axis_angle;
    AxisAngle(relative_ori, relative_axis_angle);

    Eigen::Matrix3d rotation = relative_axis_angle.toRotationMatrix();
    Eigen::Quaterniond goal_quat_EE_in_base_frame(
        rotation * init_T_EE_in_base_frame.linear());

    // robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    // 100.0}},
    // 			       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    // 100.0}},
    // 			       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
    // 			       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    robot.setCollisionBehavior({{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});

    std::atomic_bool running{true};

    struct {

    } print_data{};

    // std::thread mok_sub_thread([&goal, &relative_pos, &running]() {
    //   while (running) {
    // 	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // 	if (goal.mutex.try_lock()) {
    // 	  relative_pos << -relative_pos;
    // 	  goal.pos = goal.pos - relative_pos;
    // 	  goal.mutex.unlock();
    // 	}
    //   }
    // });

    // traj_utils::LinearPositionTrajInterpolator position_interpolator;
    traj_utils::LinearPoseTrajInterpolator pose_interpolator;

    double time = 0;
    std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
        osc_control_callback = [&](const franka::RobotState &robot_state,
                                   franka::Duration period) -> franka::Torques {
      // Control command
      Eigen::Matrix<double, 7, 1> tau_d;

      // mass
      std::array<double, 49> mass_array = model.mass(robot_state);
      Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data());

      // coriolis and gravity
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(
          coriolis_array.data());

      std::array<double, 7> gravity_array = model.gravity(robot_state);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(
          gravity_array.data());

      // Robot Joint state
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      // Jacobian
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(
          jacobian_array.data());

      // If decouple control
      Eigen::MatrixXd jacobian_pos(3, 7);
      Eigen::MatrixXd jacobian_ori(3, 7);
      jacobian_pos << jacobian.block(0, 0, 3, 7);
      jacobian_ori << jacobian.block(3, 0, 3, 7);

      // End effector pose in base frame
      Eigen::Affine3d T_EE_in_base_frame(
          Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d pos_EE_in_base_frame(T_EE_in_base_frame.translation());
      Eigen::Quaterniond quat_EE_in_base_frame(T_EE_in_base_frame.linear());

      goal_pos_EE_in_base_frame = goal_pos_EE_in_base_frame + relative_pos;
      AxisAngle(relative_ori, relative_axis_angle);
      rotation = relative_axis_angle.toRotationMatrix();
      goal_quat_EE_in_base_frame = Eigen::Quaterniond(
          rotation * goal_quat_EE_in_base_frame.toRotationMatrix());

      Eigen::Matrix3d rotation();

      if (time == 0.) {
        // position_interpolator.Reset(time, pos_EE_in_base_frame,
        // goal_pos_EE_in_base_frame, 20, 500);
        pose_interpolator.Reset(
            time, pos_EE_in_base_frame, quat_EE_in_base_frame,
            goal_pos_EE_in_base_frame, goal_quat_EE_in_base_frame, 20, 500);
      }
      time += period.toSec();

      // Eigen::Vector3d desired_pos_EE_in_base_frame;
      // Eigen::Quaterniond desired_quat_EE_in_base_frame;
      // position_interpolator.GetNextStep(time, desired_pos_EE_in_base_frame);
      // desired_quat_EE_in_base_frame = goal_quat_EE_in_base_frame;

      Eigen::Vector3d desired_pos_EE_in_base_frame;
      Eigen::Quaterniond desired_quat_EE_in_base_frame;
      pose_interpolator.GetNextStep(time, desired_pos_EE_in_base_frame,
                                    desired_quat_EE_in_base_frame);

      if ((pos_EE_in_base_frame - goal_pos_EE_in_base_frame).norm() < 1e-2 &&
          fabs(quat_EE_in_base_frame.dot(goal_quat_EE_in_base_frame)) >
              0.99999) {
        franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        return franka::MotionFinished(zero_torques);
      }

      if (time > 2) {
        franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        return franka::MotionFinished(zero_torques);
      }

      // Joint velocity
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

      // w: quat.w() x: quat.x() y: quat.y() z: quat.z()
      if (desired_quat_EE_in_base_frame.coeffs().dot(
              quat_EE_in_base_frame.coeffs()) < 0.0) {
        quat_EE_in_base_frame.coeffs() << -quat_EE_in_base_frame.coeffs();
      }

      // Compute position error and orientation error
      Eigen::Vector3d pos_error;
      // Eigen::Vector3d ori_error;
      pos_error << desired_pos_EE_in_base_frame - pos_EE_in_base_frame;
      Eigen::Quaterniond quat_error(desired_quat_EE_in_base_frame.inverse() *
                                    quat_EE_in_base_frame);

      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << pos_error;

      error.tail(3) << quat_error.x(), quat_error.y(), quat_error.z();
      error.tail(3) << T_EE_in_base_frame.linear() * error.tail(3);
      Eigen::Vector3d ori_error;
      ori_error << quat_error.x(), quat_error.y(), quat_error.z();
      ori_error << T_EE_in_base_frame.linear() * ori_error;

      // Compute matrices
      Eigen::Matrix<double, 7, 1> hack_vec;
      hack_vec << 0., 0., 0., 0., 0.1, 0.1, 0.1;
      M = M + Eigen::Matrix<double, 7, 7>(hack_vec.asDiagonal());
      Eigen::Matrix<double, 7, 7> M_inv(M.inverse());
      Eigen::MatrixXd Lambda_inv(6, 6);
      Lambda_inv << jacobian * M_inv * jacobian.transpose();
      Eigen::MatrixXd Lambda(6, 6);
      control_utils::PInverse(Lambda_inv, Lambda);

      Eigen::Matrix<double, 7, 6> J_inv;
      J_inv << M_inv * jacobian.transpose() * Lambda;
      Eigen::Matrix<double, 7, 7> Nullspace;
      Nullspace << Eigen::MatrixXd::Identity(7, 7) -
                       jacobian.transpose() * J_inv.transpose();

      // Decoupled mass matrices
      Eigen::MatrixXd Lambda_pos_inv(3, 3);
      Lambda_pos_inv << jacobian_pos * M_inv * jacobian_pos.transpose();
      Eigen::MatrixXd Lambda_ori_inv(3, 3);
      Lambda_ori_inv << jacobian_ori * M_inv * jacobian_ori.transpose();

      Eigen::MatrixXd Lambda_pos(3, 3);
      Eigen::MatrixXd Lambda_ori(3, 3);

      // Print robot state
      std::cout << "Robot State" << std::endl;
      std::cout << q.transpose() << std::endl;
      std::cout << "mass total : " << robot_state.m_total << std::endl;
      for (auto &s : robot_state.I_total) {
        std::cout << s << " ";
      }
      std::cout << std::endl;
      control_utils::PInverse(Lambda_pos_inv, Lambda_pos);

      double epsilon = 0.000025;
      // Print matrices
      std::cout << "Mass" << std::endl;
      std::cout << M << std::endl;
      std::cout << "Jacobian" << std::endl;
      std::cout << jacobian << std::endl;
      std::cout << "Jacobian pos" << std::endl;
      std::cout << jacobian_pos << std::endl;
      std::cout << "Jacobian ori" << std::endl;
      std::cout << jacobian_ori << std::endl;
      std::cout << "Epsilon: " << epsilon << std::endl;
      control_utils::PInverse(Lambda_ori_inv, Lambda_ori, epsilon);
      std::cout << "Lambda: " << Lambda_ori << std::endl;
      std::cout << "Inv: " << Lambda_ori_inv << std::endl;
      std::cout << " --- " << std::endl;

      epsilon = 0.00025;
      std::cout << "Epsilon: " << epsilon << std::endl;
      control_utils::PInverse(Lambda_ori_inv, Lambda_ori, epsilon);
      std::cout << "Lambda: " << Lambda_ori << std::endl;
      std::cout << "Inv: " << Lambda_ori_inv << std::endl;
      std::cout << " --- " << std::endl;

      epsilon = 0.0025;
      std::cout << "Epsilon: " << epsilon << std::endl;
      control_utils::PInverse(Lambda_ori_inv, Lambda_ori, epsilon);
      std::cout << "Lambda: " << Lambda_ori << std::endl;
      std::cout << "Inv: " << Lambda_ori_inv << std::endl;
      std::cout << " --- " << std::endl;

      epsilon = 0.025;
      std::cout << "Epsilon: " << epsilon << std::endl;
      control_utils::PInverse(Lambda_ori_inv, Lambda_ori, epsilon);
      std::cout << "Lambda: " << Lambda_ori << std::endl;
      std::cout << "Inv: " << Lambda_ori_inv << std::endl;
      std::cout << " --- " << std::endl;

      // Compute torque for motion part
      // tau = J^{T}(\Lambda(K_{p}(x^{d} - x) - K_{v}(\dot{x}))) + compensation
      // + nullspace torque

      // tau_d << jacobian.transpose() * (Lambda * (stiffness * error - damping
      // * (jacobian * dq)));
      tau_d << jacobian_pos.transpose() * Lambda_pos *
                       (t_stiffness * pos_error -
                        t_damping * (jacobian_pos * dq)) +
                   jacobian_ori.transpose() * Lambda_ori *
                       (r_stiffness * ori_error -
                        r_damping * (jacobian_ori * dq));

      // std::cout << Lambda << std::endl;
      // std::cout << Lambda_pos << std::endl;
      // std::cout << Lambda_ori << std::endl;
      // std::cout << (r_stiffness * ori_error - r_damping * (jacobian_ori *
      // dq)) << std::endl;
      //  Add additional torques in the nullspace
      franka::Torques zero_torques{
          {0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001}};

      return zero_torques;
    };

    robot.control(osc_control_callback);

  } catch (franka::Exception const &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
