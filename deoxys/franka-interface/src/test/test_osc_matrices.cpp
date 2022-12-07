#include "utils/control_utils.h"
#include "utils/traj_interpolators/base_traj_interpolator.h"
#include "utils/traj_interpolators/pose_traj_interpolator.h"
#include "utils/traj_interpolators/position_traj_interpolator.h"
#include <Eigen/Dense>
#include <iostream>

#include <chrono>
#include <cmath>
#include <thread>

using namespace traj_utils;

void AxisAngle(const Eigen::Vector3d axis, Eigen::AngleAxisd &axis_angle) {
  double angle = axis.norm();
  axis_angle = Eigen::AngleAxisd(angle, axis.normalized());
}

void OrientationError(const Eigen::Matrix3d &desired_ori,
                      const Eigen::Matrix3d &current_ori, Eigen::Vector3d &w) {
  w = 0.5 * (current_ori.col(0).cross(desired_ori.col(0)) +
             current_ori.col(1).cross(desired_ori.col(1)) +
             current_ori.col(2).cross(desired_ori.col(2)));
}

void PrintQuaternion(const Eigen::Quaterniond &q) {
  std::cout << q.w() << ", " << q.vec().transpose() << std::endl;
}

int main() {

  // std::chrono::high_resolution_clock::time_point t1 =
  // std::chrono::high_resolution_clock::now();

  Eigen::AngleAxisd axis_angle;
  AxisAngle(Eigen::Vector3d(0., 0., 0.0), axis_angle);

  Eigen::Matrix3d current_ori = axis_angle.toRotationMatrix();

  // std::cout << current_ori << std::endl;
  Eigen::Matrix3d desired_ori;
  desired_ori << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  // std::cout << current_ori.col(0).cross(desired_ori.col(0)) << std::endl;
  // std::cout << current_ori.col(1).cross(desired_ori.col(1)) << std::endl;
  // std::cout << current_ori.col(2).cross(desired_ori.col(2)) << std::endl;

  Eigen::Vector3d w;
  OrientationError(desired_ori, current_ori, w);
  double angle = w.norm();
  w.normalize();

  std::cout << angle / 2 * w.transpose() << std::endl;
  Eigen::Quaterniond current_quat(current_ori);
  Eigen::Quaterniond desired_quat(desired_ori);

  Eigen::Quaterniond quat_error(desired_ori.inverse() * current_ori);

  Eigen::Vector3d error_tail;
  error_tail << quat_error.x(), quat_error.y(), quat_error.z();
  error_tail << -current_ori * error_tail;

  std::cout << error_tail << std::endl;

  // std::cout << w.transpose() << std::endl;

  // Axis-angle (angle, w)
  // std::cout << angle << " : " << w.transpose() << std::endl;
  Eigen::Matrix3d W;
  W << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

  // Quaternion slerp example
  std::cout << "Quaternion slerp: " << std::endl;
  Eigen::Quaterniond q1(1., 0., 0., 0.);
  Eigen::Quaterniond q2(0., 1., 0., 0.);

  PrintQuaternion(q2.slerp(0.0, q1));
  PrintQuaternion(q2.slerp(0.5, q1));
  PrintQuaternion(q2.slerp(1.0, q1));

  // Interpolation
  Eigen::Vector3d p_start(0.0, 0.0, 0.0);
  Eigen::AngleAxisd ori_start(-0.5, Eigen::Vector3d(1.0, 0., 0));
  Eigen::Quaterniond q_start(ori_start);

  Eigen::Vector3d p_goal(0.5, 0.3, 0.0);
  Eigen::AngleAxisd ori_goal(0.5, Eigen::Vector3d(1.0, 0., 0));
  Eigen::Quaterniond q_goal(ori_goal);

  std::cout << "Simple position trajectory interpolator: " << std::endl;
  BaseTrajInterpolator *traj_ptr;
  LinearPositionTrajInterpolator simple_position_interpolator;
  traj_ptr = &simple_position_interpolator;

  std::chrono::high_resolution_clock::time_point t0 =
      std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();

  double time;
  time = static_cast<double>(
             (std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0))
                 .count()) /
         1000.;
  simple_position_interpolator.Reset(time, p_start, p_goal, 20, 500);
  Eigen::Vector3d p_t;

  for (int i = 0; i < 50; i++) {
    t1 = std::chrono::high_resolution_clock::now();
    time = static_cast<double>(
               (std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0))
                   .count()) /
           1000.;
    simple_position_interpolator.GetNextStep(time, p_t);
    std::cout << i << " : " << p_t.transpose() << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }

  LinearPoseTrajInterpolator simple_pose_interpolator;
  traj_ptr = &simple_pose_interpolator;

  Eigen::Quaterniond q_t;

  PrintQuaternion(q_start);
  PrintQuaternion(q_goal);

  t1 = std::chrono::high_resolution_clock::now();
  time = static_cast<double>(
             (std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0))
                 .count()) /
         1000.;

  std::chrono::high_resolution_clock::time_point start_t =
      std::chrono::high_resolution_clock::now();
  simple_pose_interpolator.Reset(time, p_start, q_start, p_goal, q_goal, 20,
                                 500);
  std::chrono::high_resolution_clock::time_point end_t =
      std::chrono::high_resolution_clock::now();
  std::cout << "Reset Time: "
            << std::chrono::duration_cast<std::chrono::nanoseconds>(end_t -
                                                                    start_t)
                   .count()
            << " nano secs" << std::endl;
  ;

  for (int i = 0; i < 60; i++) {
    t1 = std::chrono::high_resolution_clock::now();
    time = static_cast<double>(
               (std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0))
                   .count()) /
           1000.;

    simple_pose_interpolator.GetNextStep(time, p_t, q_t);
    std::cout << i << " : " << p_t.transpose() << std::endl;
    PrintQuaternion(q_t);
    std::cout << q_t.dot(q_goal) << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }

  bool running = true;
  std::thread mock_thread([&]() {
    while (running) {
      p_start = p_goal;
      p_goal = p_start + Eigen::Vector3d(0.0, 0., 0.);
      q_start = q_goal;
      q_goal = Eigen::Quaterniond(0.707, 0.707, 0., 0.) * q_start;
      simple_pose_interpolator.Reset(0., p_start, q_start, p_goal, q_goal, 20,
                                     500);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  for (int i = 0; i < 1000; i++) {
    t1 = std::chrono::high_resolution_clock::now();
    time = static_cast<double>(
               (std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0))
                   .count()) /
           1000.;

    simple_pose_interpolator.GetNextStep(time, p_t, q_t);
    std::cout << p_t.transpose() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  running = false;
  mock_thread.join();

  // Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3) + std::sin(angle) * W +
  // (1 - std::cos(angle)) * W * W; Eigen::MatrixXd A = desired_ori *
  // current_ori.transpose(); std::chrono::high_resolution_clock::time_point t2
  // = std::chrono::high_resolution_clock::now(); auto time =
  // std::chrono::duration_cast<std::chrono::microseconds> (t2 - t1); std::cout
  // << A * current_ori << std::endl; std::cout << time.count() << std::endl;
}
