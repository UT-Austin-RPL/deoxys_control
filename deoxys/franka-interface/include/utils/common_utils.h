// Copyright 2022 Yifeng Zhu

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <array>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#ifndef UTILS_COMMON_UTILS_H_
#define UTILS_COMMON_UTILS_H_

void setDefaultBehavior(franka::Robot &robot);

class MotionGenerator {
public:
  MotionGenerator(double speed_factor, const std::array<double, 7> q_goal);

  franka::JointPositions operator()(const franka::RobotState &robot_state,
                                    franka::Duration period);

private:
  using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
  using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

  bool calculateDesiredValues(double t, Vector7d *delta_q_d) const;
  void calculateSynchronizedValues();

  static constexpr double kDeltaQMotionFinished = 1e-6;
  const Vector7d q_goal_;

  Vector7d q_start_;
  Vector7d delta_q_;

  Vector7d dq_max_sync_;
  Vector7d t_1_sync_;
  Vector7d t_2_sync_;
  Vector7d t_f_sync_;
  Vector7d q_1_;

  double time_ = 0.0;

  Vector7d dq_max_ =
      (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
  Vector7d ddq_max_start_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
  Vector7d ddq_max_goal_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
};

// create an axis-angle from a 3d vector
inline void AxisAngle(const Eigen::Vector3d axis,
                      Eigen::AngleAxisd &axis_angle) {
  double angle = axis.norm();
  axis_angle = Eigen::AngleAxisd(angle, axis.normalized());
}

#endif
