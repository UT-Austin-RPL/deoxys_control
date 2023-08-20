// Copyright 2022 Yifeng Zhu

#include <atomic>
#include <memory>
#include <spdlog/spdlog.h>

#include "controllers/base_controller.h"
#include "utils/control_utils.h"
#include "utils/traj_interpolators/base_traj_interpolator.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_SHARED_MEMORY_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_SHARED_MEMORY_H_

struct SharedMemory {
  std::atomic_bool running{true};      // controlling control callback
  std::atomic_bool termination{false}; // controlling main loop
  double time = 0.0;

  std::shared_ptr<controller::BaseController> controller_ptr;
  std::shared_ptr<traj_utils::BaseTrajInterpolator> traj_interpolator_ptr;
  std::shared_ptr<spdlog::logger> logger;

  // for torque control
  std::atomic<double> max_torque;
  std::atomic<double> min_torque;

  // for velocity control
  std::atomic<double> max_trans_speed;
  std::atomic<double> min_trans_speed;
  std::atomic<double> max_rot_speed;
  std::atomic<double> min_rot_speed;

  // for trajectory interpolation
  double traj_interpolator_time_fraction = 1.0;

  std::atomic_int no_msg_counter;
  std::atomic_bool start{
      false}; // indicate if a message is received and start controlling
};

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_SHARED_MEMORY_H_
