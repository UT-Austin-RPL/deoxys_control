// Copyright 2022 Yifeng Zhu

#include <Eigen/Dense>
#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_SHARED_STATE_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_SHARED_STATE_H_

struct StateInfo {
  Eigen::Vector3d pos_EE_in_base_frame;
  Eigen::Quaterniond quat_EE_in_base_frame;
  Eigen::Matrix<double, 7, 1> joint_positions;
  Eigen::Vector3d trans_vel_EE_in_base_frame; // TODO (Yifeng): not used for
                                              // now. Will update in the future.
  Eigen::Vector3d rot_vel_EE_in_base_frame; // TODO (Yifeng): not used for now.
                                            // Will update in the future.
};
#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_SHARED_STATE_H_
