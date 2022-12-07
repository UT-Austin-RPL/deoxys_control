// Copyright 2022 Yifeng Zhu

#include <Eigen/Core>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "franka_robot_state.pb.h"
#include "utils/zmq_utils.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_ROBOT_UTILS_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_ROBOT_UTILS_H_

namespace robot_utils {

class FrankaRobotStateUtils {
public:
  FrankaRobotStateUtils(){};
  ~FrankaRobotStateUtils(){};
  void LoadErrorStateToMsg(const franka::Errors &,
                           FrankaRobotStateMessage::Errors &);
  void LoadRobotStateToMsg(const franka::RobotState &,
                           FrankaRobotStateMessage &);
};

class FrankaGripperStateUtils {
public:
  FrankaGripperStateUtils(){};
  ~FrankaGripperStateUtils(){};
  void LoadGripperStateToMsg(const franka::GripperState &,
                             FrankaGripperStateMessage &);
};

enum GripperControlMode { Homing, Move, Grasp, Stop };

class StatePublisher {
protected:
  std::thread state_pub_thread_;
  robot_utils::FrankaRobotStateUtils robot_state_utils_;
  struct {
    franka::RobotState robot_state;
    std::array<double, 144> current_robot_frames;
    std::mutex mutex;
  } state_;
  zmq_utils::ZMQPublisher zmq_publisher_;
  int state_pub_rate_;
  bool running_;

public:
  StatePublisher(std::string pub_port, int state_pub_rate);
  ~StatePublisher();
  void StartPublishing();
  void UpdateNewState(const franka::RobotState &robot_state,
                      const franka::Model *robot_model);
  void StopPublishing();
};

} // namespace robot_utils

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_ROBOT_UTILS_H_
