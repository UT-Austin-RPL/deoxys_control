// Copyright 2022 Yifeng Zhu

#include <atomic>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <yaml-cpp/yaml.h>

#include <spdlog/spdlog.h>

#include "utils/common_utils.h"
#include "utils/control_utils.h"
#include "utils/log_utils.h"
#include "utils/robot_utils.h"
#include "utils/shared_memory.h"
#include "utils/zmq_utils.h"

// Callback
#include "utils/control_callbacks/joint_pos_callback.h"
#include "utils/control_callbacks/torque_callback.h"
#include "utils/control_callbacks/cartesian_velocity_calllback.h"

// Interpolators
#include "utils/traj_interpolators/linear_joint_position_traj_interpolator.h"
#include "utils/traj_interpolators/linear_pose_traj_interpolator.h"
#include "utils/traj_interpolators/linear_position_traj_interpolator.h"
#include "utils/traj_interpolators/min_jerk_joint_position_traj_interpolator.h"
#include "utils/traj_interpolators/min_jerk_pose_traj_interpolator.h"
#include "utils/traj_interpolators/smooth_joint_traj_interpolator.h"
#include "utils/traj_interpolators/cosine_cartesian_velocity_traj_interpolator.h"
#include "utils/traj_interpolators/linear_cartesian_velocity_traj_interpolator.h"

// State estimators
#include "utils/state_estimators/exponential_smoothing_estimator.h"

// Controllers
#include "controllers/joint_impedance.h"
#include "controllers/joint_position.h"
#include "controllers/osc_impedance.h"
#include "controllers/osc_position_impedance.h"
#include "controllers/osc_yaw_impedance.h"
#include "controllers/cartesian_velocity.h"

#include "franka_controller.pb.h"
#include "franka_robot_state.pb.h"

enum ControllerType {
  NO_CONTROL,
  OSC_POSE,
  OSC_POSITION,
  JOINT_POSITION,
  JOINT_IMPEDANCE,
  JOINT_VELOCITY,
  TORQUE,
  OSC_YAW,
  CARTESIAN_VELOCITY,
};

enum TrajInterpolatorType {
  NO_INTERPOLATION,
  LINEAR_POSITION,
  LINEAR_POSE,
  MIN_JERK_POSE,
  SMOOTH_JOINT_POSITION,
  MIN_JERK_JOINT_POSITION,
  LINEAR_JOINT_POSITION,
  COSINE_CARTESIAN_VELOCITY,
  LINEAR_CARTESIAN_VELOCITY,
};

enum StateEstimatorType {
  NO_ESTIMATOR,
  EXPONENTIAL_SMOOTHING_ESTIMATOR,
};

bool GetControllerType(const FrankaControlMessage &franka_control_msg,
                       ControllerType &controller_type) {
  if (franka_control_msg.controller_type() ==
      FrankaControlMessage_ControllerType_OSC_POSE) {
    controller_type = ControllerType::OSC_POSE;
  } else if (franka_control_msg.controller_type() ==
             FrankaControlMessage_ControllerType_OSC_POSITION) {
    controller_type = ControllerType::OSC_POSITION;
  } else if (franka_control_msg.controller_type() ==
             FrankaControlMessage_ControllerType_OSC_YAW) {
    controller_type = ControllerType::OSC_YAW;
  } else if (franka_control_msg.controller_type() ==
             FrankaControlMessage_ControllerType_JOINT_POSITION) {
    controller_type = ControllerType::JOINT_POSITION;
  } else if (franka_control_msg.controller_type() ==
             FrankaControlMessage_ControllerType_JOINT_IMPEDANCE) {
    controller_type = ControllerType::JOINT_IMPEDANCE;
  } else if (franka_control_msg.controller_type() ==
             FrankaControlMessage_ControllerType_JOINT_VELOCITY) {
    controller_type = ControllerType::JOINT_VELOCITY;
  } else if (franka_control_msg.controller_type() ==
             FrankaControlMessage_ControllerType_TORQUE) {
    controller_type = ControllerType::TORQUE;
  } else if (franka_control_msg.controller_type() ==           
             FrankaControlMessage_ControllerType_CARTESIAN_VELOCITY) {
    controller_type = ControllerType::CARTESIAN_VELOCITY;
  } else if (franka_control_msg.controller_type() ==
             FrankaControlMessage_ControllerType_NO_CONTROL) {
    controller_type = ControllerType::NO_CONTROL;
  } else {
    return false;
  }
  return true;
}

bool GetTrajInterpolatorType(const FrankaControlMessage &franka_control_msg,
                             TrajInterpolatorType &traj_interpolator_type) {
  if (franka_control_msg.traj_interpolator_type() ==
      FrankaControlMessage_TrajInterpolatorType_LINEAR_POSITION) {
    traj_interpolator_type = TrajInterpolatorType::LINEAR_POSITION;
  } else if (franka_control_msg.traj_interpolator_type() ==
             FrankaControlMessage_TrajInterpolatorType_LINEAR_POSE) {
    traj_interpolator_type = TrajInterpolatorType::LINEAR_POSE;
  } else if (franka_control_msg.traj_interpolator_type() ==
             FrankaControlMessage_TrajInterpolatorType_MIN_JERK_POSE) {
    traj_interpolator_type = TrajInterpolatorType::MIN_JERK_POSE;
  } else if (franka_control_msg.traj_interpolator_type() ==
             FrankaControlMessage_TrajInterpolatorType_SMOOTH_JOINT_POSITION) {
    traj_interpolator_type = TrajInterpolatorType::SMOOTH_JOINT_POSITION;
  } else if (
      franka_control_msg.traj_interpolator_type() ==
      FrankaControlMessage_TrajInterpolatorType_MIN_JERK_JOINT_POSITION) {
    traj_interpolator_type = TrajInterpolatorType::MIN_JERK_JOINT_POSITION;
  } else if (franka_control_msg.traj_interpolator_type() ==
             FrankaControlMessage_TrajInterpolatorType_LINEAR_JOINT_POSITION) {
    traj_interpolator_type = TrajInterpolatorType::LINEAR_JOINT_POSITION;
  } else if (franka_control_msg.traj_interpolator_type() == FrankaControlMessage_TrajInterpolatorType_COSINE_CARTESIAN_VELOCITY) {
    traj_interpolator_type = TrajInterpolatorType::COSINE_CARTESIAN_VELOCITY;
  } else if (franka_control_msg.traj_interpolator_type() == FrankaControlMessage_TrajInterpolatorType_LINEAR_CARTESIAN_VELOCITY) {
    traj_interpolator_type = TrajInterpolatorType::LINEAR_CARTESIAN_VELOCITY;
  }
  else {
    traj_interpolator_type = TrajInterpolatorType::NO_INTERPOLATION;
    return false;
  }
  return true;
}

bool GetStateEstimatorType(const FrankaControlMessage franka_control_msg,
                           StateEstimatorType &state_estimator_type) {
  if (franka_control_msg.state_estimator_msg().estimator_type() ==
      FrankaStateEstimatorMessage_EstimatorType_EXPONENTIAL_SMOOTHING_ESTIMATOR) {
    state_estimator_type = StateEstimatorType::EXPONENTIAL_SMOOTHING_ESTIMATOR;
  } else {
    state_estimator_type = StateEstimatorType::NO_ESTIMATOR;
    return false;
  }
  return true;
}

int main(int argc, char **argv) {
  // Load cofigs
  if (argc < 2) {
    spdlog::error("It seems that you forgot to specify a yaml config file");
    return 0;
  }

  YAML::Node config = YAML::LoadFile(argv[1]);

  // Initialize zmq sub / pub
  const std::string robot_ip = config["ROBOT"]["IP"].as<std::string>();

  // Subscribing control command
  const std::string subscriber_ip = config["PC"]["IP"].as<std::string>();
  const std::string sub_port = config["NUC"]["SUB_PORT"].as<std::string>();

  // Publishing control command
  const std::string pub_port = config["NUC"]["PUB_PORT"].as<std::string>();

  // zmq_utils::ZMQPublisher zmq_pub(pub_port);
  zmq_utils::ZMQSubscriber zmq_sub(subscriber_ip, sub_port);

  YAML::Node control_config;
  if (argc > 2) {
    control_config = YAML::LoadFile(argv[2]);
  } else {
    control_config = YAML::LoadFile("config/control_config.yml");
  }

  int loaded_state_pub_rate, loaded_policy_rate, loaded_traj_rate;
  bool zmq_noblock;

  if (config["CONTROL"]["STATE_PUBLISHER_RATE"]) {
    loaded_state_pub_rate = config["CONTROL"]["STATE_PUBLISHER_RATE"].as<int>();
  } else {
    loaded_state_pub_rate = 100;
  }
  if (config["CONTROL"]["POLICY_RATE"]) {
    loaded_policy_rate = config["CONTROL"]["POLICY_RATE"].as<int>();
  } else {
    loaded_policy_rate = 20;
  }
  if (config["CONTROL"]["TRAJ_RATE"]) {
    loaded_traj_rate = config["CONTROL"]["TRAJ_RATE"].as<int>();
  } else {
    loaded_traj_rate = 500;
  }
  if (config["CONTROL"]["ZMQ_NOBLOCK"]) {
    zmq_noblock = config["CONTROL"]["ZMQ_NOBLOCK"].as<bool>();
  } else {
    zmq_noblock = true;
  }
  const int state_pub_rate = loaded_state_pub_rate;
  const int policy_rate = loaded_policy_rate;
  const int traj_rate = loaded_traj_rate;

  // Initialize robot
  log_utils::initialize_logger(
      config["ARM_LOGGER"]["CONSOLE"]["LOGGER_NAME"].as<std::string>(),
      config["ARM_LOGGER"]["CONSOLE"]["LEVEL"].as<std::string>(),
      config["ARM_LOGGER"]["CONSOLE"]["USE"].as<bool>(),
      config["ARM_LOGGER"]["FILE"]["LOGGER_NAME"].as<std::string>(),
      config["ARM_LOGGER"]["FILE"]["LEVEL"].as<std::string>(),
      config["ARM_LOGGER"]["FILE"]["USE"].as<bool>());

  try {

    franka::Robot robot(robot_ip);
    robot.automaticErrorRecovery();
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();

    // TODO(Yifeng): Read this config from yaml file
    robot.setCollisionBehavior(
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    // Get initial state
    franka::RobotState init_state = robot.readOnce();
    Eigen::Affine3d init_T_EE_in_base_frame(
        Eigen::Matrix4d::Map(init_state.O_T_EE.data()));

    std::shared_ptr<SharedMemory> global_handler =
        std::make_shared<SharedMemory>();
    global_handler->logger = log_utils::get_logger(
        config["ARM_LOGGER"]["CONSOLE"]["LOGGER_NAME"].as<std::string>());
    
    // Read torque limits from the global config
    global_handler->max_torque =
        control_config["CONTROL"]["SAFETY"]["MAX_TORQUE"].as<double>();
    global_handler->min_torque =
        control_config["CONTROL"]["SAFETY"]["MIN_TORQUE"].as<double>();

    // Read speed limits from the global config
    global_handler->max_trans_speed =
        control_config["CONTROL"]["SAFETY"]["MAX_TRANS_SPEED"].as<double>();
    global_handler->min_trans_speed =
        control_config["CONTROL"]["SAFETY"]["MIN_TRANS_SPEED"].as<double>();
    global_handler->max_rot_speed =
        control_config["CONTROL"]["SAFETY"]["MAX_ROT_SPEED"].as<double>();
    global_handler->min_rot_speed =
        control_config["CONTROL"]["SAFETY"]["MIN_ROT_SPEED"].as<double>();

    std::shared_ptr<StateInfo> current_state_info =
        std::make_shared<StateInfo>();
    std::shared_ptr<StateInfo> goal_state_info = std::make_shared<StateInfo>();

    current_state_info->pos_EE_in_base_frame
        << init_T_EE_in_base_frame.translation();
    current_state_info->quat_EE_in_base_frame =
        Eigen::Quaterniond(init_T_EE_in_base_frame.linear());
    goal_state_info->pos_EE_in_base_frame =
        current_state_info->pos_EE_in_base_frame;
    goal_state_info->quat_EE_in_base_frame =
        current_state_info->quat_EE_in_base_frame;
    current_state_info->joint_positions =
        Eigen::VectorXd::Map(init_state.q_d.data(), 7);
    goal_state_info->joint_positions = current_state_info->joint_positions;

    // Log information about current arm control frequency
    global_handler->logger->info(
        "State Publisher: {0}Hz, Policy: {1}Hz, Traj Interpolation {2}Hz, ZMQ "
        "noblock receving {3}",
        state_pub_rate, policy_rate, traj_rate, zmq_noblock);
    if (!zmq_noblock) {
      global_handler->logger->warn(
          "ZMQ communication is blocking, it could lead to severe networking "
          "issue. Recommend to set true.");
    }

    struct {
      std::mutex mutex;
      FrankaControlMessage control_msg;
      ControllerType controller_type = ControllerType::NO_CONTROL;
      TrajInterpolatorType traj_interpolator_type =
          TrajInterpolatorType::NO_INTERPOLATION;
      StateEstimatorType state_estimator_type =
          StateEstimatorType::NO_ESTIMATOR;
      double timeout = -1.0; // No timeout if negative
    } control_command{};

    struct {
      std::mutex mutex;
      franka::RobotState state;
    } state_sub{};

    std::shared_ptr<robot_utils::StatePublisher> state_publisher =
        std::make_shared<robot_utils::StatePublisher>(pub_port, state_pub_rate);
    state_publisher->StartPublishing();
    state_publisher->UpdateNewState(init_state, &model);

    ControllerType controller_type = ControllerType::NO_CONTROL;
    TrajInterpolatorType traj_interpolator_type =
        TrajInterpolatorType::NO_INTERPOLATION;
    StateEstimatorType state_estimator_type = StateEstimatorType::NO_ESTIMATOR;

    // control message subscription thread
    std::thread control_msg_sub([&]() {
      while (!global_handler->termination) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(int(1. / policy_rate * 1000.)));

        // Receive message
        std::string msg;
        msg = zmq_sub.recv(zmq_noblock);
        if (msg.length() == 0) {
          global_handler->no_msg_counter += int(global_handler->start);
          global_handler->logger->debug("Counter {0}",
                                        global_handler->no_msg_counter);
          if (global_handler->no_msg_counter >= 20) {
            global_handler->running = false;
            global_handler->termination = true;
            global_handler->logger->debug(
                "No valid messages received in 20 steps");
          }
          continue;
        }

        FrankaControlMessage control_msg;

        if (control_msg.ParseFromString(msg)) {
          global_handler->no_msg_counter = 0;
          if (control_msg.termination()) {
            global_handler->running = false;
            global_handler->termination = true;
          }
          // Determine controller message type
          if (control_command.mutex.try_lock()) {
            if (!GetControllerType(control_msg,
                                   control_command.controller_type)) {
              global_handler->logger->warn("No controller type is specified");
            };
            if (!GetTrajInterpolatorType(
                    control_msg, control_command.traj_interpolator_type)) {
              global_handler->logger->debug(
                  "No traj interpolator is specified");
            };
            if (!GetStateEstimatorType(control_msg,
                                       control_command.state_estimator_type)) {
              global_handler->logger->debug("No state estimator is specified");
            }
            if (control_msg.traj_interpolator_time_fraction() > 0.10) {
              global_handler->traj_interpolator_time_fraction =
                  control_msg.traj_interpolator_time_fraction();
            } else {
              // if the time fraction is set to less than 1/10, then we consider
              // it to be too dangerous
              global_handler->traj_interpolator_time_fraction = 0.1;
            }
            control_command.timeout = control_msg.timeout();
            control_command.control_msg = control_msg;
            control_command.mutex.unlock();
          }

          if (control_command.controller_type == ControllerType::OSC_POSE &&
              controller_type == ControllerType::NO_CONTROL) {
            global_handler->controller_ptr =
                std::make_shared<controller::OSCImpedanceController>(model);
            global_handler->logger->info("Initialize OSC Pose");
            global_handler->running = true;
          } else if (control_command.controller_type ==
                         ControllerType::OSC_POSITION &&
                     controller_type == ControllerType::NO_CONTROL) {
            global_handler->controller_ptr =
                std::make_shared<controller::OSCPositionImpedanceController>(
                    model);
            global_handler->logger->info("Initialize OSC Position");
            global_handler->running = true;
          } else if (control_command.controller_type ==
                         ControllerType::OSC_YAW &&
                     controller_type == ControllerType::NO_CONTROL) {
            global_handler->controller_ptr =
                std::make_shared<controller::OSCYawImpedanceController>(model);
            global_handler->logger->info("Initialize OSC Yaw");
            global_handler->running = true;
          } else if (control_command.controller_type ==
                         ControllerType::JOINT_POSITION &&
                     controller_type == ControllerType::NO_CONTROL) {
            global_handler->controller_ptr =
                std::make_shared<controller::JointPositionController>(model);
            global_handler->logger->info("Initialize Joint Position");
            global_handler->running = true;
          } else if (control_command.controller_type ==
                         ControllerType::JOINT_IMPEDANCE &&
                     controller_type == ControllerType::NO_CONTROL) {
            global_handler->controller_ptr =
                std::make_shared<controller::JointImpedanceController>(model);
            global_handler->logger->info("Initialize Joint Impedance");
            global_handler->running = true;
          } else if (control_command.controller_type ==
                         ControllerType::CARTESIAN_VELOCITY &&
                     controller_type == ControllerType::NO_CONTROL) {
            global_handler->controller_ptr =
                std::make_shared<controller::CartesianVelocityController>(model);
            global_handler->logger->info("Initialize Cartesian Velocity");
            global_handler->running = true;            
          } else if (control_command.controller_type ==
                         ControllerType::NO_CONTROL ||
                     controller_type == ControllerType::NO_CONTROL) {
            global_handler->running = false;
            continue;
          }

          if (control_command.controller_type != ControllerType::NO_CONTROL &&
              controller_type == ControllerType::NO_CONTROL) {
            if (control_command.state_estimator_type ==
                StateEstimatorType::EXPONENTIAL_SMOOTHING_ESTIMATOR) {
              global_handler->controller_ptr->SetStateEstimator(
                  std::make_shared<
                      estimator_utils::ExponentialSmoothingEstimator>());
              global_handler->logger->info("Initialize State Estimator");
            }
            state_estimator_type = control_command.state_estimator_type;
          }

          global_handler->start = true;
          if (traj_interpolator_type !=
              control_command.traj_interpolator_type) {
            if (control_command.traj_interpolator_type ==
                TrajInterpolatorType::LINEAR_POSE) {
              global_handler->traj_interpolator_ptr =
                  std::make_shared<traj_utils::LinearPoseTrajInterpolator>();
              global_handler->logger->info("Initialize Pose interpolator!");
            } else if (control_command.traj_interpolator_type ==
                       TrajInterpolatorType::LINEAR_POSITION) {
              global_handler->traj_interpolator_ptr = std::make_shared<
                  traj_utils::LinearPositionTrajInterpolator>();
              global_handler->logger->info("Initialize Position interpolator!");
            } else if (control_command.traj_interpolator_type ==
                       TrajInterpolatorType::MIN_JERK_POSE) {
              global_handler->traj_interpolator_ptr =
                  std::make_shared<traj_utils::MinJerkPoseTrajInterpolator>();
              global_handler->logger->info(
                  "Initialize Min Jerk Pose interpolator!");
            } else if (control_command.traj_interpolator_type ==
                       TrajInterpolatorType::SMOOTH_JOINT_POSITION) {
              global_handler->traj_interpolator_ptr =
                  std::make_shared<traj_utils::SmoothJointTrajInterpolator>();
              global_handler->logger->info(
                  "Initialize Smooth Joint Trajectory Interpolator");
            } else if (control_command.traj_interpolator_type ==
                       TrajInterpolatorType::MIN_JERK_JOINT_POSITION) {
              global_handler->traj_interpolator_ptr = std::make_shared<
                  traj_utils::MinJerkJointPositionTrajInterpolator>();
              global_handler->logger->info(
                  "Initialize Min Jerk Joint Position Trajectory Interpolator");
            } else if (control_command.traj_interpolator_type ==
                       TrajInterpolatorType::LINEAR_JOINT_POSITION) {
              global_handler->traj_interpolator_ptr = std::make_shared<
                  traj_utils::LinearJointPositionTrajInterpolator>();
              global_handler->logger->info(
                  "Initialize Linear Joint Position Trajectory Interpolator");
            } else if (control_command.traj_interpolator_type == TrajInterpolatorType::COSINE_CARTESIAN_VELOCITY) {
              global_handler->traj_interpolator_ptr = std::make_shared<
                  traj_utils::CosineCartesianVelocityTrajInterpolator>();
              global_handler->logger->info(
                  "Initialize Cosine Cartesian Velocity Trajectory Interpolator");
            } else if (control_command.traj_interpolator_type == TrajInterpolatorType::LINEAR_CARTESIAN_VELOCITY) {
              global_handler->traj_interpolator_ptr = std::make_shared<
                  traj_utils::LinearCartesianVelocityTrajInterpolator>();
              global_handler->logger->info(
                  "Initialize Linear Cartesian Velocity Trajectory Interpolator");
            }else {
              global_handler->logger->error("No interpolator is specified");
            }

            traj_interpolator_type = control_command.traj_interpolator_type;
          }

          global_handler->controller_ptr->ParseMessage(control_msg);

          global_handler->controller_ptr->ComputeGoal(current_state_info,
                                                      goal_state_info);
          switch (control_command.traj_interpolator_type) {
          case TrajInterpolatorType::LINEAR_POSE:
          case TrajInterpolatorType::LINEAR_POSITION:
          case TrajInterpolatorType::MIN_JERK_POSE:
            global_handler->traj_interpolator_ptr->Reset(
                global_handler->time, current_state_info->pos_EE_in_base_frame,
                current_state_info->quat_EE_in_base_frame,
                goal_state_info->pos_EE_in_base_frame,
                goal_state_info->quat_EE_in_base_frame, policy_rate, traj_rate,
                global_handler->traj_interpolator_time_fraction);
            break;
          case TrajInterpolatorType::SMOOTH_JOINT_POSITION:
          case TrajInterpolatorType::MIN_JERK_JOINT_POSITION:
          case TrajInterpolatorType::LINEAR_JOINT_POSITION:
            global_handler->traj_interpolator_ptr->Reset(
                global_handler->time, current_state_info->joint_positions,
                goal_state_info->joint_positions, policy_rate, traj_rate,
                global_handler->traj_interpolator_time_fraction);
            break;
          case TrajInterpolatorType::COSINE_CARTESIAN_VELOCITY:
          case TrajInterpolatorType::LINEAR_CARTESIAN_VELOCITY:
            global_handler->traj_interpolator_ptr->Reset(
                global_handler->time, current_state_info->twist_trans_EE_in_base_frame,
                current_state_info->twist_rot_EE_in_base_frame,
                goal_state_info->twist_trans_EE_in_base_frame,
                goal_state_info->twist_rot_EE_in_base_frame, policy_rate, traj_rate,
                global_handler->traj_interpolator_time_fraction);
            break;
          default:
            break;
          }
        } else {
          global_handler->no_msg_counter++;
          global_handler->logger->info("Counter {0}",
                                       global_handler->no_msg_counter);
        }
      }
    });

    // Main loop
    global_handler->logger->info("Deoxys starting");
    while (!global_handler->termination) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      // If controller_type changes, exit robot control loop and reinitialize.
      FrankaControlMessage control_msg;

      if (control_command.mutex.try_lock()) {
        controller_type = control_command.controller_type;
        control_msg = control_command.control_msg;
        control_command.mutex.unlock();
      }

      if (global_handler->running) {
        init_state = robot.readOnce();
        state_publisher->UpdateNewState(init_state, &model);
        if (controller_type == ControllerType::NO_CONTROL)
          continue;
        // Choose which control callback functions
        if (controller_type == ControllerType::OSC_POSE ||
            controller_type == ControllerType::OSC_POSITION ||
            controller_type == ControllerType::OSC_YAW) {
          // OSC control callback
          robot.control(
              control_callbacks::CreateTorqueFromCartesianSpaceCallback(
                  global_handler, state_publisher, model, current_state_info,
                  goal_state_info, policy_rate, traj_rate));
        } else if (controller_type == ControllerType::JOINT_IMPEDANCE) {
          // Joint Impedance control callback
          global_handler->logger->info("Joint impedance callback");
          robot.control(control_callbacks::CreateTorqueFromJointSpaceCallback(
              global_handler, state_publisher, model, current_state_info,
              goal_state_info, policy_rate, traj_rate));
        } else if (controller_type == ControllerType::JOINT_POSITION) {
          // Joint Position control callback
          global_handler->logger->info("Joint position callback");
          robot.control(control_callbacks::CreateJointPositionCallback(
              global_handler, state_publisher, model, current_state_info,
              goal_state_info, policy_rate, traj_rate));
        } else if (controller_type == ControllerType::CARTESIAN_VELOCITY) {
          // Cartesian Velocity control callback
          global_handler->logger->info("Cartesian velocity callback");
          robot.control(control_callbacks::CreateCartesianVelocitiesCallback(
              global_handler, state_publisher, model, current_state_info,
              goal_state_info, policy_rate, traj_rate));
        }
      }
      global_handler->time = 0.0;
    }
    state_publisher->StopPublishing();

    control_msg_sub.join();
  } catch (franka::Exception const &e) {
    auto logger = log_utils::get_logger(
        config["ARM_LOGGER"]["CONSOLE"]["LOGGER_NAME"].as<std::string>());
    logger->error(e.what());
    return -1;
  }
  return 0;
}
