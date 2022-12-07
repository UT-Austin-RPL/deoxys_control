// Copyright 2022 Yifeng Zhu

#include <atomic>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <yaml-cpp/yaml.h>

#include <spdlog/spdlog.h>

#include "utils/log_utils.h"
#include "utils/robot_utils.h"
#include "utils/zmq_utils.h"

#include "franka_controller.pb.h"
#include "franka_robot_state.pb.h"

int main(int argc, char **argv) {

  // Load cofigs
  YAML::Node config = YAML::LoadFile(argv[1]);

  double pub_rate = 40.;
  if (config["GRIPPER"]["PUB_RATE"]) {
    pub_rate = config["GRIPPER"]["PUB_RATE"].as<double>();
  }

  const std::string robot_ip = config["ROBOT"]["IP"].as<std::string>();

  // Subscribing gripper command
  const std::string subscriber_ip = config["PC"]["IP"].as<std::string>();
  const std::string sub_port =
      config["NUC"]["GRIPPER_SUB_PORT"].as<std::string>();

  // Publishing gripper command
  const std::string pub_port =
      config["NUC"]["GRIPPER_PUB_PORT"].as<std::string>();

  zmq_utils::ZMQPublisher zmq_pub(pub_port);
  zmq_utils::ZMQSubscriber zmq_sub(subscriber_ip, sub_port);

  // Initialize robot
  log_utils::initialize_logger(
      config["GRIPPER_LOGGER"]["CONSOLE"]["LOGGER_NAME"].as<std::string>(),
      config["GRIPPER_LOGGER"]["CONSOLE"]["LEVEL"].as<std::string>(),
      config["GRIPPER_LOGGER"]["CONSOLE"]["USE"].as<bool>(),
      config["GRIPPER_LOGGER"]["FILE"]["LOGGER_NAME"].as<std::string>(),
      config["GRIPPER_LOGGER"]["FILE"]["LEVEL"].as<std::string>(),
      config["GRIPPER_LOGGER"]["FILE"]["USE"].as<bool>());

  try {
    franka::Gripper gripper(robot_ip);

    std::atomic_bool running{true};
    std::atomic_bool executing{false};

    robot_utils::FrankaGripperStateUtils gripper_state_utils;
    struct {
      std::mutex mutex;
      franka::GripperState state;
    } gripper_state{};

    struct {
      std::mutex mutex;
      FrankaGripperControlMessage control_msg;
    } gripper_cmd{};

    auto gripper_logger = log_utils::get_logger(
        config["GRIPPER_LOGGER"]["CONSOLE"]["LOGGER_NAME"].as<std::string>());

    // Log information about current gripper control frequency
    gripper_logger->info("Gripper state publisher: {0}Hz", pub_rate);
    // Initialize gripper subscribing / publishing thread,
    std::thread gripper_pub_thread([&]() {
      franka::GripperState current_gripper_state;
      while (running) {
        if (gripper_state.mutex.try_lock()) {
          gripper_state.state = gripper.readOnce();
          current_gripper_state = gripper_state.state;
          gripper_state.mutex.unlock();
        }

        FrankaGripperStateMessage gripper_state_msg;
        gripper_state_utils.LoadGripperStateToMsg(current_gripper_state,
                                                  gripper_state_msg);
        std::string serialized_gripper_state_msg;
        gripper_state_msg.SerializeToString(&serialized_gripper_state_msg);
        zmq_pub.send(serialized_gripper_state_msg);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(1. / pub_rate * 1000)));
      }
    });

    // A gripper command subscribing thread, and send gripper stop command if
    // received the stop command
    std::thread gripper_sub_thread([&]() {
      bool new_control_msg = false;
      while (running) {
        std::string msg;
        msg = zmq_sub.recv(false);
        FrankaGripperControlMessage control_msg;
        if (control_msg.ParseFromString(msg)) {
          new_control_msg = true;
          if (gripper_cmd.mutex.try_lock()) {
            gripper_cmd.control_msg = control_msg;
            gripper_cmd.mutex.unlock();
          }
        }
        auto gripper_control = control_msg.control_msg();
        FrankaGripperStopMessage stop_msg;
        if (gripper_control.UnpackTo(&stop_msg)) {
          gripper.stop();
        }

        // Decide if terminate or not
        if (control_msg.termination()) {
          running = false;
        } else {
          executing = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });

    gripper.homing();
    gripper_logger->info("Gripper homing complete");
    bool has_grasped = false;
    // Main loop
    while (running) {
      if (executing) {
        FrankaGripperStopMessage homing_msg;
        FrankaGripperMoveMessage move_msg;
        FrankaGripperGraspMessage grasp_msg;
        FrankaGripperStopMessage stop_msg;

        FrankaGripperControlMessage last_control_msg;
        if (gripper_cmd.mutex.try_lock()) {
          last_control_msg = gripper_cmd.control_msg;
          gripper_cmd.mutex.unlock();
        } else {
          continue;
        }

        auto gripper_control = last_control_msg.control_msg();
        if (gripper_control.UnpackTo(&homing_msg)) {
          gripper.homing();
          has_grasped = false;
        } else if (gripper_control.UnpackTo(&move_msg)) {
          gripper.move(move_msg.width(), move_msg.speed());
          has_grasped = false;
        } else if (gripper_control.UnpackTo(&grasp_msg)) {
          if (has_grasped) {
            continue;
          }
          double epsilon_inner, epsilon_outer;
          if (grasp_msg.epsilon_inner() == 0. &&
              grasp_msg.epsilon_outer() == 0.) {
            // if not defined, we will keep epsilon high so that it won't get
            // stuck
            epsilon_inner = 0.08;
            epsilon_outer = 0.08;
          } else {
            epsilon_inner = grasp_msg.epsilon_inner();
            epsilon_outer = grasp_msg.epsilon_outer();
          }

          double force;
          if (grasp_msg.force() == 0.) {
            force = 2.0;
          } else {
            force = grasp_msg.force();
          }
          has_grasped = gripper.grasp(grasp_msg.width(), grasp_msg.speed(),
                                      force, epsilon_inner, epsilon_outer);

          gripper_logger->info("Grasped? {0}", has_grasped);
        } else if (gripper_control.UnpackTo(&stop_msg)) {
          gripper.stop();
          has_grasped = false;
        } else {
          gripper_logger->warn("Unpack failed");
        }
        executing = false;
      }
    }
    gripper_sub_thread.join();
    gripper_pub_thread.join();

  } catch (franka::Exception const &e) {
    auto gripper_logger = log_utils::get_logger(
        config["GRIPPER_LOGGER"]["CONSOLE"]["LOGGER_NAME"].as<std::string>());
    gripper_logger->error(e.what());
    return -1;
  }

  return 0;
}
