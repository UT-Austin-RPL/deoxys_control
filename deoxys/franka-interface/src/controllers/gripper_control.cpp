// Copyright 2022 Yifeng Zhu

#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <yaml-cpp/yaml.h>

int main(int argc, char **argv) {

  YAML::Node config = YAML::LoadFile(argv[1]);
  const std::string robot_ip = config["ROBOT"]["IP"].as<std::string>();

  try {
    franka::Gripper gripper(robot_ip);

    double grasping_width = 0.04;
    bool homing = true;

    if (homing) {
      std::cout << "Homging start" << std::endl;
      gripper.homing();
      std::cout << "Homing end" << std::endl;
    }

    franka::GripperState gripper_state = gripper.readOnce();
    if (gripper_state.max_width < grasping_width) {
      return -1;
    }

    if (!gripper.grasp(grasping_width, 0.1, 10, 0.08, 0.08)) {
      std::cout << "Grasping error" << std::endl;
      return -1;
    }

    if (!gripper.move(0.08, 0.1)) {
      std::cout << "Moving failed" << std::endl;
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  } catch (franka::Exception const &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}
