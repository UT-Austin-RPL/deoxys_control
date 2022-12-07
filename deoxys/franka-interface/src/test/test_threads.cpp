#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include "franka_controller.pb.h"
#include "utils/common.h"
#include <yaml-cpp/yaml.h>

int main(int argc, char **argv) {

  // Load yaml configs
  YAML::Node config = YAML::LoadFile(argv[1]);

  // Example of initialize stiffness, damping coefficients
  const double translation_stiffness = 150.0;
  const double rotation_stiffness = 10.0;
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  damping.setZero();
  stiffness.topLeftCorner(3, 3)
      << translation_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3)
      << rotation_stiffness * Eigen::MatrixXd::Identity(3, 3);
  // critical damping example
  stiffness.topLeftCorner(3, 3)
      << 2.0 * sqrt(translation_stiffness) * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3)
      << 2.0 * sqrt(rotation_stiffness) * Eigen::MatrixXd::Identity(3, 3);

  const std::string subscriber_ip = config["PC"]["IP"].as<std::string>();
  const std::string sub_port = config["PC"]["SUB_PORT"].as<std::string>();
  const std::string robot_ip = config["ROBOT"]["IP"].as<std::string>();

  // ZMQSubscriber zmq_sub(subscriber_ip, sub_port);
  std::cout << "Starting the controller" << std::endl;

  double time = 0.;

  Eigen::Vector3d goal;
  goal.setZero();

  struct {
    std::mutex mutex;
    Eigen::Vector3d desired_goal;
  } desired_goal{};

  struct {
    std::mutex mutex;
    Eigen::Vector3d commanded_goal;
  } print_goal{};
  std::atomic_bool running{true};

  std::thread print_thread([&goal, &print_goal, &running]() {
    while (running) {

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (print_goal.mutex.try_lock()) {
        // std::cout << "Current print goal: " << goal.transpose() << std::endl;
        std::cout << print_goal.commanded_goal.transpose() << std::endl;
        print_goal.mutex.unlock();
      }
    }
  });

  std::thread sub_thread([&desired_goal, &running]() {
    int counter = 0;
    while (running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      if (desired_goal.mutex.try_lock()) {
        desired_goal.desired_goal = Eigen::Vector3d(counter, counter, counter);
        counter++;
        // receive desired_goal from zmq
        // std::string msg;
        // // msg = zmq_sub.recv(true);
        // FrankaDummyControllerMessage control_msg;
        // if (control_msg.ParseFromString(msg)) {
        //   desired_goal.desired_goal = Eigen::Vector3d(control_msg.goal().x(),
        //   control_msg.goal().y(), control_msg.goal().z());
        //   // if (control_msg.termination()) {
        //   //   running = false;
        //   // }
        // }
        desired_goal.mutex.unlock();
      }
    }
  });

  std::thread mok_control_thread([&print_goal, &desired_goal, &running]() {
    int count = 0;
    while (running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      print_goal.commanded_goal = desired_goal.desired_goal;
      // if (desired_goal.mutex.try_lock()) {
      // 	if (print_goal.mutex.try_lock()) {

      // 	}
      // 	std::cout << desired_goal.desired_goal.transpose() << std::endl;
      // 	desired_goal.mutex.unlock();
      // }
    }
  });

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}
