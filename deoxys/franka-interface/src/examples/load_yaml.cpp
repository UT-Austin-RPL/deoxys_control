// Copyright 2022 Yifeng Zhu

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

int main(int argc, char **argv) {
  // Usage example of yaml file
  YAML::Node config = YAML::LoadFile(argv[1]);

  std::string pc_name = config["PC"]["NAME"].as<std::string>();
  std::string pc_ip = config["PC"]["IP"].as<std::string>();
  std::string pc_pub = config["PC"]["PUB_PORT"].as<std::string>();
  std::string pc_sub = config["PC"]["SUB_PORT"].as<std::string>();

  std::cout << "PC: " << std::endl;
  std::cout << "NAME: " << pc_name << std::endl;
  std::cout << "IP: " << pc_ip << std::endl;
  std::cout << "PUB PORT: " << pc_pub << std::endl;
  std::cout << "SUB PORT: " << pc_sub << std::endl;
}
