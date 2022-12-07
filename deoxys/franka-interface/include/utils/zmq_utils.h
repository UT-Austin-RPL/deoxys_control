// Copyright 2022 Yifeng Zhu

#include <string>
#include <zmqpp/zmqpp.hpp>

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_ZMQ_UTILS_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_ZMQ_UTILS_H_

namespace zmq_utils {
class ZMQPublisher {
private:
  zmqpp::context context_;
  zmqpp::socket publish_socket_;
  std::string pub_host_;

public:
  ZMQPublisher(std::string port);
  ~ZMQPublisher();
  void send(std::string msg);
};

class ZMQSubscriber {
private:
  zmqpp::context context_;
  zmqpp::socket subscriber_socket_;
  std::string sub_host_;

public:
  ZMQSubscriber(std::string ip, std::string port, std::string filter = "");
  ~ZMQSubscriber();
  std::string recv(bool noblock = true);
};

} // namespace zmq_utils
#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_ZMQ_UTILS_H_
