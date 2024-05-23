// Copyright 2022 Yifeng Zhu

#include "utils/zmq_utils.h"

namespace zmq_utils {
ZMQPublisher::ZMQPublisher(std::string port)
    : publish_socket_(context_, zmqpp::socket_type::publish) {
  pub_host_ = "tcp://*:" + port;
  publish_socket_.bind(pub_host_);
}

ZMQPublisher::~ZMQPublisher() {
  // publish_socket_.disconnect(pub_host_);
}

void ZMQPublisher::send(std::string msg) { publish_socket_.send(msg, true); }

ZMQSubscriber::ZMQSubscriber(std::string ip, std::string port,
                             std::string filter)
    : subscriber_socket_(context_, zmqpp::socket_type::subscribe) {
  sub_host_ = "tcp://" + ip + ":" + port;
  subscriber_socket_.set(zmqpp::socket_option::conflate, 1);
  subscriber_socket_.subscribe(filter);
  subscriber_socket_.connect(sub_host_);
};

ZMQSubscriber::~ZMQSubscriber() { subscriber_socket_.disconnect(sub_host_); }

std::string ZMQSubscriber::recv(bool noblock) {
  zmqpp::message message;
  subscriber_socket_.receive(message, noblock);
  std::string str;
  if (message.parts() > 0) {
    str = message.get(0);
  } else {
    str = "";
  }
  return str;
}
} // namespace zmq_utils
