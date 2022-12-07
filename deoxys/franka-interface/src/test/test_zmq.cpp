#include "franka_robot_state.pb.h"
#include <chrono>
#include <iostream>
#include <string>
#include <zmqpp/zmqpp.hpp>

using namespace std;

class ZMQPublisher {
private:
  zmqpp::context context_;
  zmqpp::socket publish_socket_;
  string pub_host_;

public:
  ZMQPublisher(string port);
  ~ZMQPublisher();
  void send(string msg);
};

ZMQPublisher::ZMQPublisher(string port)
    : publish_socket_(context_, zmqpp::socket_type::publish) {
  pub_host_ = "tcp://*:" + port;
  publish_socket_.bind(pub_host_);
}

ZMQPublisher::~ZMQPublisher() { publish_socket_.disconnect(pub_host_); }

void ZMQPublisher::send(string msg) { publish_socket_.send(msg); }

class ZMQSubscriber {
private:
  zmqpp::context context_;
  zmqpp::socket subscriber_socket_;
  string sub_host_;

public:
  ZMQSubscriber(string ip, string port, string filter = "");
  ~ZMQSubscriber();
  string recv(bool noblock = true);
};

ZMQSubscriber::ZMQSubscriber(string ip, string port, string filter)
    : subscriber_socket_(context_, zmqpp::socket_type::subscribe) {
  sub_host_ = "tcp://" + ip + ":" + port;
  subscriber_socket_.subscribe(filter);
  subscriber_socket_.connect(sub_host_);
};

ZMQSubscriber::~ZMQSubscriber() { subscriber_socket_.disconnect(sub_host_); }

string ZMQSubscriber::recv(bool noblock) {
  zmqpp::message message;
  subscriber_socket_.receive(message, noblock);
  string str;
  if (message.parts() > 0) {
    str = message.get(0);
    std::cout << str << std::endl;
  } else {
    str = "";
  }
  return str;
}

int main() {

  const string subscriber_ip = "localhost";
  const string pub_port = "5556";
  const string sub_port = "5555";

  string subscriber_host, publisher_host;
  subscriber_host = "tcp://" + subscriber_ip + ":" + sub_port;
  publisher_host = "tcp://*:" + pub_port;

  ZMQPublisher zmq_pub(pub_port);
  ZMQSubscriber zmq_sub(subscriber_ip, sub_port);

  FrankaRobotStateMessage robot_state_msg;

  while (true) {
    string str = "Test publishing";
    // std::cout << "Publlish: " << str << std::endl;
    zmq_pub.send(str);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // string msg;
    // msg = zmq_sub.recv(true);
    // std::cout << "Subscriber" << msg << std::endl;
  }

  // const string host = "tcp://localhost:5555";
  // const string pub_host = "tcp://*:5555";

  // zmqpp::context context;

  // zmqpp::socket_type publish_type = zmqpp::socket_type::publish;
  // zmqpp::socket publish_socket (context, publish_type);

  // publish_socket.bind(pub_host);
  // zmqpp::message pub_msg;
  // string str = "msg: 0000 ";
  // pub_msg << str;

  // while (true) {
  //   publish_socket.send(str);
  // };
  // // string sent_str;
  // // pub_msg >> sent_str;
  // // std::cout << sent_str;
  // publish_socket.disconnect(pub_host);

  // zmqpp::socket_type type = zmqpp::socket_type::subscribe;
  // zmqpp::socket socket (context, type);

  // socket.subscribe("");

  // socket.connect(host);

  // zmqpp::message message;
  // bool noblock = false;
  // socket.receive(message, noblock);
  // std::cout << message.get(0);

  // socket.disconnect(host);
}
