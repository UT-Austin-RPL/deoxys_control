#include "franka_controller.pb.h"
#include "google/protobuf/any.pb.h"
#include <iostream>

int main() {
  // Test using any to send arbitrary control message type

  FrankaGripperHomingMessage msg;
  // google::protobuf::Any any;

  FrankaGripperControlMessage control_msg;
  control_msg.mutable_control_msg()->PackFrom(msg);

  FrankaGripperHomingMessage homing_msg;

  if (control_msg.control_msg().UnpackTo(&homing_msg)) {
    std::cout << "Is homing!" << std::endl;
  }

  // control_msg.gripper_control().Is();
  // control_msg.set_allocated_gripper_control(&any);

  return 0;
}
