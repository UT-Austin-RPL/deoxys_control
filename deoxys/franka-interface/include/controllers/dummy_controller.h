// Copyright 2022 Yifeng Zhu

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "controllers/base_controller.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_DUMMY_CONTROLLER_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_DUMMY_CONTROLLER_H_

namespace controller {
class DummyController : public BaseController {
public:
  DummyController();

  ~DummyController();
};
} // namespace controller

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_DUMMY_CONTROLLER_H_
