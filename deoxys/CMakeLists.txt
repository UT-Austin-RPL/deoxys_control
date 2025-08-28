cmake_minimum_required (VERSION 3.10)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

project (deoxys
  VERSION 0.1.0
  LANGUAGES CXX
  )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


if(CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(DEFAULT_PYTHON_MODULE_OUTPUT_DIRECTORY
    "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
else()
  set(DEFAULT_PYTHON_MODULE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
endif(CMAKE_LIBRARY_OUTPUT_DIRECTORY)

set(PYTHON_MODULE_OUTPUT_DIRECTORY "${DEFAULT_PYTHON_MODULE_OUTPUT_DIRECTORY}"
  CACHE STRING "Set the output directory when installing python modules")

add_compile_options(-Wall -Wextra)

message("${_GLIBCXX_USE_CXX11_ABI}")
find_package(Protobuf REQUIRED)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/protobuf.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/libfranka/cmake/FindEigen3.cmake)

# cmake-format: off
#  ____            _        _            __
# |  _ \ _ __ ___ | |_ ___ | |__  _   _ / _|
# | |_) | '__/ _ \| __/ _ \| '_ \| | | | |_
# |  __/| | | (_) | || (_) | |_) | |_| |  _|
# |_|   |_|  \___/ \__\___/|_.__/ \__,_|_|
#
# cmake-format: on

set(FRANKA_PROTO_FILES
  proto/franka-interface/franka_robot_state.proto
  proto/franka-interface/franka_controller.proto
  )

set(PROTO_FILES
  proto/sample.proto
  ${FRANKA_PROTO_FILES}
  )

message("FRANKA ${BUILD_FRANKA}")
message("DEOXYS ${BUILD_DEOXYS}")

if (BUILD_FRANKA)
  message("${PROTO_NAMESPACE}")  
  add_proto_cc_library(shared_protobuf
    NAMESPACE ${PROTO_NAMESPACE}
    PROTO_FILES ${FRANKA_PROTO_FILES}
    PROTO_LIB protobuf::libprotobuf)
  set(BUILD_TESTS OFF CACHE BOOL "Turn off libfranka tests")
  set(BUILD_EXAMPLES OFF CACHE BOOL "Turn off libfranka examples")
  find_package(Eigen3 REQUIRED)

  set(THREADS_PREFER_PTHREAD_FLAG ON)
  find_package(Threads REQUIRED)

  add_subdirectory(libfranka)
  add_subdirectory(zmqpp)
  add_subdirectory(spdlog)

  set(YAML_BUILD_SHARED_LIBS ON CACHE BOOL "BUild shared library of yaml-cpp")
  set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "")
  add_subdirectory(yaml-cpp)
  include_directories(zmqpp/src)
  include_directories(franka-interface/include)

  set(FRANKA_LIBS Eigen3::Eigen3 Franka::Franka Threads::Threads zmqpp yaml-cpp spdlog)

  # Add your new controller files here
  set (CONTROLLERS
    franka-interface/src/controllers/osc_impedance.cpp
    franka-interface/src/controllers/osc_position_impedance.cpp
    franka-interface/src/controllers/osc_yaw_impedance.cpp
    franka-interface/src/controllers/joint_position.cpp
    franka-interface/src/controllers/joint_impedance.cpp
    franka-interface/src/controllers/cartesian_velocity.cpp
    )
  
  add_library(shared_utils
    franka-interface/src/utils/common_utils.cpp
    franka-interface/src/utils/control_utils.cpp
    franka-interface/src/utils/robot_utils.cpp    
    franka-interface/src/utils/zmq_utils.cpp
    ${CONTROLLERS})
  target_link_libraries(shared_utils deoxys::shared_protobuf ${FRANKA_LIBS})
  add_library(deoxys::shared_utils ALIAS shared_utils)

  add_executable(franka-interface franka-interface/src/franka_control_node.cpp)
  target_link_libraries(franka-interface deoxys::shared_protobuf ${FRANKA_LIBS} deoxys::shared_utils)

  # Franka gripper interface
  add_executable(gripper-interface franka-interface/src/gripper_control_node.cpp)
  target_link_libraries(gripper-interface deoxys::shared_protobuf ${FRANKA_LIBS} deoxys::shared_utils)

  # ---------------------- The followings are example scripts ------------------------

  # Echo Robot State Example
  add_executable(robot-state-pub franka-interface/src/examples/robot_state_publisher.cpp)
  target_link_libraries(robot-state-pub ${FRANKA_LIBS} deoxys::shared_protobuf)

  # Load Yaml example
  add_executable(yaml-example franka-interface/src/examples/spdlog_example.cpp)
  target_link_libraries(yaml-example ${FRANKA_LIBS})

  # Spdlog example
  add_executable(spdlog-example franka-interface/src/examples/spdlog_example.cpp)
  target_link_libraries(spdlog-example ${FRANKA_LIBS})

  # zmq example
  add_executable(zmq-client franka-interface/src/test/test_zmq.cpp)
  target_link_libraries(zmq-client zmqpp deoxys::shared_protobuf)

  # test cartesian velocity controller
  add_executable(cartesian-velocity franka-interface/src/test/test_cartesian_velocity_control.cpp)
  target_link_libraries(cartesian-velocity deoxys::shared_protobuf ${FRANKA_LIBS} deoxys::shared_utils)


  install(TARGETS
    franka-interface
    gripper-interface
    zmq-client
    robot-state-pub
    yaml-example
    spdlog-example
    cartesian-velocity
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    )
endif(BUILD_FRANKA)

if (BUILD_DEOXYS)
  add_proto_py_library(py_protobuf
    PROTO_FILES ${PROTO_FILES}
    PROTO_LIB protobuf::libprotobuf)
  install(DIRECTORY "${PY_PROTOBUF_OUTPUT_DIR}/"
    DESTINATION "${PYTHON_MODULE_OUTPUT_DIRECTORY}/deoxys/proto"
    )
endif(BUILD_DEOXYS)
