# .rst: Protobuf Functions
# ------------------
#
# Locate the Google Protocol Buffers library.
#
# Will set PROTOC_EXE_FOUND to true if the protoc executable is found. By
# default will look for the protoc in the Conan Protobuf bin folder via the
# variable CONAN_BIN_DIRS_PROTOBUF.
#
# If PROTOC_EXE is found, then it will define a function called
# add_proto_cc_library with the following signature:
#
# ADD_PROTO_CC_LIBRARY(<TARGET_NAME> NAMESPACE <NAMESPACE> PROTO_FILES
# <proto_path> ... PROTO_LIB <target_name>)
#
# TARGET_NAME DEFINES THE library target name that will be created
#
# NAMESPACE defines an optional include file namspace. Without this the
# PROJECT_NAME variable will be used. Any target linking against the created
# protobuf library will need to include the generated headers via
# <NAMESPACE>/<proto name>.pb.h or <PROJECT_NAME>/<proto name>.pb.h if NAMESPACE
# is not specified.
#
# PROTO_FILES is a list of multiple protobuf file paths.
#
# PROTO_LIBRARY is the name of the protobuf library target you want to link
# against. If none is specified, then CONAN_PKG::Protobuf is assumed. It is
# assumed that this library has it's public/interface target properties set
# appropriately. It is not recommended that you specify this as the raw library
# protobuf.
#
# An alias target will also be generated with the pattern
# <PROJECT_NAME>::<TARGET_NAME>. It is recommended that you use this alias when
# linking to ensure that you are not linking against unexpected libraries with
# the same names due to typos, issues with the generation, etc.
#

# first find the protoc executable
find_program(PROTOC_EXE
             NAMES protoc
             DOC "The Google Protocol Buffers Compiler"
             PATHS "${CONAN_BIN_DIRS_PROTOBUF}")
mark_as_advanced(PROTOC_EXECUTABLE)

if(PROTOC_EXE)
  set(PROTOC_EXE_FOUND TRUE)
else()
  set(PROTOC_EXE_FOUND FALSE)
endif(PROTOC_EXE)

if(PROTOC_EXE_FOUND) # this is based on the function in FindProtobuf included
                     # with cmake,  but tweaked to use modern CMake style of
                     # targets, and to interact  nicely with Conan
  function(ADD_PROTO_CC_LIBRARY TARGET_NAME)
    cmake_parse_arguments(ARGS "" "NAMESPACE;PROTO_LIB" "PROTO_FILES" ${ARGN})

    if(NOT ARGS_NAMESPACE)
      set(NAMESPACE "${PROJECT_NAME}")
      set(NO_NAMESPACE TRUE)
    else()
      set(NAMESPACE "${ARGS_NAMESPACE}")
      set(NO_NAMESPACE FALSE)
    endif(NOT ARGS_NAMESPACE)

    if(NOT ARGS_PROTO_LIB)
      set(ARGS_PROTO_LIB CONAN_PKG::Protobuf)
    endif(NOT ARGS_PROTO_LIB)

    # construct protoc include directories
    foreach(FIL ${ARGS_PROTO_FILES})
      get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
      get_filename_component(ABS_PATH ${ABS_FIL} DIRECTORY)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
        list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif(${_contains_already} EQUAL -1)
    endforeach(FIL)

    if(NO_NAMESPACE)
      set(INCLUDE_DIR
          "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/proto/${NAMESPACE}")
      set(OUTPUT_DIR "${INCLUDE_DIR}")
    else()
      set(INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/proto")
      set(OUTPUT_DIR "${INCLUDE_DIR}/${NAMESPACE}")
    endif(NO_NAMESPACE)
    file(MAKE_DIRECTORY "${OUTPUT_DIR}")

    set(_proto_srcs)
    foreach(FIL ${ARGS_PROTO_FILES})
      get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
      get_filename_component(FIL_WE ${FIL} NAME_WE)
      # get_filename_component(FIL_DIR ${FIL} DIRECTORY)
      # # Hacking the path
      # string(SUBSTRING ${FIL_DIR} 5 -1 PROTO_DIR)

      list(APPEND _proto_srcs)
      list(APPEND _proto_hdrs)

      set(_curr_src "${OUTPUT_DIR}/${PROTO_DIR}/${FIL_WE}.pb.cc")
      set(_curr_hdr "${OUTPUT_DIR}/${PROTO_DIR}/${FIL_WE}.pb.h")

      add_custom_command(OUTPUT "${_curr_src}" "${_curr_hdr}"
                         COMMAND ${PROTOC_EXE} ARGS
                                 --cpp_out "${OUTPUT_DIR}"
                                 ${_protobuf_include_path} ${ABS_FIL}
                         DEPENDS ${ABS_FIL} ${PROTOC_EXE}
                         COMMENT
                           "Running C++ protocol buffer compiler on ${FIL}"
                           VERBATIM)

      set_source_files_properties(
        "${_curr_src}" "${_curr_hdr}" PROPERTIES GENERATED TRUE
      )

      list(APPEND _proto_srcs "${_curr_src}")
    endforeach(FIL)

    # create the library target
    add_library(${TARGET_NAME} ${_proto_srcs})
    target_link_libraries(${TARGET_NAME} PUBLIC ${ARGS_PROTO_LIB})
    target_include_directories(${TARGET_NAME} PUBLIC "${INCLUDE_DIR}")
    target_compile_options(${TARGET_NAME}
                           PRIVATE
                           "-Wno-everything"
                           "-Wno-reserved-id-macro"
                           "-Wno-zero-as-null-pointer-constant")
    add_library("${PROJECT_NAME}::${TARGET_NAME}" ALIAS ${TARGET_NAME})
  endfunction(ADD_PROTO_CC_LIBRARY)

  # write the python command target
  function(ADD_PROTO_PY_LIBRARY TARGET_NAME)
    cmake_parse_arguments(ARGS "" "NAMESPACE;PROTO_LIB" "PROTO_FILES" ${ARGN})

    if(NOT ARGS_NAMESPACE)
      set(NAMESPACE "${PROJECT_NAME}")
      set(NO_NAMESPACE TRUE)
    else()
      set(NAMESPACE "${ARGS_NAMESPACE}")
      set(NO_NAMESPACE FALSE)
    endif(NOT ARGS_NAMESPACE)

    if(NOT ARGS_PROTO_LIB)
      set(ARGS_PROTO_LIB CONAN_PKG::Protobuf)
    endif(NOT ARGS_PROTO_LIB)

    # construct protoc include directories
    foreach(FIL ${ARGS_PROTO_FILES})
      get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
      get_filename_component(ABS_PATH ${ABS_FIL} DIRECTORY)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
        list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif(${_contains_already} EQUAL -1)
    endforeach(FIL)

    if(NO_NAMESPACE)
      set(OUTPUT_DIR
          "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/proto/python")
    else()
      set(
        OUTPUT_DIR
        "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/proto/python/${NAMESPACE}"
      )
    endif(NO_NAMESPACE)
    string(TOUPPER ${TARGET_NAME} UPPER_TARGET_NAME)
    set("${UPPER_TARGET_NAME}_OUTPUT_DIR" "${OUTPUT_DIR}" PARENT_SCOPE)
    file(MAKE_DIRECTORY "${OUTPUT_DIR}")

    set(_proto_srcs)
    foreach(FIL ${ARGS_PROTO_FILES})
      get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
      get_filename_component(FIL_WE ${FIL} NAME_WE)

      list(APPEND _proto_srcs)

      set(_curr_src "${OUTPUT_DIR}/${FIL_WE}_pb2.py")

      add_custom_command(OUTPUT "${_curr_src}"
                         COMMAND ${PROTOC_EXE} ARGS
                                 --python_out "${OUTPUT_DIR}"
                                 ${_protobuf_include_path} ${ABS_FIL}
                         DEPENDS ${ABS_FIL} ${PROTOC_EXE}
                         COMMENT
                           "Running Python protocol buffer compiler on ${FIL}"
                           VERBATIM)

      set_source_files_properties("${_curr_src}" PROPERTIES GENERATED TRUE)

      list(APPEND _proto_srcs "${_curr_src}")
    endforeach(FIL)

    add_custom_target(${TARGET_NAME} ALL DEPENDS ${_proto_srcs})
  endfunction(ADD_PROTO_PY_LIBRARY)
else() # write a dummy function wrapper. Should throw a fatal error if this
       # function is called as PROTOC was not found.
  function(ADD_PROTO_CC_LIBRARY TARGET_NAME)
    message(FATAL_ERROR
              "PROTOC_EXE NOT FOUND. Could not generate proto cc library.")
  endfunction(ADD_PROTO_CC_LIBRARY)

  function(ADD_PROTO_PY_LIBRARY TARGET_NAME)
    message(FATAL_ERROR
              "PROTOC_EXE NOT FOUND. Could not generate proto python library.")
  endfunction(ADD_PROTO_PY_LIBRARY)
endif(PROTOC_EXE_FOUND)
