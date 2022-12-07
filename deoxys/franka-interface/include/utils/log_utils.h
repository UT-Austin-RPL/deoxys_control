// Copyright 2022 Yifeng Zhu
#include <string>

#include <spdlog/async.h>
#include <spdlog/cfg/env.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_LOG_UTILS_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_LOG_UTILS_H_

namespace log_utils {
inline void
initialize_logger(std::string logger_name = "deoxys_logger",
                  std::string console_level = "info", bool use_console = true,
                  std::string log_filename = "logs/deoxys_control_program.log",
                  std::string file_level = "trace", bool use_file = true) {
  spdlog::init_thread_pool(8192, 1);
  spdlog::cfg::load_env_levels();
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::from_str(console_level));
  console_sink->set_pattern("[%^%l - %n%$] %v %!");
  auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      log_filename, 1048576, 5, false);
  file_sink->set_level(spdlog::level::from_str(file_level));
  std::vector<spdlog::sink_ptr> sinks;

  if (use_console) {
    sinks.push_back(console_sink);
  }
  if (use_file) {
    sinks.push_back(file_sink);
  }

  auto logger = std::make_shared<spdlog::async_logger>(
      logger_name, sinks.begin(), sinks.end(), spdlog::thread_pool(),
      spdlog::async_overflow_policy::overrun_oldest);
  logger->set_level(spdlog::level::trace);
  spdlog::register_logger(logger);
};

inline std::shared_ptr<spdlog::logger>
get_logger(std::string logger_name = "deoxys_logger") {
  return spdlog::get(logger_name);
};

} // namespace log_utils

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_LOG_UTILS_H_
