#include <memory>

#include "spdlog/async.h"
#include "spdlog/cfg/env.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

void multi_sink_example() {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::warn);
  console_sink->set_pattern("[multi_sink_example] [%^%l%$] %v");

  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
      "logs/multisink.txt", true);
  file_sink->set_level(spdlog::level::trace);

  spdlog::logger logger("multi_sink", {console_sink, file_sink});
  logger.set_level(spdlog::level::debug);
  spdlog::set_default_logger(std::shared_ptr<spdlog::logger>(&logger));
  logger.warn("this should appear in both console and file");
  logger.info(
      "this message should not appear in the console, only in the file");
}

void get_logger() {
  spdlog::cfg::load_env_levels();
  spdlog::init_thread_pool(8192, 1);
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::debug);
  console_sink->set_pattern("[%n - %H:%M:%S %z] [%^%l%$] [thread %t] %!");

  // auto file_sink =
  // std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/multisink.txt",
  // true);
  auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      "logs/rotating.txt", 1048576, 5, false);
  file_sink->set_level(spdlog::level::trace);
  // auto logger =
  // spdlog::rotating_logger_mt<spdlog::async_factory>("basic_logger",
  // "logs/rotating.txt", 1048576 * 5, 3); spdlog::logger logger("basic_logger",
  // {console_sink, file_sink});
  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(console_sink);
  sinks.push_back(file_sink);

  auto logger = std::make_shared<spdlog::async_logger>(
      "basic_logger", sinks.begin(), sinks.end(), spdlog::thread_pool(),
      spdlog::async_overflow_policy::overrun_oldest);
  logger->set_level(spdlog::level::trace);
  spdlog::register_logger(logger);
}

int main(int argc, char *argv[]) {

  // directly imported from spdlog repo

  get_logger();
  auto logger = spdlog::get("basic_logger");
  SPDLOG_LOGGER_INFO(logger, "Info message");
  SPDLOG_LOGGER_CRITICAL(logger, "critical message");
  SPDLOG_LOGGER_WARN(logger, "Warn message");
  SPDLOG_LOGGER_TRACE(logger, "Trace message");
  logger->info("Welcome to spdlog!");
  logger->warn("WARNING!!!");
  logger->critical("Critical information");
  logger->info("Positional args are {1} {0}..", "too", "supported");

  // // multi_sink_example();
  // spdlog::info("Welcome to spdlog!");
  // spdlog::error("Some error message with arg: {}", 1);

  // spdlog::warn("Easy padding in numbers like {:08d}", 12);
  // spdlog::critical("Support for int: {0:d};  hex: {0:x};  oct: {0:o}; bin:
  // {0:b}", 42); spdlog::info("Support for floats {:03.2f}", 1.23456);
  // spdlog::info("Positional args are {1} {0}..", "too", "supported");
  // spdlog::info("{:<30}", "left aligned");

  // // spdlog::set_level(spdlog::level::debug); // Set global log level to
  // debug spdlog::debug("This message should be displayed..");

  // // change log pattern
  // spdlog::set_pattern("[%H:%M:%S %z] [%n] [%^---%L---%$] [thread %t] %v");

  // // Compile time log levels
  // // define SPDLOG_ACTIVE_LEVEL to desired level
  SPDLOG_TRACE("Some trace message with param {}", 42);
  spdlog::shutdown();
  // SPDLOG_DEBUG("Some debug message");
}