#pragma once

#include <memory>

#include "common.hpp"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

class App {
 private:
  void PrepareLogging(const std::string &path) {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto file_sink =
        std::make_shared<spdlog::sinks::basic_file_sink_mt>(path, true);

    spdlog::set_default_logger(std::make_shared<spdlog::logger>(
        "default", spdlog::sinks_init_list{console_sink, file_sink}));

#if (SPDLOG_ACTIVE_LEVEL == SPDLOG_LEVEL_DEBUG)
    spdlog::flush_on(spdlog::level::debug);
    RMlogger::SetLogger(spdlog::level::debug, RMlogger::fmt1);
#elif (SPDLOG_ACTIVE_LEVEL == SPDLOG_LEVEL_INFO)
    spdlog::flush_on(spdlog::level::info);
    RMlogger::SetLogger(spdlog::level::info, RMlogger::fmt1);
#endif
    SPDLOG_DEBUG("Logging setted.");
  }

 public:
  App(const std::string &log_path) { PrepareLogging(log_path); }
  /* 运行的主程序 */
  virtual void Run() = 0;
};
