#pragma once

#include <memory>

#include "log.hpp"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

class App {
 public:
  App(const std::string &log_path) { Logger::SetLogger(log_path); }
  /* 运行的主程序 */
  virtual void Run() = 0;
};
