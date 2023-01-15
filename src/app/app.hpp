#pragma once

#include "log.hpp"

class App {
 public:
  App(const std::string &log_path,
      component::Logger::FMT fmt = component::Logger::FMT::kFMT_THREAD) {
    component::Logger::SetLogger(log_path, fmt);
    SPDLOG_DEBUG("Log path : {}", log_path);
    SPDLOG_TRACE("Constructed App.");
  }
  /* 运行的主程序 */
  virtual void Run() = 0;
};
