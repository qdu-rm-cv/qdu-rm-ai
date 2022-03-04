#pragma once

#include "log.hpp"

class App {
 public:
  App(const std::string &log_path) { Logger::SetLogger(log_path); }
  /* 运行的主程序 */
  virtual void Run() = 0;
};
