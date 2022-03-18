#pragma once

#include <chrono>
#include <thread>

#include "spdlog/spdlog.h"

namespace component {

class Timer {
 private:
  std::chrono::high_resolution_clock::time_point start_, end_;
  std::chrono::milliseconds duration_;

 public:
  Timer() { SPDLOG_TRACE("Constructed."); }
  ~Timer() { SPDLOG_TRACE("Destructed."); }

  void Start() { start_ = std::chrono::high_resolution_clock::now(); }

  std::chrono::milliseconds Calc(const std::string& duration_name) {
    end_ = std::chrono::high_resolution_clock::now();
    duration_ =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_ - start_);
    SPDLOG_INFO("Duration of {} : {}ms", duration_name, duration_.count());
    return duration_;
  }

  long int Count() const { return duration_.count(); }
};

class Recorder {
 private:
  std::thread thread_record_;
  std::string thread_name_;
  int fps_;

  void Print() {
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      SPDLOG_DEBUG("[{}] FPS : {}", thread_name_, fps_);
      fps_ = 0;
    }
  }

 public:
  Recorder(const std::string& name = "RecordThread", int fps = 0)
      : thread_name_(name), fps_(fps) {
    thread_record_ = std::thread(&Recorder::Print, this);
    SPDLOG_TRACE("Constructed.");
  }

  ~Recorder() {
    thread_record_.join();
    SPDLOG_TRACE("Destructed.");
  }

  void Record() {
    // SPDLOG_DEBUG("[RecordThread] FPS++");
    fps_++;
  }
};

}  // namespace component
