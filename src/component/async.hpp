#pragma once

#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include "semaphore.hpp"
#include "spdlog/spdlog.h"
#include "tbb/concurrent_vector.h"

template <typename Source, typename Result, typename Method>
class Async {
 public:
  size_t thread_count_;
  std::vector<std::thread *> threads_;
  bool thread_continue = false;

  std::deque<Source> source_;
  component::Semaphore source_signal_;
  std::mutex mutex_source_;

  std::deque<Result> result_;
  std::mutex mutex_result_;

  std::vector<Method *> method_vec_;

  virtual void Work(const size_t i) = 0;

  void Start() {
    method_vec_.clear();
    thread_continue = true;
    for (size_t i = 0; i < thread_count_; i++) {
      std::thread *t = new std::thread(&Async::Work, this, i);
      threads_.emplace_back(t);
      SPDLOG_WARN("start thread id : {}", i);
    }
  }

  void Stop() { thread_continue = false; }
};
