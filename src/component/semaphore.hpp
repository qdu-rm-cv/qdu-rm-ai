#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

namespace component {

class Semaphore {
 public:
  explicit Semaphore(int count = 0) : count_(count) {}

  void Init(int count = 0) { count_ = count; }

  void Signal() {
    std::unique_lock<std::mutex> lock(mutex_);
    ++count_;
    condition_.notify_one();
    lock.unlock();
  }

  void Wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [=] { return count_ > 0; });
    --count_;
    lock.unlock();
  }

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  int count_;
};

}  // namespace component
