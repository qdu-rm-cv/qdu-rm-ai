#pragma once

#include <semaphore.h>
#include <time.h>

#include <cstdint>

namespace {

struct timespec CalDuration(uint32_t timeout) {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  uint32_t secs = timeout / 1000;
  timeout = timeout % 1000;

  uint64_t raw_time = timeout * 1000U * 1000U + ts.tv_nsec;
  uint32_t add = raw_time / (1000U * 1000U * 1000U);
  ts.tv_sec += (add + secs);
  ts.tv_nsec = raw_time % (1000U * 1000U * 1000U);
  return ts;
}

}  // namespace

namespace component {

class Semaphore {
 public:
  explicit Semaphore(uint16_t max_count = 1, int init_count = 0)
      : max_count_(max_count) {
    Init(init_count);
  }

  void Init(int init_count = 0) { sem_init(&this->handle_, 0, init_count); }

  // Signal, V
  void Give() {
    int tmp = 0;
    sem_getvalue(&this->handle_, &tmp);
    if (tmp < this->max_count_) {
      sem_post(&this->handle_);
    }
  }

  // Wait, P
  bool Take() { return sem_wait(&handle_) == 0; }
  bool Take(uint32_t timeout) {
    struct timespec ts = CalDuration(timeout);
    return sem_timedwait(&handle_, &ts) == 0;
  }

  uint32_t GetCount() {
    int value = 0;
    sem_getvalue(&handle_, &value);
    return value;
  }

  int GetMaxCount() { return max_count_; }

 private:
  sem_t handle_;
  int max_count_;
};

}  // namespace component
