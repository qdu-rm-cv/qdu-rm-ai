#include "semaphore.hpp"

#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "spdlog/spdlog.h"
#include "timer.hpp"

namespace {

component::Semaphore ping(0);
component::Semaphore pong(0);

std::atomic<int> counter{};
constexpr int count_limit = 1000;

void PingThread() {
  while (counter <= count_limit) {
    ping.Give();
    SPDLOG_INFO("Ping, {}", counter);
    ++counter;
    pong.TryTake();
  }
}

void PongThread() {
  while (counter < count_limit) {
    ping.TryTake();
    pong.Give();
    SPDLOG_INFO("Pong");
  }
}

}  // namespace

TEST(TestComponent, TestSemaphore) {
  component::Timer timer;
  timer.Start();

  std::thread t1(PingThread);
  std::thread t2(PongThread);
  t1.join();
  t2.join();

  timer.Calc("Pingpong game");
}
