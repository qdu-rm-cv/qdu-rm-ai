#include "robot.hpp"

#include "gtest/gtest.h"
#include "spdlog/spdlog.h"

TEST(TestRobot, ExampleTest) {
#if WITH_MCU
  Robot robot("/dev/ttyTHS2");
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
#endif
}
