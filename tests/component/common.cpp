#include "common.hpp"

#include "gtest/gtest.h"
#include "log.hpp"

TEST(TestComponent, TestGame) {
  ASSERT_TRUE(game::HasBigArmor(game::Model::kHERO));
  ASSERT_TRUE(game::HasBigArmor(game::Model::kSENTRY));
}

TEST(TestComponent, TestEuler) {
  component::Logger::SetLogger();
  component::Euler e(1, 1, 1);
  SPDLOG_WARN("{}", e.ToString());
}
