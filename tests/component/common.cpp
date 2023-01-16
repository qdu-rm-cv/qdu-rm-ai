#include "common.hpp"

#include "gtest/gtest.h"
#include "log.hpp"

TEST(TestComponent, TestGame) {
  ASSERT_TRUE(game::HasBigArmor(game::Model::kHERO));
  ASSERT_TRUE(game::HasBigArmor(game::Model::kSENTRY));
}

TEST(TestComponent, TestEuler) {
  component::logger::SetLogger();
  component::Euler e(1, 1, 1);
  SPDLOG_DEBUG("{}", e.ToString());
}

TEST(TestComponent, TestRandomValue) {
  for (int i = 0; i < 6; i++)
    SPDLOG_DEBUG("{}", algo::GetRealRandomValue(0, 3));
  for (int i = 0; i < 6; i++) SPDLOG_DEBUG("{}", algo::GetIntRandomValue(0, 3));
  for (int i = 0; i < 6; i++) SPDLOG_DEBUG("{}", algo::GetIntRandomValue(6, 3));
}
