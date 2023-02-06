#include "log.hpp"

#include "gtest/gtest.h"

TEST(TestComponent, TestLog) {
  SPDLOG_TRACE("trace message");
  SPDLOG_DEBUG("debug message");
  SPDLOG_INFO("info message");
  SPDLOG_WARN("warn message");
  SPDLOG_ERROR("error message");
  SPDLOG_CRITICAL("critical message");
  spdlog::trace("spdlog::trace message");
  spdlog::debug("spdlog::debug message");
  spdlog::info("spdlog::info message");
  spdlog::warn("spdlog::warn message");
  spdlog::error("spdlog::error message");
  spdlog::critical("spdlog::critical message");
}

TEST(TestComponent, TestSetLogger) {
  component::logger::SetLogger("log/test.log",
                               component::logger::FMT::kFMT_DEFAULT,
                               spdlog::level::debug);
  SPDLOG_DEBUG("debug message");
  SPDLOG_INFO("info message");
  SPDLOG_INFO("active level : {}", component::logger::GetLevelString());
}
