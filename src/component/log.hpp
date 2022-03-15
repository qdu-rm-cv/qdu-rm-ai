#pragma once

#include <memory>

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

namespace component {

namespace Logger {

enum class FMT {
  kFMT_DEFAULT,
  kFMT_FILE,
  kFMT_TEST,
  kFMT_THREAD,
};

void SetLogger(
    spdlog::level::level_enum level = spdlog::level::level_enum::debug,
    FMT fmt = FMT::kFMT_TEST, const std::string& path = "log/log.log");

void SetLogger(const std::string& path, FMT fmt = FMT::kFMT_TEST);

}  // namespace Logger

}  // namespace component