#pragma once

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

namespace Logger {

enum class FMT { kFMT_DEFAULT, kFMT_FILE, kFMT_FUNC };

const std::string ToFormatString(FMT fmt);

void SetLogger(
    spdlog::level::level_enum level = spdlog::level::level_enum::debug,
    FMT fmt = FMT::kFMT_FUNC, const std::string& path = "log/log.log");

void SetLogger(const std::string& path, FMT fmt = FMT::kFMT_FUNC);

}  // namespace Logger