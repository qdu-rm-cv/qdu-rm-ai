#include "log.hpp"

namespace Logger {

const std::string fmt_default("%+");
const std::string fmt_filelogger("[%Y-%m-%d %T.%3!u] %^[%l] [%!]%$ [%s:%#] %v");
const std::string fmt_funcname(
    "[%Y-%m-%d %T.%3!u] %^[%l]%$ [%s:%#] \033[34m[%!]\033[0m %v");

const std::string ToFormatString(FMT fmt) {
  switch (fmt) {
    case FMT::kFMT_DEFAULT:
      return fmt_default;
    case FMT::kFMT_FILE:
      return fmt_filelogger;
    case FMT::kFMT_TEST:
      return fmt_funcname;
    default:
      return fmt_default;
  }
}

void SetLogger(spdlog::level::level_enum level, FMT fmt,
               const std::string& path) {
  std::string fmt_str = ToFormatString(fmt);

  if (fmt == FMT::kFMT_FILE) {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_pattern(fmt_str);
    auto file_sink =
        std::make_shared<spdlog::sinks::basic_file_sink_mt>(path, true);
    file_sink->set_pattern(fmt_default);
    spdlog::set_default_logger(std::make_shared<spdlog::logger>(
        "default", spdlog::sinks_init_list{console_sink, file_sink}));

#if (SPDLOG_ACTIVE_LEVEL == SPDLOG_LEVEL_DEBUG)
    spdlog::flush_on(spdlog::level::debug);
    spdlog::set_level(spdlog::level::debug);
#elif (SPDLOG_ACTIVE_LEVEL == SPDLOG_LEVEL_INFO)
    spdlog::flush_on(spdlog::level::info);
    spdlog::set_level(spdlog::level::info);
#endif

  } else {
    spdlog::set_pattern(fmt_str);
    spdlog::set_level(level);
  }
  SPDLOG_DEBUG("Logging setted.");
}

void SetLogger(const std::string& path, FMT fmt) {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_pattern(ToFormatString(fmt));
  auto file_sink =
      std::make_shared<spdlog::sinks::basic_file_sink_mt>(path, true);
  file_sink->set_pattern(fmt_default);
  spdlog::set_default_logger(std::make_shared<spdlog::logger>(
      "default", spdlog::sinks_init_list{console_sink, file_sink}));

#if (SPDLOG_ACTIVE_LEVEL == SPDLOG_LEVEL_DEBUG)
  spdlog::flush_on(spdlog::level::debug);
  spdlog::set_level(spdlog::level::debug);
#elif (SPDLOG_ACTIVE_LEVEL == SPDLOG_LEVEL_INFO)
  spdlog::flush_on(spdlog::level::info);
  spdlog::set_level(spdlog::level::info);
#endif
  SPDLOG_DEBUG("Logging setted.");
}

}  // namespace Logger
