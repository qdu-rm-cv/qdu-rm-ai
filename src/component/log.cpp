#include "log.hpp"

namespace component {

namespace Logger {

static const std::string fmt_default("%+");
static const std::string fmt_filelogger(
    "[%Y-%m-%d %T.%3!u] %^[%l] [%!]%$ [%s:%#] %v");
static const std::string fmt_funcname(
    "[%Y-%m-%d %T.%3!u] %^[%l]%$ [%s:%#] \033[34m[%!]\033[0m %v");
static const std::string fmt_thread(
    "[%Y-%m-%d %T.%3!u] (id:%t) %^[%l]%$ [%s:%#] [%!] %v");

const std::string& ToFormatString(FMT fmt) {
  switch (fmt) {
    case FMT::kFMT_DEFAULT:
      return fmt_default;
    case FMT::kFMT_FILE:
      return fmt_filelogger;
    case FMT::kFMT_TEST:
      return fmt_funcname;
    case FMT::kFMT_THREAD:
      return fmt_thread;
    default:
      return fmt_default;
  }
}

void SetLogger(const std::string& path, FMT fmt,
               spdlog::level::level_enum level) {
  std::string fmt_str = ToFormatString(fmt);
  SPDLOG_TRACE("Format code : {}", fmt_str);

  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_pattern(fmt_str);
  auto file_sink =
      std::make_shared<spdlog::sinks::basic_file_sink_mt>(path, true);
  file_sink->set_pattern(fmt_default);

  spdlog::logger logger("QDU-AI", {console_sink, file_sink});
  console_sink->set_level(level);

#if (BUILD_TYPE == SPDLOG_LEVEL_DEBUG)
  spdlog::flush_on(spdlog::level::debug);
  console_sink->set_level(spdlog::level::debug);
#elif (BUILD_TYPE == SPDLOG_LEVEL_INFO)
  spdlog::flush_on(spdlog::level::info);
  console_sink->set_level(spdlog::level::info);
#endif

  SPDLOG_DEBUG("Logger is setted.");
}

}  // namespace Logger

}  // namespace component
