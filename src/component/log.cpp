#include "log.hpp"
namespace component {

namespace Logger {

const static std::string fmt_default("%+");
const static std::string fmt_filelogger(
    "[%Y-%m-%d %T.%3!u] %^[%l] [%!]%$ [%s:%#] %v");
const static std::string fmt_funcname(
    "[%Y-%m-%d %T.%3!u] %^[%l]%$ [%s:%#] \033[34m[%!]\033[0m %v");
const static std::string fmt_thread(
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

  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_pattern(fmt_str);
  auto file_sink =
      std::make_shared<spdlog::sinks::basic_file_sink_mt>(path, true);
  file_sink->set_pattern(fmt_default);

#if (SPDLOG_ACTIVE_LEVEL == SPDLOG_LEVEL_DEBUG)
  spdlog::flush_on(spdlog::level::debug);
  spdlog::set_level(spdlog::level::debug);
#elif (SPDLOG_ACTIVE_LEVEL == SPDLOG_LEVEL_INFO)
  spdlog::flush_on(spdlog::level::info);
  spdlog::set_level(spdlog::level::info);
#endif

  if (fmt != FMT::kFMT_FILE) {
    file_sink->set_pattern(fmt_str);
    spdlog::flush_on(level);
    spdlog::set_level(level);
  }

  spdlog::set_default_logger(std::make_shared<spdlog::logger>(
      "default", spdlog::sinks_init_list{console_sink, file_sink}));
  SPDLOG_DEBUG("Logging setted.");
}

}  // namespace Logger

}  // namespace component