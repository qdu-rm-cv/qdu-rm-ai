#include "log.hpp"

namespace {

static int logger_num = 0;

int GetLoggerNum() { return logger_num++; }

}  // namespace

namespace component {

namespace Logger {

/*
 * %+ :         default log pattern
 * %Y-%m-%d :   date(yyyy-MM-dd)
 * %T :         clock, equivalent to %H:%M:%S(hh:mm:ss)
 * %3!u :       nanoseconds since previous message(3 bits)
 * %l :         level(trace, debug, info, warn, error, critical)
 * %L :         level(T, D, I, W, E, C)
 * %! :         function name
 * %s :         file name
 * %# :         line number
 * %v :         user text
 * %t :         thread id
 * %^[]%$ :     color start [] end
 *
 * see also : https://github.com/gabime/spdlog/wiki/3.-Custom-formatting
 */

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
  std::string logger_name = fmt::format("handle_{}", GetLoggerNum());
  std::string fmt_str = ToFormatString(fmt);

  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_pattern(fmt_str);
  console_sink->set_level(level);

  auto file_sink =
      std::make_shared<spdlog::sinks::basic_file_sink_mt>(path, true);
  file_sink->set_pattern(fmt_default);
  file_sink->set_level(level);

  std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};
  auto logger = std::make_shared<spdlog::logger>(logger_name.c_str(),
                                                 sinks.begin(), sinks.end());
  spdlog::register_logger(logger);
  spdlog::set_default_logger(logger);

#if (BUILD_TYPE == SPDLOG_LEVEL_DEBUG)
  spdlog::flush_on(spdlog::level::debug);
  spdlog::set_level(spdlog::level::debug);
  logger->set_level(spdlog::level::debug);
  logger->flush_on(spdlog::level::debug);
#elif (BUILD_TYPE == SPDLOG_LEVEL_INFO)
  spdlog::flush_on(spdlog::level::info);
  spdlog::set_level(spdlog::level::info);
  logger->set_level(spdlog::level::info);
  logger->flush_on(spdlog::level::info);
#endif

  SPDLOG_TRACE("Format code : {}", fmt_str);
  SPDLOG_DEBUG("file path : {}", path);
  SPDLOG_DEBUG("{} Logger is setted.", logger_name);
}

}  // namespace Logger

}  // namespace component
