#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <unistd.h>

namespace fs = std::filesystem;

namespace
{

std::tm local_time(std::time_t value)
{
  std::tm result{};
  localtime_r(&value, &result);
  return result;
}

std::string format_date(std::time_t value)
{
  const auto tm = local_time(value);
  std::ostringstream stream;
  stream << std::put_time(&tm, "%Y-%m-%d");
  return stream.str();
}

std::string format_run_id(std::time_t value)
{
  const auto tm = local_time(value);
  std::ostringstream stream;
  stream << std::put_time(&tm, "%Y%m%d_%H%M%S") << "_" << ::getpid();
  return stream.str();
}

std::string format_session_name(std::time_t value)
{
  const auto tm = local_time(value);
  std::ostringstream stream;
  stream << std::put_time(&tm, "%H-%M-%S");
  return stream.str();
}

std::string format_timestamp(std::int64_t seconds, std::uint32_t nanoseconds)
{
  const auto tm = local_time(static_cast<std::time_t>(seconds));
  std::ostringstream stream;
  stream << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
         << "." << std::setfill('0') << std::setw(3) << nanoseconds / 1000000U;
  return stream.str();
}

std::string severity_name(std::uint8_t severity)
{
  switch (severity) {
    case rcl_interfaces::msg::Log::DEBUG:
      return "DEBUG";
    case rcl_interfaces::msg::Log::INFO:
      return "INFO";
    case rcl_interfaces::msg::Log::WARN:
      return "WARN";
    case rcl_interfaces::msg::Log::ERROR:
      return "ERROR";
    case rcl_interfaces::msg::Log::FATAL:
      return "FATAL";
    default:
      return "LEVEL_" + std::to_string(static_cast<unsigned int>(severity));
  }
}

std::string sanitize_one_line(const std::string & value)
{
  std::string result;
  result.reserve(value.size());
  for (const char character : value) {
    if (character == '\n') {
      result += "\\n";
    } else if (character == '\r') {
      result += "\\r";
    } else {
      result += character;
    }
  }
  return result;
}

std::string basename_or_empty(const std::string & path)
{
  if (path.empty()) {
    return {};
  }
  return fs::path(path).filename().string();
}

fs::path default_log_root()
{
  if (const char * configured = std::getenv("ZHISHU_ROBOT_LOG_DIR")) {
    if (configured[0] != '\0') {
      return fs::path(configured);
    }
  }

  // An ament prefix is normally either <workspace>/install (merged install)
  // or <workspace>/install/<package> (isolated install). Walk upward to the
  // install directory so `ros2 run` uses the same root as the launch files.
  if (const char * prefixes = std::getenv("AMENT_PREFIX_PATH")) {
    std::istringstream stream(prefixes);
    std::string prefix;
    while (std::getline(stream, prefix, ':')) {
      fs::path current(prefix);
      while (!current.empty()) {
        if (current.filename() == "install") {
          return current.parent_path() / "zhishu_robot_logs";
        }
        const auto parent = current.parent_path();
        if (parent == current) {
          break;
        }
        current = parent;
      }
    }
  }

  return fs::current_path() / "zhishu_robot_logs";
}

}  // namespace

class ZhishuRobotLogRecorder final : public rclcpp::Node
{
public:
  ZhishuRobotLogRecorder()
  : Node("zhishu_robot_log_recorder")
  {
    const auto configured_root = declare_parameter<std::string>("log_root", "");
    session_name_ = declare_parameter<std::string>("session_name", "");
    file_prefix_ = declare_parameter<std::string>("file_prefix", "zhishu_robot");
    const auto max_file_size_mb = declare_parameter<int64_t>("max_file_size_mb", 10);
    min_severity_ = declare_parameter<int64_t>("min_severity", rcl_interfaces::msg::Log::DEBUG);
    flush_each_line_ = declare_parameter<bool>("flush_each_line", true);
    const auto excluded = declare_parameter<std::vector<std::string>>(
      "exclude_loggers", {"zhishu_robot_log_recorder"});

    if (max_file_size_mb <= 0) {
      throw std::invalid_argument("max_file_size_mb must be greater than zero");
    }
    if (file_prefix_.empty()) {
      throw std::invalid_argument("file_prefix must not be empty");
    }
    if (session_name_.empty()) {
      session_name_ = format_session_name(std::time(nullptr));
    }
    if (session_name_ == "." || session_name_ == ".." ||
      !std::regex_match(session_name_, std::regex("[A-Za-z0-9_.-]+")))
    {
      throw std::invalid_argument(
              "session_name must contain only letters, digits, '.', '_' or '-'");
    }

    max_file_size_bytes_ = static_cast<std::uint64_t>(max_file_size_mb) * 1024ULL * 1024ULL;
    log_root_ = configured_root.empty() ? default_log_root() : fs::path(configured_root);
    excluded_loggers_.insert(excluded.begin(), excluded.end());
    run_id_ = format_run_id(std::time(nullptr));

    open_for_day(format_date(std::time(nullptr)));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1000));
    qos.reliable();
    qos.durability_volatile();
    subscription_ = create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", qos,
      [this](const rcl_interfaces::msg::Log::SharedPtr message) {
        record(*message);
      });

    RCLCPP_INFO(
      get_logger(),
      "Central log recorder ready: %s (rotate at %ld MiB, min severity %ld)",
      active_path_.c_str(), static_cast<long>(max_file_size_mb),
      static_cast<long>(min_severity_));
  }

  ~ZhishuRobotLogRecorder() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_.is_open()) {
      stream_ << "# recorder stopped\n";
      stream_.flush();
      stream_.close();
    }
  }

private:
  void record(const rcl_interfaces::msg::Log & message)
  {
    if (static_cast<int64_t>(message.level) < min_severity_ ||
      excluded_loggers_.count(message.name) != 0U)
    {
      return;
    }

    std::int64_t seconds = message.stamp.sec;
    std::uint32_t nanoseconds = message.stamp.nanosec;
    if (seconds <= 0) {
      const auto now = std::chrono::system_clock::now();
      const auto duration = now.time_since_epoch();
      seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
      nanoseconds = static_cast<std::uint32_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() % 1000000000LL);
    }

    try {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto day = format_date(static_cast<std::time_t>(seconds));
      if (day != active_day_) {
        open_for_day(day);
      }

      std::ostringstream line;
      line << "[" << format_timestamp(seconds, nanoseconds) << "] "
           << "[" << severity_name(message.level) << "] "
           << "[" << sanitize_one_line(message.name) << "] ";

      const auto file = basename_or_empty(message.file);
      if (!file.empty() || !message.function.empty()) {
        line << "[" << file;
        if (message.line != 0U) {
          line << ":" << message.line;
        }
        if (!message.function.empty()) {
          line << " " << sanitize_one_line(message.function);
        }
        line << "] ";
      }
      line << sanitize_one_line(message.msg) << "\n";

      const auto text = line.str();
      if (current_size_bytes_ > 0U &&
        current_size_bytes_ + text.size() > max_file_size_bytes_)
      {
        ++part_index_;
        open_file();
      }
      stream_ << text;
      current_size_bytes_ += text.size();
      if (flush_each_line_) {
        stream_.flush();
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to write central log: %s", error.what());
    }
  }

  void open_for_day(const std::string & day)
  {
    active_day_ = day;
    part_index_ = 0U;
    active_directory_ = log_root_ / active_day_ / session_name_;
    fs::create_directories(active_directory_);
    open_file();
  }

  void open_file()
  {
    if (stream_.is_open()) {
      stream_.flush();
      stream_.close();
    }

    std::ostringstream filename;
    filename << file_prefix_ << "_" << run_id_;
    if (part_index_ != 0U) {
      filename << "_part" << std::setfill('0') << std::setw(3) << part_index_;
    }
    filename << ".log";

    active_path_ = active_directory_ / filename.str();
    stream_.open(active_path_, std::ios::out | std::ios::app);
    if (!stream_) {
      throw std::runtime_error("cannot open log file: " + active_path_.string());
    }
    current_size_bytes_ = fs::exists(active_path_) ? fs::file_size(active_path_) : 0U;
    stream_ << "# zhishu_robot central log; run=" << run_id_
            << "; part=" << part_index_ << "\n";
    stream_.flush();
    current_size_bytes_ = fs::file_size(active_path_);
  }

  fs::path log_root_;
  fs::path active_directory_;
  fs::path active_path_;
  std::string file_prefix_;
  std::string run_id_;
  std::string active_day_;
  std::string session_name_;
  std::ofstream stream_;
  std::uint64_t max_file_size_bytes_{10ULL * 1024ULL * 1024ULL};
  std::uint64_t current_size_bytes_{0U};
  std::uint32_t part_index_{0U};
  int64_t min_severity_{rcl_interfaces::msg::Log::DEBUG};
  bool flush_each_line_{true};
  std::unordered_set<std::string> excluded_loggers_;
  std::mutex mutex_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ZhishuRobotLogRecorder>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("zhishu_robot_log_recorder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
