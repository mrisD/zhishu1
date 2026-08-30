#include "zhishu_robot_mcp/control_policy.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <utility>

namespace zhishu_robot_mcp
{
namespace
{

std::string environment_value(const char * name)
{
  const char * value = std::getenv(name);
  return value == nullptr ? std::string{} : std::string{value};
}

std::string lowercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  return value;
}

bool parse_boolean(const char * name, bool default_value)
{
  const auto raw = lowercase(environment_value(name));
  if (raw.empty()) {
    return default_value;
  }
  if (raw == "1" || raw == "true" || raw == "yes" || raw == "on") {
    return true;
  }
  if (raw == "0" || raw == "false" || raw == "no" || raw == "off") {
    return false;
  }
  throw std::invalid_argument(std::string{name} + " must be a boolean value");
}

std::size_t parse_size(const char * name, std::size_t default_value)
{
  const auto raw = environment_value(name);
  if (raw.empty()) {
    return default_value;
  }
  std::size_t parsed_count = 0U;
  const auto parsed = std::stoull(raw, &parsed_count, 10);
  if (parsed_count != raw.size()) {
    throw std::invalid_argument(std::string{name} + " must be an unsigned integer");
  }
  return static_cast<std::size_t>(parsed);
}

double parse_double(const char * name, double default_value)
{
  const auto raw = environment_value(name);
  if (raw.empty()) {
    return default_value;
  }
  std::size_t parsed_count = 0U;
  const double parsed = std::stod(raw, &parsed_count);
  if (parsed_count != raw.size() || !std::isfinite(parsed)) {
    throw std::invalid_argument(std::string{name} + " must be a finite number");
  }
  return parsed;
}

}  // namespace

ServerOptions ServerOptions::from_environment()
{
  ServerOptions options;
  options.write_enabled = parse_boolean("ZHISHU_MCP_ENABLE_WRITES", false);
  options.allow_physical_hardware = parse_boolean("ZHISHU_MCP_ALLOW_PHYSICAL", false);
  options.max_writes_per_minute = parse_size("ZHISHU_MCP_MAX_WRITES_PER_MINUTE", 10U);
  options.max_physical_joint_step_rad =
    parse_double("ZHISHU_MCP_MAX_JOINT_STEP_RAD", 0.35);

  const auto raw_mode = lowercase(environment_value("ZHISHU_MCP_ROBOT_MODE"));
  if (raw_mode.empty() || raw_mode == "simulation" || raw_mode == "sim") {
    options.robot_mode = RobotMode::kSimulation;
  } else if (raw_mode == "physical" || raw_mode == "real") {
    options.robot_mode = RobotMode::kPhysical;
  } else {
    throw std::invalid_argument(
            "ZHISHU_MCP_ROBOT_MODE must be 'simulation' or 'physical'");
  }

  if (options.max_physical_joint_step_rad <= 0.0 ||
    options.max_physical_joint_step_rad > 3.14159265358979323846)
  {
    throw std::invalid_argument("ZHISHU_MCP_MAX_JOINT_STEP_RAD must be in (0, pi]");
  }
  return options;
}

std::string robot_mode_name(RobotMode mode)
{
  return mode == RobotMode::kPhysical ? "physical" : "simulation";
}

ControlPolicy::ControlPolicy(ServerOptions options)
: options_(std::move(options))
{
}

const ServerOptions & ControlPolicy::options() const noexcept
{
  return options_;
}

bool ControlPolicy::authorize(
  std::string_view required_confirmation,
  std::string_view supplied_confirmation,
  bool apply_rate_limit,
  std::string & reason)
{
  if (!options_.write_enabled) {
    reason = "write operations are disabled; set ZHISHU_MCP_ENABLE_WRITES=1 at server startup";
    return false;
  }
  if (options_.robot_mode == RobotMode::kPhysical && !options_.allow_physical_hardware) {
    reason =
      "physical writes are disabled; set ZHISHU_MCP_ALLOW_PHYSICAL=1 at server startup";
    return false;
  }
  if (!required_confirmation.empty() && supplied_confirmation != required_confirmation) {
    reason = "confirmation must exactly equal '" + std::string{required_confirmation} + "'";
    return false;
  }
  if (!apply_rate_limit || options_.max_writes_per_minute == 0U) {
    return true;
  }

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  discard_expired_locked(now);
  if (recent_writes_.size() >= options_.max_writes_per_minute) {
    reason = "write rate limit exceeded";
    return false;
  }
  recent_writes_.push_back(now);
  return true;
}

void ControlPolicy::discard_expired_locked(std::chrono::steady_clock::time_point now)
{
  const auto cutoff = now - std::chrono::minutes(1);
  while (!recent_writes_.empty() && recent_writes_.front() <= cutoff) {
    recent_writes_.pop_front();
  }
}

}  // namespace zhishu_robot_mcp
