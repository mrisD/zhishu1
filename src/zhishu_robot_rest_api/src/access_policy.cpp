#include "zhishu_robot_rest_api/access_policy.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <utility>

namespace zhishu_robot_rest_api
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
  std::size_t count = 0U;
  const auto value = std::stoull(raw, &count, 10);
  if (count != raw.size() || value > std::numeric_limits<std::size_t>::max()) {
    throw std::invalid_argument(std::string{name} + " must be an unsigned integer");
  }
  return static_cast<std::size_t>(value);
}

double parse_double(const char * name, double default_value)
{
  const auto raw = environment_value(name);
  if (raw.empty()) {
    return default_value;
  }
  std::size_t count = 0U;
  const double value = std::stod(raw, &count);
  if (count != raw.size() || !std::isfinite(value)) {
    throw std::invalid_argument(std::string{name} + " must be a finite number");
  }
  return value;
}

}  // namespace

RestOptions RestOptions::from_environment()
{
  RestOptions options;
  const auto configured_host = environment_value("ZHISHU_REST_HOST");
  if (!configured_host.empty()) {
    options.host = configured_host;
  }
  const auto port = parse_size("ZHISHU_REST_PORT", 8088U);
  if (port > 65535U) {
    throw std::invalid_argument("ZHISHU_REST_PORT must be in [1, 65535]");
  }
  options.port = static_cast<std::uint16_t>(port);
  options.max_request_bytes = parse_size("ZHISHU_REST_MAX_REQUEST_BYTES", 64U * 1024U);
  options.cors_origin = environment_value("ZHISHU_REST_CORS_ORIGIN");
  options.api_token = environment_value("ZHISHU_REST_API_TOKEN");
  options.allow_remote = parse_boolean("ZHISHU_REST_ALLOW_REMOTE", false);
  options.write_enabled = parse_boolean("ZHISHU_REST_ENABLE_WRITES", false);
  options.allow_physical_hardware =
    parse_boolean("ZHISHU_REST_ALLOW_PHYSICAL", false);
  options.max_writes_per_minute =
    parse_size("ZHISHU_REST_MAX_WRITES_PER_MINUTE", 10U);
  options.max_physical_joint_step_rad =
    parse_double("ZHISHU_REST_MAX_JOINT_STEP_RAD", 0.35);

  const auto mode = lowercase(environment_value("ZHISHU_REST_ROBOT_MODE"));
  if (mode.empty() || mode == "simulation" || mode == "sim") {
    options.robot_mode = RobotMode::kSimulation;
  } else if (mode == "physical" || mode == "real") {
    options.robot_mode = RobotMode::kPhysical;
  } else {
    throw std::invalid_argument(
            "ZHISHU_REST_ROBOT_MODE must be 'simulation' or 'physical'");
  }
  options.validate();
  return options;
}

bool RestOptions::is_loopback() const
{
  const auto normalized = lowercase(host);
  return normalized == "127.0.0.1" || normalized == "localhost" || normalized == "::1";
}

bool RestOptions::read_auth_required() const
{
  return !is_loopback();
}

void RestOptions::validate() const
{
  if (host.empty()) {
    throw std::invalid_argument("ZHISHU_REST_HOST must not be empty");
  }
  if (port == 0U) {
    throw std::invalid_argument("ZHISHU_REST_PORT must be in [1, 65535]");
  }
  if (max_request_bytes < 1024U || max_request_bytes > 1024U * 1024U) {
    throw std::invalid_argument("ZHISHU_REST_MAX_REQUEST_BYTES must be in [1024, 1048576]");
  }
  if (max_physical_joint_step_rad <= 0.0 ||
    max_physical_joint_step_rad > 3.14159265358979323846)
  {
    throw std::invalid_argument("ZHISHU_REST_MAX_JOINT_STEP_RAD must be in (0, pi]");
  }
  if (!is_loopback() && !allow_remote) {
    throw std::invalid_argument(
            "non-loopback binding requires ZHISHU_REST_ALLOW_REMOTE=1");
  }
  if ((!is_loopback() || write_enabled) && api_token.empty()) {
    throw std::invalid_argument(
            "remote access and write mode require ZHISHU_REST_API_TOKEN");
  }
  if ((!is_loopback() || write_enabled) && api_token.size() < 24U) {
    throw std::invalid_argument("ZHISHU_REST_API_TOKEN must contain at least 24 characters");
  }
  if (!cors_origin.empty() && cors_origin == "*") {
    throw std::invalid_argument("ZHISHU_REST_CORS_ORIGIN must be an exact origin, not '*'");
  }
}

std::string robot_mode_name(RobotMode mode)
{
  return mode == RobotMode::kPhysical ? "physical" : "simulation";
}

AccessPolicy::AccessPolicy(RestOptions options)
: options_(std::move(options))
{
  options_.validate();
}

const RestOptions & AccessPolicy::options() const noexcept
{
  return options_;
}

bool AccessPolicy::authorize_read(std::string_view bearer_token, std::string & reason) const
{
  if (!options_.read_auth_required()) {
    return true;
  }
  if (!token_matches(bearer_token)) {
    reason = "a valid bearer token is required";
    return false;
  }
  return true;
}

bool AccessPolicy::authorize_write(
  std::string_view bearer_token,
  std::string_view required_confirmation,
  std::string_view supplied_confirmation,
  bool apply_rate_limit,
  std::string & reason)
{
  if (!options_.write_enabled) {
    reason = "write operations are disabled; set ZHISHU_REST_ENABLE_WRITES=1 at startup";
    return false;
  }
  if (!token_matches(bearer_token)) {
    reason = "a valid bearer token is required for write operations";
    return false;
  }
  if (options_.robot_mode == RobotMode::kPhysical && !options_.allow_physical_hardware) {
    reason = "physical writes require ZHISHU_REST_ALLOW_PHYSICAL=1 at startup";
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

bool AccessPolicy::token_matches(std::string_view supplied) const
{
  const auto expected = std::string_view{options_.api_token};
  std::size_t difference = expected.size() ^ supplied.size();
  const auto maximum = std::max(expected.size(), supplied.size());
  for (std::size_t index = 0; index < maximum; ++index) {
    const unsigned char left = index < expected.size() ?
      static_cast<unsigned char>(expected[index]) : 0U;
    const unsigned char right = index < supplied.size() ?
      static_cast<unsigned char>(supplied[index]) : 0U;
    difference |= static_cast<std::size_t>(left ^ right);
  }
  return difference == 0U && !expected.empty();
}

void AccessPolicy::discard_expired_locked(std::chrono::steady_clock::time_point now)
{
  const auto cutoff = now - std::chrono::minutes(1);
  while (!recent_writes_.empty() && recent_writes_.front() <= cutoff) {
    recent_writes_.pop_front();
  }
}

}  // namespace zhishu_robot_rest_api
