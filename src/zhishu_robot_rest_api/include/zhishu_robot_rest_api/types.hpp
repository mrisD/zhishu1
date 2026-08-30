#ifndef ZHISHU_ROBOT_REST_API__TYPES_HPP_
#define ZHISHU_ROBOT_REST_API__TYPES_HPP_

#include <nlohmann/json.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

namespace zhishu_robot_rest_api
{

using Json = nlohmann::json;

enum class RobotMode
{
  kSimulation,
  kPhysical,
};

struct RestOptions
{
  std::string host{"127.0.0.1"};
  std::uint16_t port{8088U};
  std::size_t max_request_bytes{64U * 1024U};
  std::string cors_origin;
  std::string api_token;
  bool allow_remote{false};
  bool write_enabled{false};
  RobotMode robot_mode{RobotMode::kSimulation};
  bool allow_physical_hardware{false};
  std::size_t max_writes_per_minute{10U};
  double max_physical_joint_step_rad{0.35};

  static RestOptions from_environment();
  bool is_loopback() const;
  bool read_auth_required() const;
  void validate() const;
};

std::string robot_mode_name(RobotMode mode);

struct ApiResult
{
  bool ok{false};
  std::string code;
  std::string message;
  Json data{Json::object()};
  int http_status{500};

  static ApiResult success(
    Json value = Json::object(),
    std::string text = "",
    int status = 200)
  {
    return ApiResult{true, "ok", std::move(text), std::move(value), status};
  }

  static ApiResult failure(
    std::string error_code,
    std::string text,
    int status,
    Json value = Json::object())
  {
    return ApiResult{
      false, std::move(error_code), std::move(text), std::move(value), status};
  }
};

}  // namespace zhishu_robot_rest_api

#endif  // ZHISHU_ROBOT_REST_API__TYPES_HPP_
