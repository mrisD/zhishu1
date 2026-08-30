#ifndef ZHISHU_ROBOT_MCP__RESULT_HPP_
#define ZHISHU_ROBOT_MCP__RESULT_HPP_

#include <cxxmcp/protocol/types.hpp>

#include <string>
#include <utility>

namespace zhishu_robot_mcp
{

using Json = mcp::protocol::Json;

struct OperationResult
{
  bool ok{false};
  std::string code;
  std::string message;
  Json data{Json::object()};

  static OperationResult success(Json value = Json::object(), std::string text = "")
  {
    return OperationResult{true, "ok", std::move(text), std::move(value)};
  }

  static OperationResult failure(
    std::string error_code,
    std::string text,
    Json value = Json::object())
  {
    return OperationResult{
      false, std::move(error_code), std::move(text), std::move(value)};
  }
};

}  // namespace zhishu_robot_mcp

#endif  // ZHISHU_ROBOT_MCP__RESULT_HPP_
