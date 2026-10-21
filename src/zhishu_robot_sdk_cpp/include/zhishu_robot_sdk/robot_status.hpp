#ifndef ZHISHU_ROBOT_SDK__ROBOT_STATUS_HPP_
#define ZHISHU_ROBOT_SDK__ROBOT_STATUS_HPP_

#include <string>

namespace zhishu_robot_sdk
{

/// Controller/firmware version information. Fields remain empty until a backend is connected.
struct RobotVersionInfo
{
  std::string sdk_version;
  std::string controller_version;
  std::string firmware_version;
  std::string message_schema_version;
};

/// Normalized error information independent of a vendor-specific controller protocol.
struct RobotErrorInfo
{
  bool active{false};
  int main_code{0};
  int sub_code{0};
  std::string message;
};

}  // namespace zhishu_robot_sdk

#endif  // ZHISHU_ROBOT_SDK__ROBOT_STATUS_HPP_
