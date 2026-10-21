#ifndef ZHISHU_ROBOT_SDK__ROBOT_IO_HPP_
#define ZHISHU_ROBOT_SDK__ROBOT_IO_HPP_

#include <cstdint>

namespace zhishu_robot_sdk
{

enum class IOModule
{
  kControlBox,
  kTool,
  kAuxiliary,
  kFieldbus,
};

enum class IOType
{
  kDigital,
  kAnalog,
};

enum class IODirection
{
  kInput,
  kOutput,
};

/// Normalized IO address. Channel numbering/mapping is backend-specific and must be documented
/// by the hardware adapter before these placeholder APIs are implemented.
struct IOAddress
{
  IOModule module{IOModule::kControlBox};
  IOType type{IOType::kDigital};
  IODirection direction{IODirection::kInput};
  std::uint32_t channel{0};
};

struct IOValue
{
  IOAddress address{};
  double value{0.0};
  bool valid{false};
};

}  // namespace zhishu_robot_sdk

#endif  // ZHISHU_ROBOT_SDK__ROBOT_IO_HPP_
