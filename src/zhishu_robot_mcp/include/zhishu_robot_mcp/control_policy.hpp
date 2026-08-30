#ifndef ZHISHU_ROBOT_MCP__CONTROL_POLICY_HPP_
#define ZHISHU_ROBOT_MCP__CONTROL_POLICY_HPP_

#include <chrono>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <string_view>

namespace zhishu_robot_mcp
{

enum class RobotMode
{
  kSimulation,
  kPhysical,
};

struct ServerOptions
{
  bool write_enabled{false};
  RobotMode robot_mode{RobotMode::kSimulation};
  bool allow_physical_hardware{false};
  std::size_t max_writes_per_minute{10U};
  double max_physical_joint_step_rad{0.35};

  static ServerOptions from_environment();
};

std::string robot_mode_name(RobotMode mode);

class ControlPolicy
{
public:
  explicit ControlPolicy(ServerOptions options);

  const ServerOptions & options() const noexcept;

  bool authorize(
    std::string_view required_confirmation,
    std::string_view supplied_confirmation,
    bool apply_rate_limit,
    std::string & reason);

private:
  void discard_expired_locked(std::chrono::steady_clock::time_point now);

  ServerOptions options_;
  std::mutex mutex_;
  std::deque<std::chrono::steady_clock::time_point> recent_writes_;
};

}  // namespace zhishu_robot_mcp

#endif  // ZHISHU_ROBOT_MCP__CONTROL_POLICY_HPP_
