#ifndef ZHISHU_ROBOT_MCP__ROBOT_BRIDGE_HPP_
#define ZHISHU_ROBOT_MCP__ROBOT_BRIDGE_HPP_

#include "zhishu_robot_mcp/control_policy.hpp"
#include "zhishu_robot_mcp/log_access.hpp"
#include "zhishu_robot_mcp/result.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <zhishu_robot_commander_cpp/commander.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace zhishu_robot_mcp
{

class RobotBridge
{
public:
  RobotBridge(
    rclcpp::Node::SharedPtr node,
    ServerOptions options,
    std::vector<LogRoot> log_roots);
  ~RobotBridge();

  RobotBridge(const RobotBridge &) = delete;
  RobotBridge & operator=(const RobotBridge &) = delete;

  OperationResult server_info() const;
  OperationResult robot_status();
  OperationResult ethercat_status() const;
  OperationResult list_ros_nodes() const;
  OperationResult list_ros_topics(const std::string & filter, std::size_t limit) const;
  OperationResult list_recent_logs(std::size_t limit) const;
  OperationResult read_log(
    const std::string & root_name,
    const std::string & relative_path,
    std::size_t tail_lines) const;

  OperationResult move_named(
    const std::string & group,
    const std::string & target,
    const std::string & confirmation);
  OperationResult move_joints(
    const std::string & group,
    const std::vector<double> & positions,
    const std::string & confirmation);
  OperationResult set_gripper(
    const std::string & side,
    double position,
    const std::string & confirmation);
  OperationResult motion_status() const;
  OperationResult stop_motion();
  OperationResult reset_fault(
    const std::string & arm,
    const std::string & confirmation);
  OperationResult set_motors_enabled(
    const std::string & arm,
    bool enabled,
    const std::string & confirmation);

  void shutdown();

private:
  using Commander = zhishu_robot_commander_cpp::Commander;

  struct MotionJob
  {
    std::string id;
    std::string command;
    std::string state{"idle"};
    std::string message;
    std::chrono::system_clock::time_point started_at{};
    std::chrono::system_clock::time_point finished_at{};
  };

  OperationResult authorize_write(
    const std::string & required_confirmation,
    const std::string & supplied_confirmation,
    bool rate_limited = true);
  OperationResult start_motion_job(
    const std::string & command,
    std::function<bool(Commander &)> operation);
  std::shared_ptr<Commander> commander();
  Json motion_status_locked() const;
  bool physical_joint_step_is_safe(
    const std::string & group,
    const std::vector<double> & positions,
    std::string & reason) const;

  OperationResult call_reset_service(const std::string & arm);
  OperationResult call_enable_service(const std::string & arm, bool enabled);
  Json controller_status();
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message);

  rclcpp::Node::SharedPtr node_;
  ControlPolicy policy_;
  LogAccess logs_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

  mutable std::mutex joint_state_mutex_;
  std::map<std::string, double> joint_positions_;
  std::chrono::steady_clock::time_point last_joint_state_at_{};

  mutable std::mutex commander_mutex_;
  std::shared_ptr<Commander> commander_;

  mutable std::mutex motion_mutex_;
  std::mutex motion_start_mutex_;
  MotionJob motion_job_;
  std::thread motion_thread_;
  std::atomic_uint64_t motion_sequence_{0U};
  std::atomic_bool shutting_down_{false};
};

}  // namespace zhishu_robot_mcp

#endif  // ZHISHU_ROBOT_MCP__ROBOT_BRIDGE_HPP_
