#include "zhishu_robot_mcp/robot_bridge.hpp"

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <future>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <sys/wait.h>
#include <utility>

namespace zhishu_robot_mcp
{
namespace
{

using namespace std::chrono_literals;

constexpr char kMoveConfirmation[] = "MOVE ROBOT";
constexpr char kResetConfirmation[] = "RESET FAULT";
constexpr char kEnableConfirmation[] = "ENABLE MOTORS";
constexpr char kDisableConfirmation[] = "DISABLE MOTORS";

const std::array<std::string, 7> kLeftJointNames{
  "left_shoulder_pitch_joint",
  "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint",
  "left_elbow_pitch_joint",
  "left_wrist_yaw_joint",
  "left_wrist_pitch_joint",
  "left_wrist_roll_joint",
};

const std::array<std::string, 7> kRightJointNames{
  "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint",
  "right_elbow_pitch_joint",
  "right_wrist_yaw_joint",
  "right_wrist_pitch_joint",
  "right_wrist_roll_joint",
};

std::string format_time(std::chrono::system_clock::time_point point)
{
  if (point.time_since_epoch().count() == 0) {
    return "";
  }
  const auto value = std::chrono::system_clock::to_time_t(point);
  std::tm local{};
  localtime_r(&value, &local);
  std::ostringstream stream;
  stream << std::put_time(&local, "%Y-%m-%dT%H:%M:%S%z");
  return stream.str();
}

struct CommandResult
{
  int exit_code{-1};
  bool output_truncated{false};
  std::string output;
};

CommandResult run_fixed_command(const char * command)
{
  CommandResult result;
  FILE * pipe = popen(command, "r");
  if (pipe == nullptr) {
    result.output = "failed to start command";
    return result;
  }

  constexpr std::size_t kMaximumOutput = 128U * 1024U;
  std::array<char, 4096> buffer{};
  while (std::fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    const std::string chunk{buffer.data()};
    if (result.output.size() + chunk.size() <= kMaximumOutput) {
      result.output += chunk;
    } else {
      result.output_truncated = true;
    }
  }
  const int status = pclose(pipe);
  if (status >= 0 && WIFEXITED(status)) {
    result.exit_code = WEXITSTATUS(status);
  }
  return result;
}

bool is_active_motion_state(const std::string & state)
{
  return state == "queued" || state == "running" || state == "stop_requested";
}

std::vector<std::string> joint_names_for_group(const std::string & group)
{
  if (group == "left_arm") {
    return {kLeftJointNames.begin(), kLeftJointNames.end()};
  }
  if (group == "right_arm") {
    return {kRightJointNames.begin(), kRightJointNames.end()};
  }
  if (group == "both_arms") {
    std::vector<std::string> result{kLeftJointNames.begin(), kLeftJointNames.end()};
    result.insert(result.end(), kRightJointNames.begin(), kRightJointNames.end());
    return result;
  }
  return {};
}

}  // namespace

RobotBridge::RobotBridge(
  rclcpp::Node::SharedPtr node,
  ServerOptions options,
  std::vector<LogRoot> log_roots)
: node_(std::move(node)), policy_(std::move(options)), logs_(std::move(log_roots))
{
  if (!node_) {
    throw std::invalid_argument("RobotBridge requires a valid ROS 2 node");
  }
  joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::JointState::SharedPtr message) {
      joint_state_callback(std::move(message));
    });
}

RobotBridge::~RobotBridge()
{
  shutdown();
}

OperationResult RobotBridge::server_info() const
{
  const auto & options = policy_.options();
  const bool writes_effective = options.write_enabled &&
    (options.robot_mode == RobotMode::kSimulation || options.allow_physical_hardware);
  Json motion;
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    motion = motion_status_locked();
  }
  return OperationResult::success(Json{
    {"server", "zhishu-robot-mcp"},
    {"version", "0.1.0"},
    {"transport", "stdio"},
    {"robot_mode", robot_mode_name(options.robot_mode)},
    {"write_enabled", options.write_enabled},
    {"allow_physical_hardware", options.allow_physical_hardware},
    {"writes_effective", writes_effective},
    {"max_writes_per_minute", options.max_writes_per_minute},
    {"max_physical_joint_step_rad", options.max_physical_joint_step_rad},
    {"log_roots", logs_.roots_summary()},
    {"motion", motion},
  });
}

OperationResult RobotBridge::robot_status()
{
  Json joints = Json::object();
  double joint_state_age_seconds = -1.0;
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    for (const auto & item : joint_positions_) {
      joints[item.first] = item.second;
    }
    if (last_joint_state_at_.time_since_epoch().count() != 0) {
      joint_state_age_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_joint_state_at_).count();
    }
  }

  Json motion;
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    motion = motion_status_locked();
  }

  return OperationResult::success(Json{
    {"ros_ok", rclcpp::ok()},
    {"node_count", node_->get_node_names().size()},
    {"topic_count", node_->get_topic_names_and_types().size()},
    {"controllers", controller_status()},
    {"joint_state_received", joint_state_age_seconds >= 0.0},
    {"joint_state_age_seconds",
      joint_state_age_seconds >= 0.0 ? Json(joint_state_age_seconds) : Json(nullptr)},
    {"joint_positions_rad", joints},
    {"motion", motion},
  });
}

OperationResult RobotBridge::ethercat_status() const
{
  const auto masters = run_fixed_command("timeout --signal=KILL 3s ethercat master 2>&1");
  const auto slaves = run_fixed_command("timeout --signal=KILL 3s ethercat slaves 2>&1");
  const Json data{
    {"master", Json{
      {"exit_code", masters.exit_code},
      {"output", masters.output},
      {"truncated", masters.output_truncated}}},
    {"slaves", Json{
      {"exit_code", slaves.exit_code},
      {"output", slaves.output},
      {"truncated", slaves.output_truncated}}},
  };
  if (masters.exit_code != 0 || slaves.exit_code != 0) {
    return OperationResult::failure(
      "ethercat_query_failed", "one or more fixed EtherCAT queries failed", data);
  }
  return OperationResult::success(data);
}

OperationResult RobotBridge::list_ros_nodes() const
{
  auto names = node_->get_node_names();
  std::sort(names.begin(), names.end());
  names.erase(std::unique(names.begin(), names.end()), names.end());
  return OperationResult::success(Json{{"nodes", names}, {"count", names.size()}});
}

OperationResult RobotBridge::list_ros_topics(
  const std::string & filter,
  std::size_t limit) const
{
  limit = std::clamp<std::size_t>(limit, 1U, 500U);
  Json topics = Json::array();
  for (const auto & item : node_->get_topic_names_and_types()) {
    if (!filter.empty() && item.first.find(filter) == std::string::npos) {
      continue;
    }
    topics.push_back(Json{{"name", item.first}, {"types", item.second}});
  }
  std::sort(
    topics.begin(), topics.end(),
    [](const Json & left, const Json & right) {
      return left.at("name").get<std::string>() < right.at("name").get<std::string>();
    });
  const auto total = topics.size();
  if (topics.size() > limit) {
    topics.erase(topics.begin() + static_cast<std::ptrdiff_t>(limit), topics.end());
  }
  return OperationResult::success(
    Json{{"topics", topics}, {"returned", topics.size()}, {"total", total}});
}

OperationResult RobotBridge::list_recent_logs(std::size_t limit) const
{
  return logs_.list_recent(limit);
}

OperationResult RobotBridge::read_log(
  const std::string & root_name,
  const std::string & relative_path,
  std::size_t tail_lines) const
{
  return logs_.read_tail(root_name, relative_path, tail_lines);
}

OperationResult RobotBridge::move_named(
  const std::string & group,
  const std::string & target,
  const std::string & confirmation)
{
  static const std::map<std::string, std::set<std::string>> allowed_targets{
    {"left_arm", {"left_arm_home", "left_arm_pose1", "left_arm_crate_pre"}},
    {"right_arm", {"right_arm_home", "right_arm_pose1", "right_arm_crate_pre"}},
    {"both_arms", {
      "both_arms_home", "both_arms_pose1", "both_arms_crate_safe", "both_arms_crate_pre"}},
  };
  const auto group_iterator = allowed_targets.find(group);
  if (group_iterator == allowed_targets.end() || group_iterator->second.count(target) == 0U) {
    return OperationResult::failure(
      "target_not_allowed", "group and named target are not in the MCP allowlist");
  }
  const auto authorization = authorize_write(kMoveConfirmation, confirmation);
  if (!authorization.ok) {
    return authorization;
  }

  return start_motion_job(
    "move_named:" + group + ":" + target,
    [group, target](Commander & command) {
      if (group == "left_arm") {
        return command.moveLeftArmNamed(target);
      }
      if (group == "right_arm") {
        return command.moveRightArmNamed(target);
      }
      return command.moveBothArmsNamed(target);
    });
}

OperationResult RobotBridge::move_joints(
  const std::string & group,
  const std::vector<double> & positions,
  const std::string & confirmation)
{
  const auto names = joint_names_for_group(group);
  if (names.empty()) {
    return OperationResult::failure("invalid_group", "group must be left_arm, right_arm or both_arms");
  }
  if (positions.size() != names.size() ||
    !std::all_of(positions.begin(), positions.end(), [](double value) {return std::isfinite(value);}))
  {
    return OperationResult::failure(
      "invalid_joint_target", "joint array has the wrong size or contains a non-finite value");
  }
  const auto authorization = authorize_write(kMoveConfirmation, confirmation);
  if (!authorization.ok) {
    return authorization;
  }
  std::string safety_reason;
  if (!physical_joint_step_is_safe(group, positions, safety_reason)) {
    return OperationResult::failure("joint_step_denied", safety_reason);
  }

  return start_motion_job(
    "move_joints:" + group,
    [group, positions](Commander & command) {
      if (group == "left_arm") {
        return command.moveLeftArmJoints(positions);
      }
      if (group == "right_arm") {
        return command.moveRightArmJoints(positions);
      }
      return command.moveBothArmsJoints(positions);
    });
}

OperationResult RobotBridge::set_gripper(
  const std::string & side,
  double position,
  const std::string & confirmation)
{
  if ((side != "left" && side != "right" && side != "both") ||
    !std::isfinite(position) || position < 0.0 || position > 0.06)
  {
    return OperationResult::failure(
      "invalid_gripper_target", "side must be left, right or both and position must be in [0, 0.06] m");
  }
  const auto authorization = authorize_write(kMoveConfirmation, confirmation);
  if (!authorization.ok) {
    return authorization;
  }
  return start_motion_job(
    "set_gripper:" + side,
    [side, position](Commander & command) {
      if (side == "left") {
        return command.moveLeftGripper(position);
      }
      if (side == "right") {
        return command.moveRightGripper(position);
      }
      return command.moveBothGrippers(position, position);
    });
}

OperationResult RobotBridge::motion_status() const
{
  std::lock_guard<std::mutex> lock(motion_mutex_);
  return OperationResult::success(motion_status_locked());
}

OperationResult RobotBridge::stop_motion()
{
  const auto authorization = authorize_write("", "", false);
  if (!authorization.ok) {
    return authorization;
  }

  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (is_active_motion_state(motion_job_.state)) {
      motion_job_.state = "stop_requested";
      motion_job_.message = "MoveIt stop requested; this is not a hardware emergency stop";
    }
  }

  std::shared_ptr<Commander> active_commander;
  {
    std::lock_guard<std::mutex> lock(commander_mutex_);
    active_commander = commander_;
  }
  if (active_commander) {
    active_commander->stop();
  }

  std::lock_guard<std::mutex> lock(motion_mutex_);
  return OperationResult::success(motion_status_locked(), "stop request processed");
}

OperationResult RobotBridge::reset_fault(
  const std::string & arm,
  const std::string & confirmation)
{
  if (policy_.options().robot_mode != RobotMode::kPhysical) {
    return OperationResult::failure("invalid_mode", "fault reset is only available in physical mode");
  }
  if (arm != "left" && arm != "right" && arm != "both") {
    return OperationResult::failure("invalid_arm", "arm must be left, right or both");
  }
  const auto authorization = authorize_write(kResetConfirmation, confirmation);
  if (!authorization.ok) {
    return authorization;
  }

  Json results = Json::array();
  const std::vector<std::string> arms = arm == "both" ?
    std::vector<std::string>{"left", "right"} : std::vector<std::string>{arm};
  for (const auto & selected : arms) {
    const auto result = call_reset_service(selected);
    results.push_back(Json{
      {"arm", selected}, {"ok", result.ok}, {"code", result.code}, {"message", result.message}});
    if (!result.ok) {
      return OperationResult::failure(
        "fault_reset_failed", "fault reset stopped after a service failure", Json{{"results", results}});
    }
  }
  return OperationResult::success(
    Json{{"results", results}, {"motors_enabled", false}},
    "fault reset completed; motors were not enabled");
}

OperationResult RobotBridge::set_motors_enabled(
  const std::string & arm,
  bool enabled,
  const std::string & confirmation)
{
  if (policy_.options().robot_mode != RobotMode::kPhysical) {
    return OperationResult::failure("invalid_mode", "motor enable is only available in physical mode");
  }
  if (arm != "left" && arm != "right" && arm != "both") {
    return OperationResult::failure("invalid_arm", "arm must be left, right or both");
  }
  const auto authorization = authorize_write(
    enabled ? kEnableConfirmation : kDisableConfirmation,
    confirmation,
    enabled);
  if (!authorization.ok) {
    return authorization;
  }

  const std::vector<std::string> arms = arm == "both" ?
    std::vector<std::string>{"left", "right"} : std::vector<std::string>{arm};
  Json results = Json::array();
  std::vector<std::string> enabled_arms;
  bool all_succeeded = true;
  for (const auto & selected : arms) {
    const auto result = call_enable_service(selected, enabled);
    results.push_back(Json{
      {"arm", selected}, {"ok", result.ok}, {"code", result.code}, {"message", result.message}});
    if (result.ok && enabled) {
      enabled_arms.push_back(selected);
    }
    all_succeeded = all_succeeded && result.ok;
    if (!result.ok && enabled) {
      Json rollback = Json::array();
      for (const auto & rollback_arm : enabled_arms) {
        const auto rollback_result = call_enable_service(rollback_arm, false);
        rollback.push_back(Json{{"arm", rollback_arm}, {"ok", rollback_result.ok}});
      }
      return OperationResult::failure(
        "motor_enable_failed", "motor enable failed and rollback was requested",
        Json{{"results", results}, {"rollback", rollback}});
    }
  }
  if (!all_succeeded) {
    return OperationResult::failure(
      "motor_disable_failed", "one or more arms could not be disabled",
      Json{{"results", results}});
  }
  return OperationResult::success(Json{{"enabled", enabled}, {"results", results}});
}

void RobotBridge::shutdown()
{
  if (shutting_down_.exchange(true)) {
    return;
  }
  std::shared_ptr<Commander> active_commander;
  {
    std::lock_guard<std::mutex> lock(commander_mutex_);
    active_commander = commander_;
  }
  if (active_commander) {
    active_commander->stop();
  }
  if (motion_thread_.joinable()) {
    motion_thread_.join();
  }
}

OperationResult RobotBridge::authorize_write(
  const std::string & required_confirmation,
  const std::string & supplied_confirmation,
  bool rate_limited)
{
  std::string reason;
  if (!policy_.authorize(required_confirmation, supplied_confirmation, rate_limited, reason)) {
    return OperationResult::failure("write_denied", std::move(reason));
  }
  return OperationResult::success();
}

OperationResult RobotBridge::start_motion_job(
  const std::string & command,
  std::function<bool(Commander &)> operation)
{
  std::lock_guard<std::mutex> start_lock(motion_start_mutex_);
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (is_active_motion_state(motion_job_.state)) {
      return OperationResult::failure(
        "motion_busy", "another motion is already active", motion_status_locked());
    }
  }
  if (motion_thread_.joinable()) {
    motion_thread_.join();
  }

  const auto sequence = ++motion_sequence_;
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    motion_job_ = MotionJob{};
    motion_job_.id = "motion-" + std::to_string(sequence);
    motion_job_.command = command;
    motion_job_.state = "queued";
    motion_job_.message = "accepted";
    motion_job_.started_at = std::chrono::system_clock::now();
  }

  motion_thread_ = std::thread([this, operation = std::move(operation)]() mutable {
      {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        if (motion_job_.state == "stop_requested" || shutting_down_.load()) {
          motion_job_.state = "stopped";
          motion_job_.message = "motion stopped before execution began";
          motion_job_.finished_at = std::chrono::system_clock::now();
          return;
        }
        motion_job_.state = "running";
        motion_job_.message = "MoveIt planning/execution in progress";
      }
      bool completed = false;
      std::string error_message;
      try {
        if (!shutting_down_.load()) {
          auto active_commander = commander();
          bool stopped_before_execution = false;
          {
            std::lock_guard<std::mutex> lock(motion_mutex_);
            stopped_before_execution = motion_job_.state == "stop_requested";
          }
          if (stopped_before_execution) {
            active_commander->stop();
            error_message = "motion stopped before execution began";
          } else {
            completed = operation(*active_commander);
          }
        } else {
          error_message = "server is shutting down";
        }
      } catch (const std::exception & error) {
        error_message = error.what();
      }

      std::lock_guard<std::mutex> lock(motion_mutex_);
      if (motion_job_.state == "stop_requested" || shutting_down_.load()) {
        motion_job_.state = "stopped";
        motion_job_.message = error_message.empty() ? "motion stopped" : error_message;
      } else if (completed) {
        motion_job_.state = "succeeded";
        motion_job_.message = "motion completed";
      } else {
        motion_job_.state = "failed";
        motion_job_.message = error_message.empty() ? "MoveIt returned failure" : error_message;
      }
      motion_job_.finished_at = std::chrono::system_clock::now();
    });

  std::lock_guard<std::mutex> lock(motion_mutex_);
  return OperationResult::success(motion_status_locked(), "motion accepted asynchronously");
}

std::shared_ptr<RobotBridge::Commander> RobotBridge::commander()
{
  std::lock_guard<std::mutex> lock(commander_mutex_);
  if (!commander_) {
    Commander::Options options;
    const bool physical = policy_.options().robot_mode == RobotMode::kPhysical;
    options.velocity_scaling = physical ? 0.10 : 0.20;
    options.acceleration_scaling = physical ? 0.10 : 0.20;
    options.planning_time = 5.0;
    options.planning_attempts = 5;
    commander_ = std::make_shared<Commander>(node_, options);
  }
  return commander_;
}

Json RobotBridge::motion_status_locked() const
{
  return Json{
    {"id", motion_job_.id},
    {"command", motion_job_.command},
    {"state", motion_job_.state},
    {"message", motion_job_.message},
    {"started_at", format_time(motion_job_.started_at)},
    {"finished_at", format_time(motion_job_.finished_at)},
  };
}

bool RobotBridge::physical_joint_step_is_safe(
  const std::string & group,
  const std::vector<double> & positions,
  std::string & reason) const
{
  if (policy_.options().robot_mode != RobotMode::kPhysical) {
    return true;
  }
  const auto names = joint_names_for_group(group);
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  if (last_joint_state_at_.time_since_epoch().count() == 0 ||
    std::chrono::steady_clock::now() - last_joint_state_at_ > std::chrono::seconds(1))
  {
    reason = "physical joint motion requires a /joint_states sample newer than one second";
    return false;
  }
  for (std::size_t index = 0; index < names.size(); ++index) {
    const auto current = joint_positions_.find(names[index]);
    if (current == joint_positions_.end()) {
      reason = "missing current position for " + names[index];
      return false;
    }
    const double step = std::abs(positions[index] - current->second);
    if (step > policy_.options().max_physical_joint_step_rad) {
      std::ostringstream stream;
      stream << names[index] << " step " << step << " rad exceeds limit " <<
        policy_.options().max_physical_joint_step_rad << " rad";
      reason = stream.str();
      return false;
    }
  }
  return true;
}

OperationResult RobotBridge::call_reset_service(const std::string & arm)
{
  const auto service_name = "/" + arm + "_arm/reset_fault";
  auto client = node_->create_client<std_srvs::srv::Trigger>(service_name);
  if (!client->wait_for_service(2s)) {
    return OperationResult::failure("service_unavailable", service_name + " is unavailable");
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready) {
    return OperationResult::failure("service_timeout", service_name + " timed out");
  }
  const auto response = future.get();
  return response->success ? OperationResult::success(Json::object(), response->message) :
         OperationResult::failure("service_rejected", response->message);
}

OperationResult RobotBridge::call_enable_service(const std::string & arm, bool enabled)
{
  const auto service_name = "/" + arm + "_arm/enable";
  auto client = node_->create_client<std_srvs::srv::SetBool>(service_name);
  if (!client->wait_for_service(2s)) {
    return OperationResult::failure("service_unavailable", service_name + " is unavailable");
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  auto future = client->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready) {
    return OperationResult::failure("service_timeout", service_name + " timed out");
  }
  const auto response = future.get();
  return response->success ? OperationResult::success(Json::object(), response->message) :
         OperationResult::failure("service_rejected", response->message);
}

Json RobotBridge::controller_status()
{
  const std::string service_name = "/controller_manager/list_controllers";
  auto client = node_->create_client<controller_manager_msgs::srv::ListControllers>(service_name);
  if (!client->wait_for_service(500ms)) {
    return Json{{"available", false}, {"controllers", Json::array()}};
  }
  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto future = client->async_send_request(request);
  if (future.wait_for(2s) != std::future_status::ready) {
    return Json{{"available", false}, {"error", "timeout"}, {"controllers", Json::array()}};
  }
  Json controllers = Json::array();
  for (const auto & controller : future.get()->controller) {
    controllers.push_back(Json{
      {"name", controller.name}, {"type", controller.type}, {"state", controller.state}});
  }
  return Json{{"available", true}, {"controllers", controllers}};
}

void RobotBridge::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  const auto count = std::min(message->name.size(), message->position.size());
  for (std::size_t index = 0; index < count; ++index) {
    if (std::isfinite(message->position[index])) {
      joint_positions_[message->name[index]] = message->position[index];
    }
  }
  last_joint_state_at_ = std::chrono::steady_clock::now();
}

}  // namespace zhishu_robot_mcp
