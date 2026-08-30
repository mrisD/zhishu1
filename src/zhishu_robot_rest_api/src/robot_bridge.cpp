#include "zhishu_robot_rest_api/robot_bridge.hpp"

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <ctime>
#include <future>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <sys/wait.h>
#include <utility>

namespace zhishu_robot_rest_api
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
  std::tm utc{};
  gmtime_r(&value, &utc);
  std::ostringstream stream;
  stream << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
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
  RestOptions options,
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

const AccessPolicy & RobotBridge::policy() const noexcept
{
  return policy_;
}

ApiResult RobotBridge::server_info() const
{
  const auto & options = policy_.options();
  Json motion;
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    motion = motion_status_locked();
  }
  return ApiResult::success(Json{
    {"server", "zhishu-robot-rest-api"},
    {"version", "0.1.0"},
    {"api_version", "v1"},
    {"bind_host", options.host},
    {"port", options.port},
    {"remote_access_enabled", options.allow_remote},
    {"read_auth_required", options.read_auth_required()},
    {"api_token_configured", !options.api_token.empty()},
    {"robot_mode", robot_mode_name(options.robot_mode)},
    {"write_enabled", options.write_enabled},
    {"allow_physical_hardware", options.allow_physical_hardware},
    {"max_writes_per_minute", options.max_writes_per_minute},
    {"max_physical_joint_step_rad", options.max_physical_joint_step_rad},
    {"log_roots", logs_.roots_summary()},
    {"motion", motion},
  });
}

ApiResult RobotBridge::robot_status()
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
  return ApiResult::success(Json{
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

ApiResult RobotBridge::ethercat_status() const
{
  std::lock_guard<std::mutex> command_lock(ethercat_mutex_);
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
    return ApiResult::failure(
      "ethercat_query_failed", "one or more fixed EtherCAT queries failed", 503, data);
  }
  return ApiResult::success(data);
}

ApiResult RobotBridge::list_ros_nodes() const
{
  auto names = node_->get_node_names();
  std::sort(names.begin(), names.end());
  names.erase(std::unique(names.begin(), names.end()), names.end());
  return ApiResult::success(Json{{"nodes", names}, {"count", names.size()}});
}

ApiResult RobotBridge::list_ros_topics(
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
  return ApiResult::success(
    Json{{"topics", topics}, {"returned", topics.size()}, {"total", total}});
}

ApiResult RobotBridge::list_recent_logs(std::size_t limit) const
{
  return logs_.list_recent(limit);
}

ApiResult RobotBridge::read_log(
  const std::string & root_name,
  const std::string & relative_path,
  std::size_t tail_lines) const
{
  return logs_.read_tail(root_name, relative_path, tail_lines);
}

ApiResult RobotBridge::move_named(
  const std::string & group,
  const std::string & target,
  const std::string & confirmation,
  const std::string & bearer_token)
{
  static const std::map<std::string, std::set<std::string>> allowed_targets{
    {"left_arm", {"left_arm_home", "left_arm_pose1", "left_arm_crate_pre"}},
    {"right_arm", {"right_arm_home", "right_arm_pose1", "right_arm_crate_pre"}},
    {"both_arms", {
      "both_arms_home", "both_arms_pose1", "both_arms_crate_safe", "both_arms_crate_pre"}},
  };
  const auto iterator = allowed_targets.find(group);
  if (iterator == allowed_targets.end() || iterator->second.count(target) == 0U) {
    return ApiResult::failure(
      "target_not_allowed", "group and named target are not in the REST allowlist", 400);
  }
  const auto authorization = authorize_write(
    bearer_token, kMoveConfirmation, confirmation);
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

ApiResult RobotBridge::move_joints(
  const std::string & group,
  const std::vector<double> & positions,
  const std::string & confirmation,
  const std::string & bearer_token)
{
  const auto names = joint_names_for_group(group);
  if (names.empty()) {
    return ApiResult::failure(
      "invalid_group", "group must be left_arm, right_arm or both_arms", 400);
  }
  if (positions.size() != names.size() ||
    !std::all_of(positions.begin(), positions.end(), [](double value) {return std::isfinite(value);}))
  {
    return ApiResult::failure(
      "invalid_joint_target", "joint array has wrong size or a non-finite value", 400);
  }
  const auto authorization = authorize_write(
    bearer_token, kMoveConfirmation, confirmation);
  if (!authorization.ok) {
    return authorization;
  }
  std::string safety_reason;
  if (!physical_joint_step_is_safe(group, positions, safety_reason)) {
    return ApiResult::failure("joint_step_denied", safety_reason, 403);
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

ApiResult RobotBridge::set_gripper(
  const std::string & side,
  double position,
  const std::string & confirmation,
  const std::string & bearer_token)
{
  if ((side != "left" && side != "right" && side != "both") ||
    !std::isfinite(position) || position < 0.0 || position > 0.06)
  {
    return ApiResult::failure(
      "invalid_gripper_target", "side must be left, right or both; position must be [0, 0.06] m",
      400);
  }
  const auto authorization = authorize_write(
    bearer_token, kMoveConfirmation, confirmation);
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

ApiResult RobotBridge::motion_status() const
{
  std::lock_guard<std::mutex> lock(motion_mutex_);
  return ApiResult::success(motion_status_locked());
}

ApiResult RobotBridge::stop_motion(const std::string & bearer_token)
{
  const auto authorization = authorize_write(bearer_token, "", "", false);
  if (!authorization.ok) {
    return authorization;
  }
  std::lock_guard<std::mutex> motion_gate(motion_start_mutex_);
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
  return ApiResult::success(motion_status_locked(), "stop request processed");
}

ApiResult RobotBridge::reset_fault(
  const std::string & arm,
  const std::string & confirmation,
  const std::string & bearer_token)
{
  if (policy_.options().robot_mode != RobotMode::kPhysical) {
    return ApiResult::failure("invalid_mode", "fault reset is physical mode only", 400);
  }
  if (arm != "left" && arm != "right" && arm != "both") {
    return ApiResult::failure("invalid_arm", "arm must be left, right or both", 400);
  }
  std::lock_guard<std::mutex> motion_gate(motion_start_mutex_);
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (is_active_motion_state(motion_job_.state)) {
      return ApiResult::failure(
        "motion_busy", "stop the active motion before resetting faults", 409,
        motion_status_locked());
    }
  }
  const auto authorization = authorize_write(
    bearer_token, kResetConfirmation, confirmation);
  if (!authorization.ok) {
    return authorization;
  }

  std::lock_guard<std::mutex> recovery_lock(recovery_mutex_);
  Json results = Json::array();
  const std::vector<std::string> arms = arm == "both" ?
    std::vector<std::string>{"left", "right"} : std::vector<std::string>{arm};
  for (const auto & selected : arms) {
    const auto result = call_reset_service(selected);
    results.push_back(Json{
      {"arm", selected}, {"ok", result.ok}, {"code", result.code}, {"message", result.message}});
    if (!result.ok) {
      return ApiResult::failure(
        "fault_reset_failed", "fault reset stopped after a service failure", result.http_status,
        Json{{"results", results}});
    }
  }
  return ApiResult::success(
    Json{{"results", results}, {"motors_enabled", false}},
    "fault reset completed; motors were not enabled");
}

ApiResult RobotBridge::set_motors_enabled(
  const std::string & arm,
  bool enabled,
  const std::string & confirmation,
  const std::string & bearer_token)
{
  if (policy_.options().robot_mode != RobotMode::kPhysical) {
    return ApiResult::failure("invalid_mode", "motor enable is physical mode only", 400);
  }
  if (arm != "left" && arm != "right" && arm != "both") {
    return ApiResult::failure("invalid_arm", "arm must be left, right or both", 400);
  }
  std::lock_guard<std::mutex> motion_gate(motion_start_mutex_);
  if (enabled) {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (is_active_motion_state(motion_job_.state)) {
      return ApiResult::failure(
        "motion_busy", "motors cannot be enabled while a REST motion is active", 409,
        motion_status_locked());
    }
  }
  const auto authorization = authorize_write(
    bearer_token, enabled ? kEnableConfirmation : kDisableConfirmation,
    confirmation, enabled);
  if (!authorization.ok) {
    return authorization;
  }

  if (!enabled) {
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      if (is_active_motion_state(motion_job_.state)) {
        motion_job_.state = "stop_requested";
        motion_job_.message = "motor disable requested; MoveIt stop also requested";
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
  }

  std::lock_guard<std::mutex> recovery_lock(recovery_mutex_);
  const std::vector<std::string> arms = arm == "both" ?
    std::vector<std::string>{"left", "right"} : std::vector<std::string>{arm};
  Json results = Json::array();
  std::vector<std::string> enabled_arms;
  bool all_succeeded = true;
  for (const auto & selected : arms) {
    const auto result = call_enable_service(selected, enabled);
    results.push_back(Json{
      {"arm", selected}, {"ok", result.ok}, {"code", result.code}, {"message", result.message}});
    all_succeeded = all_succeeded && result.ok;
    if (result.ok && enabled) {
      enabled_arms.push_back(selected);
    }
    if (!result.ok && enabled) {
      Json rollback = Json::array();
      for (const auto & rollback_arm : enabled_arms) {
        const auto rollback_result = call_enable_service(rollback_arm, false);
        rollback.push_back(Json{{"arm", rollback_arm}, {"ok", rollback_result.ok}});
      }
      return ApiResult::failure(
        "motor_enable_failed", "motor enable failed and rollback was requested",
        result.http_status, Json{{"results", results}, {"rollback", rollback}});
    }
  }
  if (!all_succeeded) {
    return ApiResult::failure(
      "motor_disable_failed", "one or more arms could not be disabled", 503,
      Json{{"results", results}});
  }
  return ApiResult::success(Json{{"enabled", enabled}, {"results", results}});
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

ApiResult RobotBridge::authorize_write(
  const std::string & bearer_token,
  const std::string & required_confirmation,
  const std::string & supplied_confirmation,
  bool rate_limited)
{
  std::string reason;
  if (!policy_.authorize_write(
      bearer_token, required_confirmation, supplied_confirmation, rate_limited, reason))
  {
    int status = 403;
    std::string code = "write_denied";
    if (reason.find("bearer token") != std::string::npos) {
      status = 401;
      code = "unauthorized";
    } else if (reason.find("rate limit") != std::string::npos) {
      status = 429;
      code = "rate_limited";
    }
    return ApiResult::failure(code, std::move(reason), status);
  }
  return ApiResult::success();
}

ApiResult RobotBridge::start_motion_job(
  const std::string & command,
  std::function<bool(Commander &)> operation)
{
  std::lock_guard<std::mutex> start_lock(motion_start_mutex_);
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (is_active_motion_state(motion_job_.state)) {
      return ApiResult::failure(
        "motion_busy", "another REST motion is already active", 409, motion_status_locked());
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
  return ApiResult::success(
    motion_status_locked(), "motion accepted asynchronously", 202);
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

ApiResult RobotBridge::call_reset_service(const std::string & arm)
{
  const auto service_name = "/" + arm + "_arm/reset_fault";
  auto client = node_->create_client<std_srvs::srv::Trigger>(service_name);
  if (!client->wait_for_service(2s)) {
    return ApiResult::failure("service_unavailable", service_name + " is unavailable", 503);
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready) {
    return ApiResult::failure("service_timeout", service_name + " timed out", 504);
  }
  const auto response = future.get();
  return response->success ? ApiResult::success(Json::object(), response->message) :
         ApiResult::failure("service_rejected", response->message, 409);
}

ApiResult RobotBridge::call_enable_service(const std::string & arm, bool enabled)
{
  const auto service_name = "/" + arm + "_arm/enable";
  auto client = node_->create_client<std_srvs::srv::SetBool>(service_name);
  if (!client->wait_for_service(2s)) {
    return ApiResult::failure("service_unavailable", service_name + " is unavailable", 503);
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  auto future = client->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready) {
    return ApiResult::failure("service_timeout", service_name + " timed out", 504);
  }
  const auto response = future.get();
  return response->success ? ApiResult::success(Json::object(), response->message) :
         ApiResult::failure("service_rejected", response->message, 409);
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

}  // namespace zhishu_robot_rest_api
