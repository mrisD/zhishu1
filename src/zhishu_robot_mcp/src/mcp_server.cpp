#include "zhishu_robot_mcp/control_policy.hpp"
#include "zhishu_robot_mcp/robot_bridge.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <cxxmcp/peer.hpp>
#include <cxxmcp/run.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace zhishu_robot_mcp
{
namespace
{

using ToolFunction = std::function<OperationResult(const Json &)>;

std::string environment_value(const char * name)
{
  const char * value = std::getenv(name);
  return value == nullptr ? std::string{} : std::string{value};
}

std::string timestamp_now()
{
  const auto now = std::chrono::system_clock::now();
  const auto value = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&value, &local);
  std::ostringstream stream;
  stream << std::put_time(&local, "%Y-%m-%dT%H:%M:%S%z");
  return stream.str();
}

std::filesystem::path workspace_path()
{
  const auto configured = environment_value("ZHISHU_MCP_WORKSPACE");
  if (!configured.empty()) {
    return std::filesystem::path{configured}.lexically_normal();
  }

  auto current = std::filesystem::path{
    ament_index_cpp::get_package_prefix("zhishu_robot_mcp")}.lexically_normal();
  while (!current.empty()) {
    if (current.filename() == "install") {
      return current.parent_path();
    }
    const auto parent = current.parent_path();
    if (parent == current) {
      break;
    }
    current = parent;
  }
  return std::filesystem::current_path();
}

std::vector<LogRoot> log_roots(const std::filesystem::path & workspace)
{
  const auto configured_robot_logs = environment_value("ZHISHU_ROBOT_LOG_DIR");
  return {
    LogRoot{
      "robot",
      configured_robot_logs.empty() ? workspace / "zhishu_robot_logs" :
      std::filesystem::path{configured_robot_logs}},
    LogRoot{"stability", workspace / "log" / "ethercat_stability"},
  };
}

Json empty_object_schema()
{
  return Json{
    {"type", "object"},
    {"properties", Json::object()},
    {"additionalProperties", false},
  };
}

Json output_envelope_schema()
{
  return Json{
    {"type", "object"},
    {"required", Json::array({"ok", "code", "message", "data", "request_id", "timestamp"})},
    {"properties", Json{
      {"ok", Json{{"type", "boolean"}}},
      {"code", Json{{"type", "string"}}},
      {"message", Json{{"type", "string"}}},
      {"data", Json::object()},
      {"request_id", Json{{"type", "string"}}},
      {"timestamp", Json{{"type", "string"}}},
    }},
    {"additionalProperties", false},
  };
}

mcp::protocol::ToolDefinition tool_definition(
  std::string name,
  std::string title,
  std::string description,
  Json input_schema,
  bool read_only,
  bool destructive,
  bool idempotent,
  bool open_world)
{
  mcp::protocol::ToolAnnotations annotations;
  annotations.title = title;
  annotations.description_hint = description;
  annotations.read_only_hint = read_only;
  annotations.destructive_hint = destructive;
  annotations.idempotent_hint = idempotent;
  annotations.open_world_hint = open_world;

  return mcp::protocol::tool_definition(std::move(name))
         .title(std::move(title))
         .description(std::move(description))
         .input_schema(std::move(input_schema))
         .output_schema(output_envelope_schema())
         .tool_annotations(std::move(annotations))
         .build();
}

mcp::protocol::ToolResult tool_result(OperationResult operation, std::uint64_t request_number)
{
  Json envelope{
    {"ok", operation.ok},
    {"code", operation.code},
    {"message", operation.message},
    {"data", operation.data},
    {"request_id", "request-" + std::to_string(request_number)},
    {"timestamp", timestamp_now()},
  };

  mcp::protocol::ContentBlock text;
  text.type = "text";
  text.text = envelope.dump(2);
  text.data = Json::object();

  mcp::protocol::ToolResult result;
  result.content.push_back(std::move(text));
  result.structured_content = envelope;
  result.is_error = !operation.ok;
  return result;
}

template<typename BuilderT>
void register_tool(
  BuilderT & builder,
  mcp::protocol::ToolDefinition definition,
  ToolFunction function,
  const std::shared_ptr<std::atomic_uint64_t> & request_sequence)
{
  builder.add_tool(
    std::move(definition),
    [function = std::move(function), request_sequence](
      const mcp::server::ToolContext & context)
    -> mcp::core::Result<mcp::protocol::ToolResult> {
      OperationResult operation;
      try {
        if (!context.arguments.is_object()) {
          operation = OperationResult::failure("invalid_arguments", "tool arguments must be an object");
        } else {
          operation = function(context.arguments);
        }
      } catch (const std::exception & error) {
        operation = OperationResult::failure("invalid_arguments", error.what());
      }
      return tool_result(std::move(operation), ++(*request_sequence));
    });
}

std::size_t bounded_size_argument(
  const Json & arguments,
  const char * name,
  std::size_t default_value,
  std::size_t maximum)
{
  if (!arguments.contains(name)) {
    return default_value;
  }
  const auto value = arguments.at(name).get<long long>();
  if (value <= 0 || static_cast<unsigned long long>(value) > maximum) {
    throw std::invalid_argument(std::string{name} + " is outside its allowed range");
  }
  return static_cast<std::size_t>(value);
}

template<typename BuilderT>
void register_read_tools(
  BuilderT & builder,
  const std::shared_ptr<RobotBridge> & bridge,
  const std::shared_ptr<std::atomic_uint64_t> & sequence)
{
  register_tool(
    builder,
    tool_definition(
      "get_server_info", "MCP server info",
      "Return immutable write policy, robot mode, log roots and current motion state.",
      empty_object_schema(), true, false, true, false),
    [bridge](const Json &) {return bridge->server_info();}, sequence);

  register_tool(
    builder,
    tool_definition(
      "get_robot_status", "Robot status",
      "Return ROS graph counts, controller states, joint positions and motion state.",
      empty_object_schema(), true, false, true, true),
    [bridge](const Json &) {return bridge->robot_status();}, sequence);

  register_tool(
    builder,
    tool_definition(
      "get_ethercat_status", "EtherCAT status",
      "Run fixed, read-only 'ethercat master' and 'ethercat slaves' queries with a timeout.",
      empty_object_schema(), true, false, true, true),
    [bridge](const Json &) {return bridge->ethercat_status();}, sequence);

  register_tool(
    builder,
    tool_definition(
      "list_ros_nodes", "List ROS nodes", "List discovered ROS 2 node names.",
      empty_object_schema(), true, false, true, true),
    [bridge](const Json &) {return bridge->list_ros_nodes();}, sequence);

  register_tool(
    builder,
    tool_definition(
      "list_ros_topics", "List ROS topics",
      "List discovered ROS 2 topics and message types, optionally filtered by name.",
      Json{
        {"type", "object"},
        {"properties", Json{
          {"filter", Json{{"type", "string"}, {"default", ""}}},
          {"limit", Json{{"type", "integer"}, {"minimum", 1}, {"maximum", 500}, {"default", 200}}},
        }},
        {"additionalProperties", false},
      }, true, false, true, true),
    [bridge](const Json & arguments) {
      return bridge->list_ros_topics(
        arguments.value("filter", std::string{}),
        bounded_size_argument(arguments, "limit", 200U, 500U));
    }, sequence);

  register_tool(
    builder,
    tool_definition(
      "list_recent_logs", "List robot logs",
      "List recent .log and .txt files only within configured robot log roots.",
      Json{
        {"type", "object"},
        {"properties", Json{
          {"limit", Json{{"type", "integer"}, {"minimum", 1}, {"maximum", 200}, {"default", 50}}},
        }},
        {"additionalProperties", false},
      }, true, false, true, false),
    [bridge](const Json & arguments) {
      return bridge->list_recent_logs(bounded_size_argument(arguments, "limit", 50U, 200U));
    }, sequence);

  register_tool(
    builder,
    tool_definition(
      "read_log", "Read robot log tail",
      "Read the tail of a .log or .txt file under an allowed root; path traversal is rejected.",
      Json{
        {"type", "object"},
        {"required", Json::array({"root", "path"})},
        {"properties", Json{
          {"root", Json{{"type", "string"}, {"enum", Json::array({"robot", "stability"})}}},
          {"path", Json{{"type", "string"}, {"minLength", 1}}},
          {"tail_lines", Json{{"type", "integer"}, {"minimum", 1}, {"maximum", 2000}, {"default", 200}}},
        }},
        {"additionalProperties", false},
      }, true, false, true, false),
    [bridge](const Json & arguments) {
      return bridge->read_log(
        arguments.at("root").get<std::string>(),
        arguments.at("path").get<std::string>(),
        bounded_size_argument(arguments, "tail_lines", 200U, 2000U));
    }, sequence);

  register_tool(
    builder,
    tool_definition(
      "get_motion_status", "Motion status",
      "Return the state of the most recently accepted asynchronous MCP motion.",
      empty_object_schema(), true, false, true, true),
    [bridge](const Json &) {return bridge->motion_status();}, sequence);
}

template<typename BuilderT>
void register_write_tools(
  BuilderT & builder,
  const std::shared_ptr<RobotBridge> & bridge,
  const std::shared_ptr<std::atomic_uint64_t> & sequence)
{
  register_tool(
    builder,
    tool_definition(
      "move_named_target", "Move to named target",
      "Plan and execute an allowlisted MoveIt named target asynchronously. Requires confirmation='MOVE ROBOT'.",
      Json{
        {"type", "object"},
        {"required", Json::array({"group", "target", "confirmation"})},
        {"properties", Json{
          {"group", Json{{"type", "string"}, {"enum", Json::array({"left_arm", "right_arm", "both_arms"})}}},
          {"target", Json{{"type", "string"}, {"minLength", 1}}},
          {"confirmation", Json{{"type", "string"}, {"const", "MOVE ROBOT"}}},
        }},
        {"additionalProperties", false},
      }, false, true, false, true),
    [bridge](const Json & arguments) {
      return bridge->move_named(
        arguments.at("group").get<std::string>(),
        arguments.at("target").get<std::string>(),
        arguments.at("confirmation").get<std::string>());
    }, sequence);

  register_tool(
    builder,
    tool_definition(
      "move_joint_target", "Move to joint target",
      "Plan and execute a 7- or 14-joint MoveIt target asynchronously. Physical mode also enforces a fresh-state step limit.",
      Json{
        {"type", "object"},
        {"required", Json::array({"group", "positions", "confirmation"})},
        {"properties", Json{
          {"group", Json{{"type", "string"}, {"enum", Json::array({"left_arm", "right_arm", "both_arms"})}}},
          {"positions", Json{{"type", "array"}, {"items", Json{{"type", "number"}}}, {"minItems", 7}, {"maxItems", 14}}},
          {"confirmation", Json{{"type", "string"}, {"const", "MOVE ROBOT"}}},
        }},
        {"additionalProperties", false},
      }, false, true, false, true),
    [bridge](const Json & arguments) {
      return bridge->move_joints(
        arguments.at("group").get<std::string>(),
        arguments.at("positions").get<std::vector<double>>(),
        arguments.at("confirmation").get<std::string>());
    }, sequence);

  register_tool(
    builder,
    tool_definition(
      "set_gripper", "Set gripper position",
      "Plan and execute an allowlisted gripper target in [0, 0.06] metres. Requires confirmation='MOVE ROBOT'.",
      Json{
        {"type", "object"},
        {"required", Json::array({"side", "position", "confirmation"})},
        {"properties", Json{
          {"side", Json{{"type", "string"}, {"enum", Json::array({"left", "right", "both"})}}},
          {"position", Json{{"type", "number"}, {"minimum", 0.0}, {"maximum", 0.06}}},
          {"confirmation", Json{{"type", "string"}, {"const", "MOVE ROBOT"}}},
        }},
        {"additionalProperties", false},
      }, false, true, false, true),
    [bridge](const Json & arguments) {
      return bridge->set_gripper(
        arguments.at("side").get<std::string>(),
        arguments.at("position").get<double>(),
        arguments.at("confirmation").get<std::string>());
    }, sequence);

  register_tool(
    builder,
    tool_definition(
      "stop_motion", "Stop MoveIt motion",
      "Request MoveIt to stop the current MCP motion. This cannot replace the physical emergency stop.",
      empty_object_schema(), false, false, true, true),
    [bridge](const Json &) {return bridge->stop_motion();}, sequence);

  register_tool(
    builder,
    tool_definition(
      "reset_arm_fault", "Reset arm fault",
      "Reset a left/right/both arm EtherCAT fault without enabling motors. Physical mode only; requires confirmation='RESET FAULT'.",
      Json{
        {"type", "object"},
        {"required", Json::array({"arm", "confirmation"})},
        {"properties", Json{
          {"arm", Json{{"type", "string"}, {"enum", Json::array({"left", "right", "both"})}}},
          {"confirmation", Json{{"type", "string"}, {"const", "RESET FAULT"}}},
        }},
        {"additionalProperties", false},
      }, false, true, false, true),
    [bridge](const Json & arguments) {
      return bridge->reset_fault(
        arguments.at("arm").get<std::string>(),
        arguments.at("confirmation").get<std::string>());
    }, sequence);

  register_tool(
    builder,
    tool_definition(
      "set_motors_enabled", "Enable or disable arm motors",
      "Enable or disable left/right/both arm motors through fixed recovery services. Physical mode only.",
      Json{
        {"type", "object"},
        {"required", Json::array({"arm", "enabled", "confirmation"})},
        {"properties", Json{
          {"arm", Json{{"type", "string"}, {"enum", Json::array({"left", "right", "both"})}}},
          {"enabled", Json{{"type", "boolean"}}},
          {"confirmation", Json{
            {"type", "string"},
            {"enum", Json::array({"ENABLE MOTORS", "DISABLE MOTORS"})}}},
        }},
        {"additionalProperties", false},
      }, false, true, false, true),
    [bridge](const Json & arguments) {
      return bridge->set_motors_enabled(
        arguments.at("arm").get<std::string>(),
        arguments.at("enabled").get<bool>(),
        arguments.at("confirmation").get<std::string>());
    }, sequence);
}

}  // namespace
}  // namespace zhishu_robot_mcp

int main(int argc, char ** argv)
{
  // MCP stdio owns stdout.  Force ROS logging to stderr before rclcpp starts.
  setenv("RCUTILS_LOGGING_USE_STDOUT", "0", 0);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("zhishu_robot_mcp_server");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4U);
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  std::shared_ptr<zhishu_robot_mcp::RobotBridge> bridge;
  int exit_code = 1;
  try {
    const auto workspace = zhishu_robot_mcp::workspace_path();
    bridge = std::make_shared<zhishu_robot_mcp::RobotBridge>(
      node,
      zhishu_robot_mcp::ServerOptions::from_environment(),
      zhishu_robot_mcp::log_roots(workspace));

    auto builder = mcp::ServerPeer::builder();
    builder
    .name("zhishu-robot-mcp")
    .version("0.1.0")
    .instructions(
      "Zhishu dual-arm ROS 2 diagnostics and guarded control. Read tools are always available. "
      "Write tools are server-gated, allowlisted, rate-limited and disabled by default. "
      "MoveIt stop is not a physical emergency stop.")
    .stdio();

    auto request_sequence = std::make_shared<std::atomic_uint64_t>(0U);
    zhishu_robot_mcp::register_read_tools(builder, bridge, request_sequence);
    zhishu_robot_mcp::register_write_tools(builder, bridge, request_sequence);
    exit_code = builder.run();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "zhishu_robot_mcp fatal: %s\n", error.what());
  }

  if (bridge) {
    bridge->shutdown();
  }
  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  rclcpp::shutdown();
  return exit_code;
}
