#include "zhishu_robot_rest_api/log_access.hpp"
#include "zhishu_robot_rest_api/rest_server.hpp"
#include "zhishu_robot_rest_api/robot_bridge.hpp"
#include "zhishu_robot_rest_api/types.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace zhishu_robot_rest_api
{
namespace
{

std::string environment_value(const char * name)
{
  const char * value = std::getenv(name);
  return value == nullptr ? std::string{} : std::string{value};
}

std::filesystem::path workspace_path()
{
  const auto configured = environment_value("ZHISHU_REST_WORKSPACE");
  if (!configured.empty()) {
    return std::filesystem::path{configured}.lexically_normal();
  }
  auto current = std::filesystem::path{
    ament_index_cpp::get_package_prefix("zhishu_robot_rest_api")}.lexically_normal();
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

std::string load_openapi_document()
{
  const auto path = std::filesystem::path{
    ament_index_cpp::get_package_share_directory("zhishu_robot_rest_api")} /
    "openapi" / "zhishu_robot_rest_api.openapi.json";
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot read OpenAPI document: " + path.string());
  }
  return std::string{std::istreambuf_iterator<char>{stream}, std::istreambuf_iterator<char>{}};
}

}  // namespace

int run_server(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("zhishu_robot_rest_api_server");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4U);
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  std::shared_ptr<RobotBridge> bridge;
  std::unique_ptr<RestServer> server;
  std::atomic_bool listen_finished{false};
  std::thread shutdown_monitor;
  int exit_code = 1;
  try {
    auto options = RestOptions::from_environment();
    const auto workspace = workspace_path();
    bridge = std::make_shared<RobotBridge>(node, options, log_roots(workspace));
    server = std::make_unique<RestServer>(bridge, options, load_openapi_document());

    std::fprintf(
      stderr, "zhishu_robot_rest_api listening on http://%s:%u/api/v1\n",
      options.host.c_str(), static_cast<unsigned int>(options.port));
    shutdown_monitor = std::thread([&server, &listen_finished]() {
        while (rclcpp::ok() && !listen_finished.load()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!listen_finished.load() && server) {
          server->stop();
        }
      });
    const bool listen_result = server->listen();
    listen_finished.store(true);
    exit_code = listen_result || !rclcpp::ok() ? 0 : 1;
  } catch (const std::exception & error) {
    std::fprintf(stderr, "zhishu_robot_rest_api fatal: %s\n", error.what());
  }

  listen_finished.store(true);
  if (shutdown_monitor.joinable()) {
    shutdown_monitor.join();
  }
  if (bridge) {
    bridge->shutdown();
  }
  executor.cancel();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  return exit_code;
}

}  // namespace zhishu_robot_rest_api

int main(int argc, char ** argv)
{
  return zhishu_robot_rest_api::run_server(argc, argv);
}
