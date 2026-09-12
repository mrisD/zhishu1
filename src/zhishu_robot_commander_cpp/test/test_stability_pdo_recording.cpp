#include "stability_pdo_recording.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

TEST(StabilityPdoRecordingTest, SynchronizesAndRollsBackBothRecorders)
{
  setenv("ROS_LOG_DIR", "/tmp/zhishu_robot_commander_cpp_test_logs", 1);
  int argc = 0;
  rclcpp::init(argc, nullptr);

  auto service_node = std::make_shared<rclcpp::Node>("test_pdo_recording_services");
  auto client_node = std::make_shared<rclcpp::Node>("test_pdo_recording_client");
  std::atomic<bool> left_active{false};
  std::atomic<bool> right_active{false};
  std::atomic<bool> reject_right_start{false};

  const auto make_callback = [](
    std::atomic<bool> & active,
    const std::atomic<bool> * reject_start)
    {
      return [&active, reject_start](
               const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response)
             {
               if (request->data && reject_start != nullptr && reject_start->load()) {
                 response->success = false;
                 response->message = "injected start failure";
                 return;
               }
               const bool was_active = active.exchange(request->data);
               response->success = request->data || was_active;
               response->message = request->data ? "started" : "stopped";
             };
    };

  auto left_service = service_node->create_service<std_srvs::srv::SetBool>(
    "/left_arm/record_pdo", make_callback(left_active, nullptr));
  auto right_service = service_node->create_service<std_srvs::srv::SetBool>(
    "/right_arm/record_pdo", make_callback(right_active, &reject_right_start));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(service_node);
  executor.add_node(client_node);
  std::thread spinner([&executor]() {executor.spin();});

  {
    zhishu_robot_commander_cpp::StabilityPdoRecording recording(client_node, 2s);
    EXPECT_TRUE(recording.start());
    EXPECT_TRUE(recording.active());
    EXPECT_TRUE(left_active.load());
    EXPECT_TRUE(right_active.load());
    EXPECT_TRUE(recording.stop());
    EXPECT_FALSE(left_active.load());
    EXPECT_FALSE(right_active.load());

    reject_right_start.store(true);
    EXPECT_FALSE(recording.start());
    EXPECT_FALSE(recording.active());
    EXPECT_FALSE(left_active.load());
    EXPECT_FALSE(right_active.load());
  }

  executor.cancel();
  spinner.join();
  executor.remove_node(client_node);
  executor.remove_node(service_node);
  left_service.reset();
  right_service.reset();
  rclcpp::shutdown();
}
