#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <array>
#include <chrono>
#include <memory>
#include <string>

namespace zhishu_robot_commander_cpp
{

// Coordinates the two hardware recorders used by the stability executables.
// Service calls are intentionally made outside the real-time EtherCAT threads.
class StabilityPdoRecording
{
public:
  StabilityPdoRecording(
    rclcpp::Node::SharedPtr node,
    std::chrono::milliseconds service_timeout);
  ~StabilityPdoRecording();

  StabilityPdoRecording(const StabilityPdoRecording &) = delete;
  StabilityPdoRecording & operator=(const StabilityPdoRecording &) = delete;

  bool start();
  bool stop();
  bool active() const;

private:
  struct CallResult
  {
    bool completed{false};
    bool success{false};
    std::string message;
  };

  std::array<CallResult, 2> callBoth(bool enabled);

  rclcpp::Node::SharedPtr node_;
  std::chrono::milliseconds service_timeout_;
  std::array<std::string, 2> service_names_{{
    "/left_arm/record_pdo", "/right_arm/record_pdo"}};
  std::array<rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr, 2> clients_;
  bool active_{false};
};

}  // namespace zhishu_robot_commander_cpp
