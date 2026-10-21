#include "stability_pdo_recording.hpp"

#include <algorithm>
#include <future>
#include <utility>
#include <vector>

namespace zhishu_robot_sdk
{

StabilityPdoRecording::StabilityPdoRecording(
  rclcpp::Node::SharedPtr node,
  std::chrono::milliseconds service_timeout)
: node_(std::move(node)), service_timeout_(service_timeout)
{
  for (std::size_t index = 0; index < clients_.size(); ++index) {
    clients_[index] = node_->create_client<std_srvs::srv::SetBool>(service_names_[index]);
  }
}

StabilityPdoRecording::~StabilityPdoRecording()
{
  if (active_ && rclcpp::ok()) {
    stop();
  }
}

std::array<StabilityPdoRecording::CallResult, 2>
StabilityPdoRecording::callBoth(bool enabled)
{
  std::array<CallResult, 2> results;
  const auto deadline = std::chrono::steady_clock::now() + service_timeout_;

  for (std::size_t index = 0; index < clients_.size(); ++index) {
    const auto remaining = deadline - std::chrono::steady_clock::now();
    if (remaining <= std::chrono::steady_clock::duration::zero() ||
      !clients_[index]->wait_for_service(remaining))
    {
      results[index].message = "service unavailable or discovery timed out";
      return results;
    }
  }

  std::vector<rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture> futures;
  futures.reserve(clients_.size());
  for (const auto & client : clients_) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    auto future_and_request_id = client->async_send_request(request);
    futures.push_back(future_and_request_id.future.share());
  }

  for (std::size_t index = 0; index < futures.size(); ++index) {
    if (futures[index].wait_until(deadline) != std::future_status::ready) {
      results[index].message = "service call timed out";
      continue;
    }
    try {
      const auto response = futures[index].get();
      results[index].completed = true;
      results[index].success = response != nullptr && response->success;
      results[index].message = response == nullptr ?
        "service returned no response" : response->message;
    } catch (const std::exception & error) {
      results[index].message = error.what();
    }
  }
  return results;
}

bool StabilityPdoRecording::start()
{
  if (active_) {
    return true;
  }

  // Stop a still-running bounded startup capture first. The driver resumes the
  // same per-master CSV, so startup and motion samples remain in one file.
  const auto cleanup_results = callBoth(false);
  if (!std::all_of(
      cleanup_results.begin(), cleanup_results.end(),
      [](const CallResult & result) {return result.completed;}))
  {
    for (std::size_t index = 0; index < cleanup_results.size(); ++index) {
      if (!cleanup_results[index].completed) {
        RCLCPP_ERROR(
          node_->get_logger(), "[STABILITY][PDO_START_FAIL] service=%s reason=%s",
          service_names_[index].c_str(), cleanup_results[index].message.c_str());
      }
    }
    return false;
  }

  const auto start_results = callBoth(true);
  bool all_started = true;
  for (std::size_t index = 0; index < start_results.size(); ++index) {
    if (!start_results[index].completed || !start_results[index].success) {
      all_started = false;
      RCLCPP_ERROR(
        node_->get_logger(), "[STABILITY][PDO_START_FAIL] service=%s reason=%s",
        service_names_[index].c_str(), start_results[index].message.c_str());
    } else {
      RCLCPP_INFO(
        node_->get_logger(), "[STABILITY][PDO_STARTED] service=%s %s",
        service_names_[index].c_str(), start_results[index].message.c_str());
    }
  }

  if (!all_started) {
    // Roll back a partial start so the two arm files never describe different
    // motion windows.
    callBoth(false);
    return false;
  }

  active_ = true;
  return true;
}

bool StabilityPdoRecording::stop()
{
  if (!active_) {
    return true;
  }

  const auto stop_results = callBoth(false);
  bool all_stopped = true;
  for (std::size_t index = 0; index < stop_results.size(); ++index) {
    if (!stop_results[index].completed || !stop_results[index].success) {
      all_stopped = false;
      RCLCPP_ERROR(
        node_->get_logger(), "[STABILITY][PDO_STOP_FAIL] service=%s reason=%s",
        service_names_[index].c_str(), stop_results[index].message.c_str());
    } else {
      RCLCPP_INFO(
        node_->get_logger(), "[STABILITY][PDO_STOPPED] service=%s %s",
        service_names_[index].c_str(), stop_results[index].message.c_str());
    }
  }
  active_ = !all_stopped;
  return all_stopped;
}

bool StabilityPdoRecording::active() const
{
  return active_;
}

}  // namespace zhishu_robot_sdk
