#ifndef ZHISHU_ROBOT_SDK__MOTION_HANDLE_HPP_
#define ZHISHU_ROBOT_SDK__MOTION_HANDLE_HPP_

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "zhishu_robot_sdk/result.hpp"
#include "zhishu_robot_sdk/visibility_control.h"

namespace zhishu_robot_sdk
{

enum class MotionStatus
{
  kInvalid = 0,
  kQueued,
  kRunning,
  kCancelling,
  kSucceeded,
  kFailed,
  kCancelled,
};

ZHISHU_ROBOT_SDK_PUBLIC const char * toString(MotionStatus status) noexcept;
ZHISHU_ROBOT_SDK_PUBLIC bool isTerminal(MotionStatus status) noexcept;

struct MotionSnapshot
{
  std::uint64_t id{0};
  std::string operation;
  MotionStatus status{MotionStatus::kInvalid};
  Result result{Result::failure(ErrorCode::kNotReady, "motion handle is invalid")};
};

namespace detail
{
struct MotionState
{
  mutable std::mutex mutex;
  std::condition_variable condition;
  std::uint64_t id{0};
  std::string operation;
  MotionStatus status{MotionStatus::kQueued};
  Result result{Result::failure(ErrorCode::kNotReady, "motion has not completed")};
  bool cancel_requested{false};
  bool cancel_callback_running{false};
  std::function<void()> cancel_callback;
};
}  // namespace detail

class RobotSdk;

class ZHISHU_ROBOT_SDK_PUBLIC MotionHandle
{
public:
  MotionHandle() = default;

  bool valid() const noexcept;
  std::uint64_t id() const;
  MotionStatus status() const;
  MotionSnapshot snapshot() const;
  bool done() const;
  // Requests cancellation; returns false when unavailable/already requested or
  // the stop callback fails. Wait for the operation to reach its terminal state.
  bool cancel();
  Result result() const;
  Result wait() const;
  Result waitFor(std::chrono::milliseconds timeout) const;

private:
  explicit MotionHandle(std::shared_ptr<detail::MotionState> state);

  std::shared_ptr<detail::MotionState> state_;

  friend class RobotSdk;
};

}  // namespace zhishu_robot_sdk

#endif  // ZHISHU_ROBOT_SDK__MOTION_HANDLE_HPP_
