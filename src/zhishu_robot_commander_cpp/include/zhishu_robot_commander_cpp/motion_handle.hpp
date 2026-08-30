#ifndef ZHISHU_ROBOT_COMMANDER_CPP__MOTION_HANDLE_HPP_
#define ZHISHU_ROBOT_COMMANDER_CPP__MOTION_HANDLE_HPP_

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "zhishu_robot_commander_cpp/result.hpp"

namespace zhishu_robot_commander_cpp
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

const char * toString(MotionStatus status) noexcept;
bool isTerminal(MotionStatus status) noexcept;

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
  std::function<void()> cancel_callback;
};
}  // namespace detail

class RobotSdk;

class MotionHandle
{
public:
  MotionHandle() = default;

  bool valid() const noexcept;
  std::uint64_t id() const;
  MotionStatus status() const;
  MotionSnapshot snapshot() const;
  bool done() const;
  bool cancel();
  Result result() const;
  Result wait() const;
  Result waitFor(std::chrono::milliseconds timeout) const;

private:
  explicit MotionHandle(std::shared_ptr<detail::MotionState> state);

  std::shared_ptr<detail::MotionState> state_;

  friend class RobotSdk;
};

}  // namespace zhishu_robot_commander_cpp

#endif  // ZHISHU_ROBOT_COMMANDER_CPP__MOTION_HANDLE_HPP_
