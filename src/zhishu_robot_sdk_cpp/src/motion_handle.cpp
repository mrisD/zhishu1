#include "zhishu_robot_sdk/motion_handle.hpp"

#include <utility>

namespace zhishu_robot_sdk
{

const char * toString(MotionStatus status) noexcept
{
  switch (status) {
    case MotionStatus::kInvalid:
      return "invalid";
    case MotionStatus::kQueued:
      return "queued";
    case MotionStatus::kRunning:
      return "running";
    case MotionStatus::kCancelling:
      return "cancelling";
    case MotionStatus::kSucceeded:
      return "succeeded";
    case MotionStatus::kFailed:
      return "failed";
    case MotionStatus::kCancelled:
      return "cancelled";
  }
  return "unknown";
}

bool isTerminal(MotionStatus status) noexcept
{
  return status == MotionStatus::kSucceeded || status == MotionStatus::kFailed ||
         status == MotionStatus::kCancelled;
}

MotionHandle::MotionHandle(std::shared_ptr<detail::MotionState> state)
: state_(std::move(state))
{
}

bool MotionHandle::valid() const noexcept
{
  return static_cast<bool>(state_);
}

std::uint64_t MotionHandle::id() const
{
  if (!state_) {
    return 0;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->id;
}

MotionStatus MotionHandle::status() const
{
  if (!state_) {
    return MotionStatus::kInvalid;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->status;
}

MotionSnapshot MotionHandle::snapshot() const
{
  if (!state_) {
    return MotionSnapshot{};
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  return MotionSnapshot{state_->id, state_->operation, state_->status, state_->result};
}

bool MotionHandle::done() const
{
  return isTerminal(status());
}

bool MotionHandle::cancel()
{
  if (!state_) {
    return false;
  }

  std::function<void()> callback;
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (isTerminal(state_->status) || state_->cancel_requested) {
      return false;
    }
    state_->cancel_requested = true;
    state_->status = MotionStatus::kCancelling;
    callback = state_->cancel_callback;
  }
  state_->condition.notify_all();
  if (callback) {
    callback();
  }
  return true;
}

Result MotionHandle::result() const
{
  if (!state_) {
    return Result::failure(ErrorCode::kNotReady, "motion handle is invalid");
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (!isTerminal(state_->status)) {
    return Result::failure(ErrorCode::kBusy, "motion has not completed");
  }
  return state_->result;
}

Result MotionHandle::wait() const
{
  if (!state_) {
    return Result::failure(ErrorCode::kNotReady, "motion handle is invalid");
  }
  std::unique_lock<std::mutex> lock(state_->mutex);
  state_->condition.wait(lock, [this]() {return isTerminal(state_->status);});
  return state_->result;
}

Result MotionHandle::waitFor(std::chrono::milliseconds timeout) const
{
  if (!state_) {
    return Result::failure(ErrorCode::kNotReady, "motion handle is invalid");
  }
  if (timeout.count() < 0) {
    return Result::failure(ErrorCode::kInvalidArgument, "timeout cannot be negative");
  }
  std::unique_lock<std::mutex> lock(state_->mutex);
  if (!state_->condition.wait_for(
      lock, timeout, [this]() {return isTerminal(state_->status);}))
  {
    return Result::failure(ErrorCode::kTimeout, "timed out waiting for motion");
  }
  return state_->result;
}

}  // namespace zhishu_robot_sdk
