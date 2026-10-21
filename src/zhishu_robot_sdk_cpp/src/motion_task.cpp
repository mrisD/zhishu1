#include "motion_task.hpp"

#include <exception>
#include <utility>

namespace zhishu_robot_sdk::detail
{
bool requestCancellation(const std::shared_ptr<MotionState> & state)
{
  if (!state) {return false;}
  std::function<void()> callback;
  MotionStatus previous_status;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (isTerminal(state->status) || state->cancel_requested) {return false;}
    callback = state->cancel_callback;
    previous_status = state->status;
    state->cancel_requested = true;
    state->cancel_callback_running = true;
    state->status = MotionStatus::kCancelling;
  }
  state->condition.notify_all();

  bool accepted = true;
  try {
    if (callback) {callback();}
  } catch (...) {
    // A failed stop request must not turn a completed operation into a
    // successful cancellation, or escape from RobotSdk's destructor.
    accepted = false;
  }
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (!accepted) {
      state->cancel_requested = false;
      state->status = previous_status;
    }
    state->cancel_callback_running = false;
  }
  state->condition.notify_all();
  return accepted;
}

void executeMotionTask(
  const std::shared_ptr<MotionState> & state, const std::function<Result()> & operation)
{
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    state->condition.wait(lock, [&]() {return !state->cancel_callback_running;});
    if (state->cancel_requested) {
      state->status = MotionStatus::kCancelled;
      state->result = Result::failure(ErrorCode::kCancelled, "motion cancelled before execution");
      lock.unlock();
      state->condition.notify_all();
      return;
    }
    state->status = MotionStatus::kRunning;
  }
  state->condition.notify_all();

  Result result;
  try {
    result = operation();
  } catch (const std::exception & error) {
    result = Result::failure(ErrorCode::kInternalError, error.what());
  } catch (...) {
    result = Result::failure(ErrorCode::kInternalError, "unknown SDK exception");
  }
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    // Keep the task nonterminal until its stop callback has returned. Otherwise
    // a delayed callback could stop a newly admitted motion.
    state->condition.wait(lock, [&]() {return !state->cancel_callback_running;});
    if (state->cancel_requested) {
      state->status = MotionStatus::kCancelled;
      state->result = Result::failure(ErrorCode::kCancelled, "motion cancelled");
    } else {
      state->status = result.ok() ? MotionStatus::kSucceeded : MotionStatus::kFailed;
      state->result = std::move(result);
    }
  }
  state->condition.notify_all();
}
}  // namespace zhishu_robot_sdk::detail
