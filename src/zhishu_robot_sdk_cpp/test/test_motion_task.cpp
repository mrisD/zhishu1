#include "motion_task.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <stdexcept>

namespace zhishu_robot_sdk
{
namespace
{
using namespace std::chrono_literals;

TEST(MotionTaskTest, SuccessAndOperationFailureStayDistinct)
{
  auto state = std::make_shared<detail::MotionState>();
  detail::executeMotionTask(state, []() {return Result::success("completed");});
  EXPECT_EQ(state->status, MotionStatus::kSucceeded);
  EXPECT_EQ(state->result.message, "completed");
  EXPECT_FALSE(detail::requestCancellation(state));

  state = std::make_shared<detail::MotionState>();
  detail::executeMotionTask(state, []() {
      return Result::failure(ErrorCode::kPlanningFailed, "no plan", -1);
    });
  EXPECT_EQ(state->status, MotionStatus::kFailed);
  EXPECT_EQ(state->result.code, ErrorCode::kPlanningFailed);
  EXPECT_EQ(state->result.native_code, -1);
}

TEST(MotionTaskTest, QueuedCancellationDoesNotExecuteOperation)
{
  auto state = std::make_shared<detail::MotionState>();
  int stops = 0;
  state->cancel_callback = [&]() {++stops;};
  ASSERT_TRUE(detail::requestCancellation(state));
  EXPECT_FALSE(detail::requestCancellation(state));
  bool executed = false;
  detail::executeMotionTask(state, [&]() {executed = true; return Result::success();});
  EXPECT_FALSE(executed);
  EXPECT_EQ(stops, 1);
  EXPECT_EQ(state->status, MotionStatus::kCancelled);
  EXPECT_EQ(state->result.code, ErrorCode::kCancelled);
}

TEST(MotionTaskTest, CompletionWaitsForOutstandingStopCallback)
{
  auto state = std::make_shared<detail::MotionState>();
  std::promise<void> operation_entered, operation_returned, release_operation;
  std::promise<void> stop_entered, release_stop;
  auto operation_release = release_operation.get_future().share();
  auto stop_release = release_stop.get_future().share();
  state->cancel_callback = [&]() {
      stop_entered.set_value();
      if (stop_release.wait_for(3s) != std::future_status::ready) {
        throw std::runtime_error("test stop callback release timed out");
      }
    };
  auto worker = std::async(std::launch::async, [&]() {
      detail::executeMotionTask(state, [&]() {
          operation_entered.set_value();
          operation_release.wait_for(3s);
          operation_returned.set_value();
          return Result::success();
        });
    });
  EXPECT_EQ(operation_entered.get_future().wait_for(2s), std::future_status::ready);
  auto cancellation = std::async(std::launch::async, [&]() {
      return detail::requestCancellation(state);
    });
  EXPECT_EQ(stop_entered.get_future().wait_for(2s), std::future_status::ready);
  release_operation.set_value();
  EXPECT_EQ(operation_returned.get_future().wait_for(2s), std::future_status::ready);
  EXPECT_EQ(worker.wait_for(50ms), std::future_status::timeout);
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    EXPECT_EQ(state->status, MotionStatus::kCancelling);
    EXPECT_TRUE(state->cancel_callback_running);
  }
  release_stop.set_value();
  EXPECT_TRUE(cancellation.get());
  worker.get();
  EXPECT_EQ(state->status, MotionStatus::kCancelled);
}

TEST(MotionTaskTest, StopCallbackExceptionDoesNotEscapeOrForgeCancellation)
{
  auto state = std::make_shared<detail::MotionState>();
  state->cancel_callback = []() {throw std::runtime_error("stop unavailable");};
  EXPECT_FALSE(detail::requestCancellation(state));
  EXPECT_FALSE(state->cancel_requested);
  EXPECT_FALSE(state->cancel_callback_running);
  EXPECT_EQ(state->status, MotionStatus::kQueued);
  detail::executeMotionTask(state, []() {return Result::success();});
  EXPECT_EQ(state->status, MotionStatus::kSucceeded);
}

TEST(MotionTaskTest, OperationExceptionsAlwaysCompleteTheTask)
{
  auto state = std::make_shared<detail::MotionState>();
  detail::executeMotionTask(state, []() -> Result {throw std::runtime_error("backend error");});
  EXPECT_EQ(state->status, MotionStatus::kFailed);
  EXPECT_EQ(state->result.code, ErrorCode::kInternalError);
  EXPECT_EQ(state->result.message, "backend error");
  state = std::make_shared<detail::MotionState>();
  detail::executeMotionTask(state, []() -> Result {throw 42;});
  EXPECT_EQ(state->status, MotionStatus::kFailed);
  EXPECT_EQ(state->result.code, ErrorCode::kInternalError);
}
}  // namespace
}  // namespace zhishu_robot_sdk
