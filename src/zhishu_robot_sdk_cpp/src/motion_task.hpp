#ifndef ZHISHU_ROBOT_SDK__MOTION_TASK_HPP_
#define ZHISHU_ROBOT_SDK__MOTION_TASK_HPP_

#include "zhishu_robot_sdk/motion_handle.hpp"

namespace zhishu_robot_sdk::detail
{
// Internal task state machine, shared by MotionHandle and the SDK worker.
ZHISHU_ROBOT_SDK_PUBLIC bool requestCancellation(const std::shared_ptr<MotionState> & state);
ZHISHU_ROBOT_SDK_PUBLIC void executeMotionTask(
  const std::shared_ptr<MotionState> & state, const std::function<Result()> & operation);
}  // namespace zhishu_robot_sdk::detail

#endif
