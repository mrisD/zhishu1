#include "zhishu_robot_commander_cpp/result.hpp"

namespace zhishu_robot_commander_cpp
{

const char * toString(ErrorCode code) noexcept
{
  switch (code) {
    case ErrorCode::kSuccess:
      return "success";
    case ErrorCode::kInvalidArgument:
      return "invalid_argument";
    case ErrorCode::kTargetRejected:
      return "target_rejected";
    case ErrorCode::kPlanningFailed:
      return "planning_failed";
    case ErrorCode::kExecutionFailed:
      return "execution_failed";
    case ErrorCode::kBusy:
      return "busy";
    case ErrorCode::kCancelled:
      return "cancelled";
    case ErrorCode::kTimeout:
      return "timeout";
    case ErrorCode::kNotReady:
      return "not_ready";
    case ErrorCode::kInternalError:
      return "internal_error";
  }
  return "unknown";
}

}  // namespace zhishu_robot_commander_cpp
