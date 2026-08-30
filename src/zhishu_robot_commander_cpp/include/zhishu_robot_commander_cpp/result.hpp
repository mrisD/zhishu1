#ifndef ZHISHU_ROBOT_COMMANDER_CPP__RESULT_HPP_
#define ZHISHU_ROBOT_COMMANDER_CPP__RESULT_HPP_

#include <string>
#include <utility>

namespace zhishu_robot_commander_cpp
{

enum class ErrorCode
{
  kSuccess = 0,
  kInvalidArgument,
  kTargetRejected,
  kPlanningFailed,
  kExecutionFailed,
  kBusy,
  kCancelled,
  kTimeout,
  kNotReady,
  kInternalError,
};

const char * toString(ErrorCode code) noexcept;

struct Result
{
  ErrorCode code{ErrorCode::kSuccess};
  std::string message;
  int native_code{0};

  bool ok() const noexcept
  {
    return code == ErrorCode::kSuccess;
  }

  explicit operator bool() const noexcept
  {
    return ok();
  }

  static Result success(std::string message = {})
  {
    return Result{ErrorCode::kSuccess, std::move(message), 0};
  }

  static Result failure(ErrorCode code, std::string message, int native_code = 0)
  {
    return Result{code, std::move(message), native_code};
  }
};

}  // namespace zhishu_robot_commander_cpp

#endif  // ZHISHU_ROBOT_COMMANDER_CPP__RESULT_HPP_
