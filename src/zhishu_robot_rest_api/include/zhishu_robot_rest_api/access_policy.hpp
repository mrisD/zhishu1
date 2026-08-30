#ifndef ZHISHU_ROBOT_REST_API__ACCESS_POLICY_HPP_
#define ZHISHU_ROBOT_REST_API__ACCESS_POLICY_HPP_

#include "zhishu_robot_rest_api/types.hpp"

#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <string_view>

namespace zhishu_robot_rest_api
{

class AccessPolicy
{
public:
  explicit AccessPolicy(RestOptions options);

  const RestOptions & options() const noexcept;
  bool authorize_read(std::string_view bearer_token, std::string & reason) const;
  bool authorize_write(
    std::string_view bearer_token,
    std::string_view required_confirmation,
    std::string_view supplied_confirmation,
    bool apply_rate_limit,
    std::string & reason);

private:
  bool token_matches(std::string_view supplied) const;
  void discard_expired_locked(std::chrono::steady_clock::time_point now);

  RestOptions options_;
  std::mutex mutex_;
  std::deque<std::chrono::steady_clock::time_point> recent_writes_;
};

}  // namespace zhishu_robot_rest_api

#endif  // ZHISHU_ROBOT_REST_API__ACCESS_POLICY_HPP_
