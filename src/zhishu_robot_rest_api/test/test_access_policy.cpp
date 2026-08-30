#include "zhishu_robot_rest_api/access_policy.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace zhishu_robot_rest_api
{

TEST(RestOptions, DefaultsToLoopbackReadOnly)
{
  const RestOptions options;
  EXPECT_TRUE(options.is_loopback());
  EXPECT_FALSE(options.read_auth_required());
  EXPECT_FALSE(options.write_enabled);
  EXPECT_NO_THROW(options.validate());
}

TEST(RestOptions, RemoteBindingRequiresExplicitOptInAndToken)
{
  RestOptions options;
  options.host = "0.0.0.0";
  EXPECT_THROW(options.validate(), std::invalid_argument);
  options.allow_remote = true;
  EXPECT_THROW(options.validate(), std::invalid_argument);
  options.api_token = "secret-token-at-least-24-chars";
  EXPECT_NO_THROW(options.validate());
  EXPECT_TRUE(options.read_auth_required());
}

TEST(RestOptions, WriteModeRequiresToken)
{
  RestOptions options;
  options.write_enabled = true;
  EXPECT_THROW(options.validate(), std::invalid_argument);
  options.api_token = "secret-token-at-least-24-chars";
  EXPECT_NO_THROW(options.validate());
}

TEST(AccessPolicy, LoopbackReadsDoNotRequireToken)
{
  AccessPolicy policy(RestOptions{});
  std::string reason;
  EXPECT_TRUE(policy.authorize_read("", reason));
}

TEST(AccessPolicy, WriteRequiresTokenAndExactConfirmation)
{
  RestOptions options;
  options.write_enabled = true;
  options.api_token = "secret-token-at-least-24-chars";
  AccessPolicy policy(options);

  std::string reason;
  EXPECT_FALSE(policy.authorize_write("wrong", "MOVE ROBOT", "MOVE ROBOT", true, reason));
  EXPECT_FALSE(policy.authorize_write(
      "secret-token-at-least-24-chars", "MOVE ROBOT", "move robot", true, reason));
  EXPECT_TRUE(policy.authorize_write(
      "secret-token-at-least-24-chars", "MOVE ROBOT", "MOVE ROBOT", true, reason));
}

TEST(AccessPolicy, PhysicalModeRequiresSecondStartupOptIn)
{
  RestOptions options;
  options.write_enabled = true;
  options.api_token = "secret-token-at-least-24-chars";
  options.robot_mode = RobotMode::kPhysical;
  AccessPolicy policy(options);

  std::string reason;
  EXPECT_FALSE(policy.authorize_write(
      "secret-token-at-least-24-chars", "MOVE ROBOT", "MOVE ROBOT", true, reason));
  EXPECT_NE(reason.find("physical"), std::string::npos);
}

TEST(AccessPolicy, RateLimitIsEnforced)
{
  RestOptions options;
  options.write_enabled = true;
  options.api_token = "secret-token-at-least-24-chars";
  options.max_writes_per_minute = 1U;
  AccessPolicy policy(options);

  std::string reason;
  EXPECT_TRUE(policy.authorize_write("secret-token-at-least-24-chars", "", "", true, reason));
  EXPECT_FALSE(policy.authorize_write("secret-token-at-least-24-chars", "", "", true, reason));
  EXPECT_NE(reason.find("rate limit"), std::string::npos);
  EXPECT_TRUE(policy.authorize_write("secret-token-at-least-24-chars", "", "", false, reason));
}

}  // namespace zhishu_robot_rest_api
