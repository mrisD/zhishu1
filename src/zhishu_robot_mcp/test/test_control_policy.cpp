#include "zhishu_robot_mcp/control_policy.hpp"

#include <gtest/gtest.h>

#include <string>

namespace zhishu_robot_mcp
{

TEST(ControlPolicy, WritesAreDisabledByDefault)
{
  ControlPolicy policy(ServerOptions{});
  std::string reason;
  EXPECT_FALSE(policy.authorize("MOVE ROBOT", "MOVE ROBOT", true, reason));
  EXPECT_NE(reason.find("disabled"), std::string::npos);
}

TEST(ControlPolicy, PhysicalModeRequiresSecondStartupOptIn)
{
  ServerOptions options;
  options.write_enabled = true;
  options.robot_mode = RobotMode::kPhysical;
  options.allow_physical_hardware = false;
  ControlPolicy policy(options);

  std::string reason;
  EXPECT_FALSE(policy.authorize("MOVE ROBOT", "MOVE ROBOT", true, reason));
  EXPECT_NE(reason.find("physical"), std::string::npos);
}

TEST(ControlPolicy, ConfirmationMustMatchExactly)
{
  ServerOptions options;
  options.write_enabled = true;
  ControlPolicy policy(options);

  std::string reason;
  EXPECT_FALSE(policy.authorize("MOVE ROBOT", "move robot", true, reason));
  EXPECT_TRUE(policy.authorize("MOVE ROBOT", "MOVE ROBOT", true, reason));
}

TEST(ControlPolicy, WriteRateLimitIsEnforced)
{
  ServerOptions options;
  options.write_enabled = true;
  options.max_writes_per_minute = 1U;
  ControlPolicy policy(options);

  std::string reason;
  EXPECT_TRUE(policy.authorize("", "", true, reason));
  EXPECT_FALSE(policy.authorize("", "", true, reason));
  EXPECT_NE(reason.find("rate limit"), std::string::npos);
  EXPECT_TRUE(policy.authorize("", "", false, reason));
}

}  // namespace zhishu_robot_mcp
