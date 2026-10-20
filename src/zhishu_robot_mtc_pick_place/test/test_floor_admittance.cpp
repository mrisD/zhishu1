#include <gtest/gtest.h>
#include <limits>
#include "zhishu_robot_mtc_pick_place/floor_admittance.hpp"

using namespace pallet_placement;

TEST(FloorAdmittance, RejectsPreviouslyAcceptedDownwardPreload)
{
  EXPECT_FALSE(releaseReady(true, true, 243.258, 0.00014, 0.054));
  EXPECT_GT(verticalStep(243.258, 0.00014), 0.0);
}
TEST(FloorAdmittance, RequiresPreloadUnloadingAndFreshSupport)
{
  EXPECT_FALSE(releaseReady(false, true, kWeight, 0.0, 0.0));
  EXPECT_FALSE(releaseReady(true, false, kWeight, 0.0, 0.0));
  EXPECT_TRUE(releaseReady(true, true, kWeight, 0.0, 0.0));
  EXPECT_FALSE(releaseReady(true, true, 80.0, 0.0, 0.0));
}
TEST(FloorAdmittance, RejectsTiltAndIncorrectHeight)
{
  EXPECT_FALSE(releaseReady(true, true, kWeight, 0.002, 0.0));
  EXPECT_FALSE(releaseReady(true, true, kWeight, 0.0, 0.6));
}
TEST(FloorAdmittance, ApproachesFloorSlowlyAndRelievesOverload)
{
  EXPECT_DOUBLE_EQ(verticalStep(0.0, 0.02), -0.002);
  EXPECT_DOUBLE_EQ(verticalStep(0.0, 0.001), -0.00025);
  EXPECT_LT(verticalStep(0.0, 0.0001), 0.0);
  EXPECT_GE(verticalStep(0.0, 0.0001), -0.00005);
  EXPECT_DOUBLE_EQ(verticalStep(kWeight, 0.0), 0.0);
  EXPECT_GT(verticalStep(250.0, 0.0), 0.0);
  EXPECT_LE(verticalStep(250.0, 0.0), 0.00005);
}
TEST(FloorAdmittance, FailsClosedOnNonFiniteFeedback)
{
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(releaseReady(true, true, nan, 0.0, 0.0));
  EXPECT_THROW(verticalStep(nan, 0.0), std::invalid_argument);
}

TEST(FloorAdmittance, IntegratesMicrostepsAndBoundsContactCompression)
{
  double command = 0.0;
  for (int i = 0; i < 10; ++i) command = integrateVerticalReference(command, -.000025, .0004);
  EXPECT_NEAR(command, -.00025, 1e-12);
  EXPECT_DOUBLE_EQ(integrateVerticalReference(-.00089, -.00005, .0004), -.0009);
  EXPECT_DOUBLE_EQ(integrateVerticalReference(.00199, .00005, .0004), .002);
}
