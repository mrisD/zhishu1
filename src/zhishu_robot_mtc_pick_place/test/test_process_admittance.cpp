#include <gtest/gtest.h>
#include <limits>
#include "zhishu_robot_mtc_pick_place/process_admittance.hpp"
using pallet_placement::ProcessAdmittance;
TEST(ProcessAdmittance, ZeroDurationAcceptsOnlyFiniteStationaryPoints) {
  using pallet_placement::stationaryZeroDuration;
  EXPECT_TRUE(stationaryZeroDuration({0,0},{{.06},{.06}}));
  EXPECT_FALSE(stationaryZeroDuration({0,0},{{.06},{.05}}));
  EXPECT_FALSE(stationaryZeroDuration({0,-1},{{.06},{.06}}));
  EXPECT_FALSE(stationaryZeroDuration({0},{{std::numeric_limits<double>::quiet_NaN()}}));
  EXPECT_FALSE(stationaryZeroDuration({0,0},{{.06},{.06,.01}}));
  EXPECT_FALSE(stationaryZeroDuration({},{}));
}
TEST(ProcessAdmittance, ForceChangesCommandAndSaturatesWithoutWindup) {
  ProcessAdmittance a; std::array<double,6> f{40,0,0,0,0,0};
  a.advance(f,.25); EXPECT_GT(a.position[0],0); EXPECT_LE(a.position[0],.0005);
  for(int i=0;i<200;++i) a.advance(f,.25);
  EXPECT_DOUBLE_EQ(a.position[0],.0005); EXPECT_DOUBLE_EQ(a.velocity[0],0);
  f[0]=-40; a.advance(f,.25); EXPECT_LT(a.position[0],.0005);
}
TEST(ProcessAdmittance, TorqueAndDeadbandAreIndependent) {
  ProcessAdmittance a; a.advance({1,2,3,0,0,2},.25);
  EXPECT_DOUBLE_EQ(a.position[0],0); EXPECT_DOUBLE_EQ(a.position[2],0);
  EXPECT_GT(a.position[5],0); EXPECT_DOUBLE_EQ(a.position[4],0);
}
TEST(ProcessAdmittance, RejectsNonFiniteFeedback) {
  ProcessAdmittance a; std::array<double,6> f{}; f[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(a.advance(f,.25),std::invalid_argument);
}

TEST(ProcessAdmittance, CarryRejectsDefaultArmReferenceInsteadOfMeasuredPose) {
  using pallet_placement::carryStepSafe;
  EXPECT_FALSE(carryStepSafe(2.4,0.0,1.0,.25));
  EXPECT_FALSE(carryStepSafe(-1.5,0.0,1.0,.25));
  EXPECT_TRUE(carryStepSafe(2.4,2.401,1.0,.25));
  EXPECT_FALSE(carryStepSafe(2.4,2.43,1.0,.25));
  EXPECT_FALSE(carryStepSafe(2.4,std::numeric_limits<double>::quiet_NaN(),1.0,.25));
}

TEST(ProcessAdmittance, SuppressesLargeNullspaceJointChangeNearSingularity) {
  using pallet_placement::admittanceCorrectionSafe;
  EXPECT_FALSE(admittanceCorrectionSafe(-.1909965,-.1585602));
  EXPECT_FALSE(admittanceCorrectionSafe(.1701847,.1379425));
  EXPECT_TRUE(admittanceCorrectionSafe(-.1909965,-.1899965));
  EXPECT_FALSE(admittanceCorrectionSafe(0,std::numeric_limits<double>::quiet_NaN()));
}
