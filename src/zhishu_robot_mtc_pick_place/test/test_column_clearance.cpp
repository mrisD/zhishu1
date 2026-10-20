#include <gtest/gtest.h>
#include "zhishu_robot_mtc_pick_place/column_clearance.hpp"

TEST(ColumnClearance, RaisingBodyCannotSubstituteForHorizontalGap)
{
  auto points = pallet_placement::envelopeCorners({-.11,-.11,0}, {.11,.11,.54});
  auto tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(.54,-1.55,.3);
  EXPECT_NEAR(pallet_placement::horizontalTableGap(points, tf), .005, 1e-9);
  tf.translation().z() += 2;
  EXPECT_NEAR(pallet_placement::horizontalTableGap(points, tf), .005, 1e-9);
}

TEST(ColumnClearance, LeaningUpperBodyMustAlsoStayOutside200Millimetres)
{
  auto points = pallet_placement::envelopeCorners({-.119,-.182,-.065}, {.154,.181,.608});
  auto tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(.55,-1.55,1.0);
  tf.linear() = Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  EXPECT_LT(pallet_placement::horizontalTableGap(points, tf), .2);
  tf.translation().x() = .16;
  EXPECT_GE(pallet_placement::horizontalTableGap(points, tf), .22);
}

TEST(ColumnClearance, DistanceUsesNearestTableEdgeAndCorner)
{
  std::vector<Eigen::Vector3d> point{{0,0,0}};
  auto tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(.505,-2.225,0);
  EXPECT_NEAR(pallet_placement::horizontalTableGap(point, tf), .25, 1e-9);
  tf.translation() = Eigen::Vector3d(1.0,-1.55,10);
  EXPECT_DOUBLE_EQ(pallet_placement::horizontalTableGap(point, tf), 0);
}

TEST(ColumnClearance, MissingGeometryFailsClosed)
{
  EXPECT_LT(pallet_placement::horizontalTableGap({}, Eigen::Isometry3d::Identity()), 0);
}
