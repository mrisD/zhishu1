#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <sstream>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include "zhishu_robot_mtc_pick_place/cartesian_microstep.hpp"

static moveit::core::RobotModelPtr testModel()
{
  std::ostringstream xml;
  xml << "<robot name='microstep'><link name='base'/>";
  const char* axes[] = {"0 0 1", "0 1 0", "1 0 0", "0 1 0", "1 0 0", "0 1 0", "0 0 1"};
  for (int i = 0; i < 7; ++i) {
    xml << "<link name='l" << i << "'/><joint name='j" << i << "' type='revolute'>"
        << "<parent link='" << (i ? "l" + std::to_string(i-1) : "base") << "'/>"
        << "<child link='l" << i << "'/><origin xyz='0.08 0 0.06'/><axis xyz='" << axes[i]
        << "'/><limit lower='-3' upper='3' effort='100' velocity='1'/></joint>";
  }
  xml << "</robot>";
  auto urdf = urdf::parseURDF(xml.str());
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, "<robot name='microstep'><group name='arm'><chain base_link='base' tip_link='l6'/></group></robot>");
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

TEST(CartesianMicrostep, Executes25MicrometresInsteadOfAcceptingUnchangedSeed)
{
  auto model = testModel();
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.setJointGroupPositions(model->getJointModelGroup("arm"), std::vector<double>{.2, -.4, .3, .6, -.3, .4, -.2});
  state.update();
  const Eigen::Isometry3d initial = state.getGlobalLinkTransform("l6");
  Eigen::Isometry3d target = initial;
  target.translation().z() -= .000025;
  pallet_placement::refineTcpPose(state, model->getJointModelGroup("arm"), "l6", target);
  const Eigen::Isometry3d actual = state.getGlobalLinkTransform("l6");
  EXPECT_NEAR((actual.translation()-initial.translation()).z(), -.000025, 1e-7);
  EXPECT_LT((actual.translation()-target.translation()).norm(), 1e-7);
  EXPECT_LT(Eigen::AngleAxisd(actual.rotation()*target.rotation().transpose()).angle(), 1e-6);
}

TEST(CartesianMicrostep, SupportsUpwardPressureReliefAndCoarseDescent)
{
  auto model = testModel();
  for (double step : {.00005, -.002}) {
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.setJointGroupPositions(model->getJointModelGroup("arm"), std::vector<double>{.2, -.4, .3, .6, -.3, .4, -.2});
    state.update();
    Eigen::Isometry3d target = state.getGlobalLinkTransform("l6");
    target.translation().z() += step;
    pallet_placement::refineTcpPose(state, model->getJointModelGroup("arm"), "l6", target);
    EXPECT_LT((state.getGlobalLinkTransform("l6").translation()-target.translation()).norm(), 1e-7);
  }
}

TEST(CartesianMicrostep, PlannerMustRetainMicrometreMotion)
{
  auto model = testModel();
  const auto* group = model->getJointModelGroup("arm");
  moveit::core::RobotState start(model);
  start.setToDefaultValues();
  start.setJointGroupPositions(group, std::vector<double>{.2, -.4, .3, .6, -.3, .4, -.2});
  start.update();
  Eigen::Isometry3d target = start.getGlobalLinkTransform("l6");
  target.translation().z() -= .000025;
  auto goal = start;
  pallet_placement::refineTcpPose(goal, group, "l6", target);
  auto from = std::make_shared<planning_scene::PlanningScene>(model);
  from->setCurrentState(start);
  auto to = from->diff();
  to->setCurrentState(goal);
  moveit::task_constructor::solvers::JointInterpolationPlanner planner;
  planner.setProperty("goal_joint_tolerance", 1e-9);
  planner.setProperty("max_step", 1e-5);
  planner.setTimeParameterization(std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>(1e-8, .02, 1e-9));
  planner.init(model);
  robot_trajectory::RobotTrajectoryPtr result;
  ASSERT_TRUE(planner.plan(from, to, group, 1.0, result));
  ASSERT_TRUE(result);
  EXPECT_GE(result->getWayPointCount(), 2u);
  EXPECT_LT((result->getLastWayPoint().getGlobalLinkTransform("l6").translation()-target.translation()).norm(), 1e-7);
}

TEST(CartesianMicrostep, TimeParameterizationMustNotDropTinyJointChange)
{
  auto model = testModel();
  const auto* group = model->getJointModelGroup("arm");
  moveit::core::RobotState start(model);
  start.setToDefaultValues();
  start.update();
  auto goal = start;
  goal.setVariablePosition("j0", 1e-5);
  goal.update();
  auto from = std::make_shared<planning_scene::PlanningScene>(model);
  from->setCurrentState(start);
  auto to = from->diff();
  to->setCurrentState(goal);
  moveit::task_constructor::solvers::JointInterpolationPlanner planner;
  planner.setTimeParameterization(std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>(1e-8, .02, 1e-9));
  planner.init(model);
  robot_trajectory::RobotTrajectoryPtr result;
  ASSERT_TRUE(planner.plan(from, to, group, 1.0, result));
  ASSERT_TRUE(result);
  EXPECT_GE(result->getWayPointCount(), 2u);
  EXPECT_NEAR(result->getLastWayPoint().getVariablePosition("j0"), 1e-5, 1e-10);
}

static moveit::core::RobotModelPtr boundedRedundantModel()
{
  std::ostringstream xml;
  xml<<"<robot name='bounded'><link name='base'/>";
  const char* axes[]={"1 0 0","0 1 0","0 0 1","1 0 0","0 1 0","0 0 1","1 0 0"};
  for(int i=0;i<7;++i) {
    const bool linear=i<3 || i==6;
    const double bound=i==0?.1:(linear?1.0:3.0);
    xml<<"<link name='b"<<i<<"'/><joint name='q"<<i<<"' type='"<<(linear?"prismatic":"revolute")<<"'>"
      <<"<parent link='"<<(i?"b"+std::to_string(i-1):"base")<<"'/><child link='b"<<i<<"'/>"
      <<"<axis xyz='"<<axes[i]<<"'/><limit lower='"<<-bound<<"' upper='"<<bound<<"' effort='100' velocity='1'/></joint>";
  }
  xml<<"</robot>";
  auto urdf=urdf::parseURDF(xml.str()); auto semantic=std::make_shared<srdf::Model>();
  semantic->initString(*urdf,"<robot name='bounded'><group name='arm'><chain base_link='base' tip_link='b6'/></group></robot>");
  return std::make_shared<moveit::core::RobotModel>(urdf,semantic);
}

TEST(CartesianMicrostep, RedundantJointCompletesMotionWithoutCrossingLimit)
{
  auto model=boundedRedundantModel(); moveit::core::RobotState state(model);
  state.setToDefaultValues(); state.setVariablePosition("q0",.099); state.update();
  auto goal=state.getGlobalLinkTransform("b6"); goal.translation().x()+=.02;
  ASSERT_NO_THROW(pallet_placement::refineTcpPose(state,model->getJointModelGroup("arm"),"b6",goal));
  EXPECT_TRUE(state.satisfiesBounds()); EXPECT_LE(state.getVariablePosition("q0"),.1);
  EXPECT_GT(state.getVariablePosition("q6"),.018);
  EXPECT_LT((state.getGlobalLinkTransform("b6").translation()-goal.translation()).norm(),1e-7);
}

TEST(CartesianMicrostep, BoundedBranchStillRejectsUnreachableTarget)
{
  auto model=boundedRedundantModel(); moveit::core::RobotState state(model);
  state.setToDefaultValues(); state.update(); auto goal=state.getGlobalLinkTransform("b6");
  goal.translation().x()+=.5;
  EXPECT_THROW(pallet_placement::refineTcpPose(state,model->getJointModelGroup("arm"),"b6",goal),std::runtime_error);
  EXPECT_TRUE(state.satisfiesBounds());
}
