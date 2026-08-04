#include "zhishu_robot_mtc_pick_place/alternative_path_costs.hpp"

#include <rclcpp/rclcpp.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/cost_terms.h>

#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

using namespace moveit::task_constructor;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::cout << "ROS2 initialized successfully!" << std::endl;

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rclcpp::Node>(
        "alternative_path_costs_node",
        node_options);

    std::cout << "Node created successfully!" << std::endl;

    std::thread spinning_thread([node]() {
        rclcpp::spin(node);
    });

    Task t;
    t.stages()->setName("alternative path costs");

    // 如果 RViz Motion Planning Tasks 看不到解，可以打开 introspection
    // t.enableIntrospection();

    std::cout << "Task created successfully!" << std::endl;

    t.loadRobotModel(node, "robot_description");

    std::cout << "Loaded robot model: "
              << t.getRobotModel()->getName()
              << std::endl;

    assert(t.getRobotModel()->getName() == "zhishu_robot");

    const auto* left_arm_jmg =
        t.getRobotModel()->getJointModelGroup("left_arm");

    if (!left_arm_jmg) {
        std::cerr << "ERROR: JointModelGroup 'left_arm' not found!" << std::endl;
        rclcpp::shutdown();
        spinning_thread.join();
        return 1;
    }

    // ============================================================
    // 起点：left_arm_home
    // ============================================================
    auto home_scene =
        std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());

    auto& home_state = home_scene->getCurrentStateNonConst();
    home_state.setToDefaultValues();
    home_state.setToDefaultValues(left_arm_jmg, "left_arm_home");

    auto initial = std::make_unique<stages::FixedState>("start_left_arm_home");
    initial->setState(home_scene);
    t.add(std::move(initial));

    std::cout << "Added start state: left_arm_home" << std::endl;

    // ============================================================
    // Planner：这里设置速度/加速度缩放
    // 0.2 表示使用 joint_limits 的 20%
    // 如果还觉得快，可以改成 0.1 或 0.05
    // ============================================================
    auto pipeline = std::make_shared<solvers::PipelinePlanner>(node);

    pipeline->setMaxVelocityScalingFactor(0.5);
    pipeline->setMaxAccelerationScalingFactor(0.5);

    std::cout << "Pipeline planner created successfully" << std::endl;
    std::cout << "Velocity scaling: 0.2" << std::endl;
    std::cout << "Acceleration scaling: 0.2" << std::endl;

    // ============================================================
    // 第一段：left_arm_home -> left_arm_pose1
    // 使用 Alternatives，保留你原来的 4 种 cost 策略
    // ============================================================
    auto alternatives = std::make_unique<Alternatives>("connect_home_to_pose1");

    std::cout << "Alternatives container created successfully!" << std::endl;

    // Strategy 1: Minimize path length
    {
        auto connect = std::make_unique<stages::Connect>(
            "path length",
            stages::Connect::GroupPlannerVector{{"left_arm", pipeline}});

        connect->setCostTerm(std::make_unique<cost::PathLength>());
        alternatives->add(std::move(connect));

        std::cout << "Added 'path length' strategy" << std::endl;
    }

    // Strategy 2: Minimize trajectory duration
    {
        auto connect = std::make_unique<stages::Connect>(
            "trajectory duration",
            stages::Connect::GroupPlannerVector{{"left_arm", pipeline}});

        connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
        alternatives->add(std::move(connect));

        std::cout << "Added 'trajectory duration' strategy" << std::endl;
    }

    // Strategy 3: Minimize end-effector motion
    {
        auto connect = std::make_unique<stages::Connect>(
            "end-effector motion",
            stages::Connect::GroupPlannerVector{{"left_arm", pipeline}});

        connect->setCostTerm(std::make_unique<cost::LinkMotion>("link5"));
        alternatives->add(std::move(connect));

        std::cout << "Added 'end-effector motion' strategy" << std::endl;
    }

    // Strategy 4: Minimize elbow motion
    {
        auto connect = std::make_unique<stages::Connect>(
            "elbow motion",
            stages::Connect::GroupPlannerVector{{"left_arm", pipeline}});

        connect->setCostTerm(std::make_unique<cost::LinkMotion>("link4"));
        alternatives->add(std::move(connect));

        std::cout << "Added 'elbow motion' strategy" << std::endl;
    }

    t.add(std::move(alternatives));

    std::cout << "Added connect stage: left_arm_home -> left_arm_pose1" << std::endl;

    // ============================================================
    // 中间点：left_arm_pose1
    // ============================================================
    auto pose1_scene = home_scene->diff();

    pose1_scene->getCurrentStateNonConst().setToDefaultValues(
        left_arm_jmg,
        "left_arm_pose1");

    auto pose1_state =
        std::make_unique<stages::FixedState>("via_left_arm_pose1");

    pose1_state->setState(pose1_scene);
    t.add(std::move(pose1_state));

    std::cout << "Added via state: left_arm_pose1" << std::endl;

    // ============================================================
    // 第二段：left_arm_pose1 -> left_arm_home
    // 这段回 home 也使用同一个 pipeline，所以同样是 0.2 低速
    // ============================================================
    {
        auto return_home = std::make_unique<stages::Connect>(
            "return_to_left_arm_home_slow",
            stages::Connect::GroupPlannerVector{{"left_arm", pipeline}});

        return_home->setCostTerm(
            std::make_unique<cost::TrajectoryDuration>());

        t.add(std::move(return_home));

        std::cout << "Added return connect: left_arm_pose1 -> left_arm_home" << std::endl;
    }

    // ============================================================
    // 最终目标：left_arm_home
    // 这样整条 solution 的结束点又回到 home
    // ============================================================
    auto final_home_scene = home_scene->diff();

    final_home_scene->getCurrentStateNonConst().setToDefaultValues(
        left_arm_jmg,
        "left_arm_home");

    auto final_home =
        std::make_unique<stages::FixedState>("goal_left_arm_home");

    final_home->setState(final_home_scene);
    t.add(std::move(final_home));

    std::cout << "Added final goal state: left_arm_home" << std::endl;

    // ============================================================
    // Planning
    // ============================================================
    std::cout << "Planning..." << std::endl;

    try {
        auto result = t.plan(0);
        (void)result;

        if (t.numSolutions() == 0) {
            std::cout << "Task planning finished, but no solutions found." << std::endl;
            t.printState();

            rclcpp::shutdown();
            spinning_thread.join();
            return 1;
        }

        std::cout << "Task planning completed successfully" << std::endl;
        std::cout << "Solution count: " << t.numSolutions() << std::endl;

        std::cout << "Planning results:" << std::endl;
        t.printState();

        // 给 RViz 一点订阅时间
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // 发布所有方案给 RViz Motion Planning Tasks
        t.introspection().publishAllSolutions(true);

        std::cout << "Published all MTC solutions to RViz /solution topic" << std::endl;
    }
    catch (const InitStageException& e) {
        std::cout << "Task planning failed: " << e << std::endl;
    }
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
    }

    std::cout << "Keeping node alive for RViz inspection. Press Ctrl+C to exit." << std::endl;

    spinning_thread.join();

    return 0;
}