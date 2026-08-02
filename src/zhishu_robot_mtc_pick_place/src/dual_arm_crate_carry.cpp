#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace mtc = moveit::task_constructor;

class DualArmCrateCarry : public rclcpp::Node
{
public:
  explicit DualArmCrateCarry(const rclcpp::NodeOptions& options)
  : Node("dual_arm_crate_carry", options)
  {
    // NodeOptions enables automatically_declare_parameters_from_overrides so
    // that the complete MoveIt configuration passed by the launch file is
    // available. Parameters explicitly present in the launch file are already
    // declared at this point; only declare defaults that are still missing.
    const auto declare_if_missing = [this](const std::string& name, const auto& default_value) {
      if (!this->has_parameter(name)) {
        this->declare_parameter(name, default_value);
      }
    };

    declare_if_missing("world_frame", std::string("base_link"));
    declare_if_missing("crate_x", 0.48);
    declare_if_missing("crate_y", 0.0);
    declare_if_missing("crate_z", 0.72);
    declare_if_missing("crate_size_x", 0.14);
    declare_if_missing("crate_size_y", 0.12);
    declare_if_missing("crate_size_z", 0.14);
    declare_if_missing("wall_thickness", 0.006);
    declare_if_missing("move_x", 0.0);
    declare_if_missing("move_y", 0.0);
    declare_if_missing("move_z", 0.30);
    // Vertical distance from each final grasp point to its overhead
    // pregrasp/retreat point.
    declare_if_missing("pregrasp_clearance", 0.14);
    // Offset of the TCP centre from the side-wall centre. Zero puts the wall
    // exactly between the two fingers.
    declare_if_missing("grasp_clearance", 0.0);
    // Grip the side-wall rim this distance below the crate's top edge.
    declare_if_missing("rim_grasp_depth", 0.02);
    // The local TCP X axes are the finger-closing axes. Point them toward
    // +/-Y (the two side-wall normals). The unequal rolls keep the fingertips
    // angled downward while remaining reachable throughout the 0.30 m lift.
    declare_if_missing("left_tcp_roll", -2.09439510239);
    declare_if_missing("left_tcp_pitch", 0.0);
    declare_if_missing("left_tcp_yaw", 1.57079632679);
    declare_if_missing("right_tcp_roll", -1.04719755120);
    declare_if_missing("right_tcp_pitch", 0.0);
    declare_if_missing("right_tcp_yaw", -1.57079632679);
    declare_if_missing("carry_steps", 15);
    declare_if_missing("execute", false);
    declare_if_missing("max_solutions", 5);
    // PipelinePlanner defaults to one sampling-planner attempt.  This task
    // contains several dual-arm OMPL stages, so one unlucky RRT seed must not
    // fail the whole task.
    declare_if_missing("planning_attempts", 8);
    declare_if_missing("task_attempts", 3);
    declare_if_missing("stage_timeout", 15.0);
    declare_if_missing("state_wait_timeout", 30.0);
    declare_if_missing("planner_id", std::string("RRTConnectkConfigDefault"));
    // Use a separate, lower speed for the first arm motion from the current
    // robot state to the overhead pregrasp pose.
    declare_if_missing("initial_velocity_scaling", 0.05);
    declare_if_missing("initial_acceleration_scaling", 0.05);
    declare_if_missing("velocity_scaling", 0.20);
    declare_if_missing("acceleration_scaling", 0.20);
    // Keep this false by default so an over-conservative IK pre-filter cannot
    // prevent the task from being published to RViz. MTC and MoveIt still
    // validate the planned stages with the real planning scene.
    declare_if_missing("strict_ik_collision_filter", false);
  }

  void run()
  {
    waitForCurrentRobotState();
    RCLCPP_INFO(get_logger(), "Adding the hollow crate to the planning scene");
    addHollowCrate();

    const auto max_solutions_parameter = get_parameter("max_solutions").as_int();
    const auto task_attempts_parameter = get_parameter("task_attempts").as_int();
    if (max_solutions_parameter < 1 || max_solutions_parameter > 100) {
      throw std::runtime_error("max_solutions must be between 1 and 100");
    }
    if (task_attempts_parameter < 1 || task_attempts_parameter > 10) {
      throw std::runtime_error("task_attempts must be between 1 and 10");
    }
    const auto max_solutions = static_cast<std::size_t>(max_solutions_parameter);
    const auto task_attempts = static_cast<std::size_t>(task_attempts_parameter);

    bool planned = false;
    for (std::size_t attempt = 1; attempt <= task_attempts; ++attempt) {
      RCLCPP_INFO(
        get_logger(),
        "Creating and planning dual-arm MTC task (task attempt %zu/%zu)",
        attempt, task_attempts);
      try {
        task_ = createTask();
        // Task::plan() initializes the task internally. Calling init()
        // separately initializes every stage twice.
        const auto result = task_.plan(max_solutions);
        if (result && task_.numSolutions() > 0) {
          planned = true;
          break;
        }
        RCLCPP_WARN(
          get_logger(),
          "MTC task attempt %zu/%zu failed (MoveIt error code %d)",
          attempt, task_attempts, result.val);
      } catch (const mtc::InitStageException& e) {
        RCLCPP_ERROR(
          get_logger(), "MTC initialization failed on attempt %zu/%zu: %s",
          attempt, task_attempts, e.what());
      } catch (const std::exception& e) {
        RCLCPP_WARN(
          get_logger(), "MTC task attempt %zu/%zu failed while generating goals: %s",
          attempt, task_attempts, e.what());
      }
    }

    if (!planned) {
      RCLCPP_ERROR(
        get_logger(),
        "No dual-arm crate-carry solution found after %zu complete task attempts",
        task_attempts);
      task_.printState();
      return;
    }

    task_.introspection().publishSolution(*task_.solutions().front());
    RCLCPP_INFO(
      get_logger(), "Planning succeeded with %zu solution(s)", task_.numSolutions());

    if (get_parameter("execute").as_bool()) {
      const auto result = task_.execute(*task_.solutions().front());
      if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Execution failed, error code: %d", result.val);
      } else {
        RCLCPP_INFO(get_logger(), "Dual-arm crate carry completed");
      }
    }
  }

private:
  void waitForCurrentRobotState()
  {
    const double timeout = get_parameter("state_wait_timeout").as_double();
    if (timeout <= 0.0 || timeout > 300.0) {
      throw std::runtime_error(
        "state_wait_timeout must be greater than 0 and at most 300 seconds");
    }

    RCLCPP_INFO(
      get_logger(),
      "Waiting up to %.1f s for a complete both_arms joint state",
      timeout);
    moveit::planning_interface::MoveGroupInterface move_group(
      shared_from_this(), "both_arms", std::shared_ptr<tf2_ros::Buffer>(),
      rclcpp::Duration::from_seconds(timeout));
    const auto state = move_group.getCurrentState(timeout);
    if (!state) {
      throw std::runtime_error(
        "Timed out waiting for a complete both_arms joint state. "
        "Check joint_state_broadcaster and /joint_states before planning.");
    }

    const auto* group = state->getJointModelGroup("both_arms");
    if (!group) {
      throw std::runtime_error("Robot model does not contain the both_arms group");
    }
    std::vector<double> positions;
    state->copyJointGroupPositions(group, positions);
    if (positions.empty() ||
        !std::all_of(positions.begin(), positions.end(), [](double value) {
          return std::isfinite(value);
        })) {
      throw std::runtime_error(
        "Received an incomplete or non-finite both_arms joint state");
    }
    RCLCPP_INFO(
      get_logger(), "Complete current state received for %zu arm joints",
      positions.size());
  }

  static geometry_msgs::msg::Pose makePose(
    double x, double y, double z, const std::vector<double>& rpy)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  static Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose& pose)
  {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(
      pose.position.x, pose.position.y, pose.position.z);
    const Eigen::Quaterniond q(
      pose.orientation.w, pose.orientation.x,
      pose.orientation.y, pose.orientation.z);
    transform.linear() = q.normalized().toRotationMatrix();
    return transform;
  }

  static geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d& transform)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    const Eigen::Quaterniond q(transform.rotation());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  static void appendBox(
    moveit_msgs::msg::CollisionObject& object, const std::vector<double>& dimensions,
    double x, double y, double z)
  {
    if (dimensions.size() != 3) {
      throw std::runtime_error("A box primitive requires exactly three dimensions");
    }
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    // ROS 2 Humble defines dimensions as BoundedVector<double, 3>,
    // which cannot be assigned directly from std::vector<double>.
    box.dimensions.resize(3);
    box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = dimensions[0];
    box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = dimensions[1];
    box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = dimensions[2];
    object.primitives.push_back(box);
    object.primitive_poses.push_back(makePose(x, y, z, {0.0, 0.0, 0.0}));
  }

  void addHollowCrate()
  {
    const auto frame = get_parameter("world_frame").as_string();
    const std::vector<double> p{
      get_parameter("crate_x").as_double(),
      get_parameter("crate_y").as_double(),
      get_parameter("crate_z").as_double()};
    const std::vector<double> s{
      get_parameter("crate_size_x").as_double(),
      get_parameter("crate_size_y").as_double(),
      get_parameter("crate_size_z").as_double()};
    const double t = get_parameter("wall_thickness").as_double();
    if (s.size() != 3 || t <= 0.0 ||
        s[0] <= 2.0 * t || s[1] <= 2.0 * t || s[2] <= t) {
      throw std::runtime_error("Invalid crate_size/wall_thickness parameters");
    }

    moveit_msgs::msg::CollisionObject crate;
    crate.header.frame_id = frame;
    crate.id = "plastic_hollow_crate";
    crate.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Bottom plus four thin walls. The top remains open, matching a plastic tote.
    appendBox(crate, {s[0], s[1], t}, p[0], p[1], p[2] - 0.5 * s[2] + 0.5 * t);
    appendBox(crate, {s[0], t, s[2]}, p[0], p[1] + 0.5 * (s[1] - t), p[2]);
    appendBox(crate, {s[0], t, s[2]}, p[0], p[1] - 0.5 * (s[1] - t), p[2]);
    appendBox(crate, {t, s[1] - 2.0 * t, s[2]}, p[0] + 0.5 * (s[0] - t), p[1], p[2]);
    appendBox(crate, {t, s[1] - 2.0 * t, s[2]}, p[0] - 0.5 * (s[0] - t), p[1], p[2]);

    moveit::planning_interface::PlanningSceneInterface psi;
    if (!psi.applyCollisionObject(crate)) {
      throw std::runtime_error("Failed to add hollow crate to the planning scene");
    }
  }

  std::map<std::string, double> solveDualArmGoal(
    const geometry_msgs::msg::Pose& left_pose, const geometry_msgs::msg::Pose& right_pose,
    const moveit::core::RobotState& seed,
    const moveit::core::RobotModelConstPtr& model) const
  {
    const auto* left = model->getJointModelGroup("left_arm");
    const auto* right = model->getJointModelGroup("right_arm");
    const auto* both = model->getJointModelGroup("both_arms");
    if (!left || !right || !both) {
      throw std::runtime_error("SRDF must contain left_arm, right_arm and both_arms groups");
    }

    // The KDL chains for left_arm/right_arm end at L_hand_link/R_hand_link.
    // The gripper TCP links belong to the separate end-effector groups, so a
    // TCP target must first be converted into the corresponding hand target.
    moveit::core::RobotState reference(seed);
    reference.updateLinkTransforms();
    const Eigen::Isometry3d left_hand_to_tcp =
      reference.getGlobalLinkTransform("L_hand_link").inverse() *
      reference.getGlobalLinkTransform("left_gripper_tcp");
    const Eigen::Isometry3d right_hand_to_tcp =
      reference.getGlobalLinkTransform("R_hand_link").inverse() *
      reference.getGlobalLinkTransform("right_gripper_tcp");
    const auto left_hand_pose = eigenToPose(
      poseToEigen(left_pose) * left_hand_to_tcp.inverse());
    const auto right_hand_pose = eigenToPose(
      poseToEigen(right_pose) * right_hand_to_tcp.inverse());

    // A pose can have several IK branches. Some satisfy the pose constraints
    // but put a wrist through the waist or the opposite arm. Search multiple
    // seeds and retain only a bounded, self-collision-free dual-arm state.
    planning_scene::PlanningScene self_collision_scene(model);
    moveit::core::RobotState state(seed);
    bool found_valid_state = false;
    bool found_bounded_state = false;
    moveit::core::RobotState first_bounded_state(seed);
    std::size_t ik_solved_count = 0;
    std::size_t bounded_count = 0;
    std::size_t collision_rejected_count = 0;
    const bool strict_collision_filter =
      get_parameter("strict_ik_collision_filter").as_bool();
    struct MirrorJoint
    {
      const char* left_name;
      const char* right_name;
      double sign;
    };
    // Mirror relationship verified from the left/right URDF kinematic chains.
    const MirrorJoint mirror_joints[] = {
      {"left_shoulder_pitch_joint", "right_shoulder_pitch_joint", -1.0},
      {"left_shoulder_roll_joint", "right_shoulder_roll_joint", -1.0},
      {"left_shoulder_yaw_joint", "right_shoulder_yaw_joint", -1.0},
      {"left_elbow_pitch_joint", "right_elbow_pitch_joint", 1.0},
      {"left_wrist_yaw_joint", "right_wrist_yaw_joint", -1.0},
      {"left_wrist_pitch_joint", "right_wrist_pitch_joint", 1.0},
      {"left_wrist_roll_joint", "right_wrist_roll_joint", -1.0}};
    constexpr double max_mirror_error = 0.10;
    std::size_t symmetry_rejected_count = 0;    
    constexpr std::size_t max_ik_attempts = 80;
    for (std::size_t attempt = 0; attempt < max_ik_attempts; ++attempt) {
      moveit::core::RobotState candidate(seed);
      if (attempt > 0) {
        // Randomize only one arm. The other arm is seeded from its exact
        // mirror so both IK solvers stay on corresponding branches.
        candidate.setToRandomPositions(left);
      }
      candidate.update();

      if (!candidate.setFromIK(left, left_hand_pose, "L_hand_link", 0.15)) {
        continue;
      }
      for (const auto& joint : mirror_joints) {
        candidate.setVariablePosition(
          joint.right_name,
          joint.sign * candidate.getVariablePosition(joint.left_name));
      }
      candidate.update();
      if (!candidate.setFromIK(right, right_hand_pose, "R_hand_link", 0.15)) {
        continue;
      }
      candidate.update();
      ++ik_solved_count;
      double mirror_error = 0.0;
      for (const auto& joint : mirror_joints) {
        const double expected =
          joint.sign * candidate.getVariablePosition(joint.left_name);
        mirror_error = std::max(
          mirror_error,
          std::abs(candidate.getVariablePosition(joint.right_name) - expected));
      }
      if (mirror_error > max_mirror_error) {
        ++symmetry_rejected_count;
        continue;
      }

      if (!candidate.satisfiesBounds(both)) {
        continue;
      }
      ++bounded_count;
      if (!found_bounded_state) {
        first_bounded_state = candidate;
        found_bounded_state = true;
      }
      if (self_collision_scene.isStateColliding(candidate, "both_arms", false)) {
        ++collision_rejected_count;
        continue;
      }

      state = candidate;
      found_valid_state = true;
      RCLCPP_INFO(
        get_logger(),
        "Found mirrored collision-free dual-arm IK branch on attempt %zu "
        "(max joint mirror error=%.4f rad)",
        attempt + 1, mirror_error);
        break;
    }

    if (!found_valid_state && found_bounded_state && !strict_collision_filter) {
      state = first_bounded_state;
      found_valid_state = true;
      RCLCPP_WARN(
        get_logger(),
        "All bounded IK branches were rejected by the pre-filter "
        "(dual IK solved=%zu, symmetry rejected=%zu, bounded=%zu, "
        "collision rejected=%zu). "
        "Using the first bounded IK branch so MTC can publish the task tree; "
        "set strict_ik_collision_filter:=true to restore hard rejection.",
        ik_solved_count, symmetry_rejected_count, bounded_count,
        collision_rejected_count);
    }

    if (!found_valid_state) {
      throw std::runtime_error(
        "No collision-free dual-arm IK branch found after 80 attempts (dual IK solved=" +
        std::to_string(ik_solved_count) + ", symmetry rejected=" +
        std::to_string(symmetry_rejected_count) + ", bounded=" +
        std::to_string(bounded_count) + ", collision rejected=" +
        std::to_string(collision_rejected_count) + ")");
    }

    std::map<std::string, double> goal;
    for (const auto& name : both->getActiveJointModelNames()) {
      goal[name] = state.getVariablePosition(name);
    }
    return goal;
  }

  mtc::Task createTask()
  {
    mtc::Task task;
    task.stages()->setName("dual arm carry hollow crate");
    task.loadRobotModel(shared_from_this());

    auto current = std::make_unique<mtc::stages::CurrentState>("current state");
    task.add(std::move(current));
    const auto planning_attempts_parameter = get_parameter("planning_attempts").as_int();
    const double stage_timeout = get_parameter("stage_timeout").as_double();
    if (planning_attempts_parameter < 1 || planning_attempts_parameter > 100) {
      throw std::runtime_error("planning_attempts must be between 1 and 100");
    }
    if (stage_timeout <= 0.0 || stage_timeout > 300.0) {
      throw std::runtime_error(
        "stage_timeout must be greater than 0 and at most 300 seconds");
    }
    const auto planning_attempts =
      static_cast<unsigned int>(planning_attempts_parameter);
    const auto planner_id = get_parameter("planner_id").as_string();

    // Normal-speed planner used by the grasp, carry and retreat stages.
    auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    pipeline->setPlannerId("ompl", planner_id);
    pipeline->setProperty("num_planning_attempts", planning_attempts);
    pipeline->setMaxVelocityScalingFactor(get_parameter("velocity_scaling").as_double());
    pipeline->setMaxAccelerationScalingFactor(get_parameter("acceleration_scaling").as_double());
    // Reduced-speed planner for the startup approach stages.
    auto initial_pipeline =
      std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    initial_pipeline->setPlannerId("ompl", planner_id);
    initial_pipeline->setProperty("num_planning_attempts", planning_attempts);
    initial_pipeline->setMaxVelocityScalingFactor(
      get_parameter("initial_velocity_scaling").as_double());
    initial_pipeline->setMaxAccelerationScalingFactor(
      get_parameter("initial_acceleration_scaling").as_double());
    auto joint = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    // Use deterministic joint-space interpolation for the startup transition.
    // The robot is expected to start from both_arms_home.  Splitting the motion
    // through an explicit safe waypoint prevents OMPL/RRTConnect from choosing
    // a different random route on each launch.
    auto startup_joint = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    startup_joint->setMaxVelocityScalingFactor(
      get_parameter("initial_velocity_scaling").as_double());
    startup_joint->setMaxAccelerationScalingFactor(
      get_parameter("initial_acceleration_scaling").as_double());
    // Use relatively dense waypoints so collision checking along the fixed
    // interpolation is more conservative than the default coarse spacing.
    startup_joint->setProperty("max_step", 0.05);

    auto to_crate_safe =
      std::make_unique<mtc::stages::MoveTo>(
        "move both arms to both_arms_crate_safe", startup_joint);
    to_crate_safe->setGroup("both_arms");
    to_crate_safe->setGoal("both_arms_crate_safe");
    task.add(std::move(to_crate_safe));

    auto to_crate_pre =
      std::make_unique<mtc::stages::MoveTo>(
        "move both arms to both_arms_crate_pre", startup_joint);
    to_crate_pre->setTimeout(stage_timeout);
    to_crate_pre->setGroup("both_arms");
    to_crate_pre->setGoal("both_arms_crate_pre");
    task.add(std::move(to_crate_pre));

    // auto open_left = std::make_unique<mtc::stages::MoveTo>("open left gripper", joint);
    // open_left->setGroup("left_gripper");
    // open_left->setGoal("left_gripper_opened");
    // task.add(std::move(open_left));
    // auto open_right = std::make_unique<mtc::stages::MoveTo>("open right gripper", joint);
    // open_right->setGroup("right_gripper");
    // open_right->setGoal("right_gripper_opened");
    // task.add(std::move(open_right));

    auto open_both =
      std::make_unique<mtc::stages::MoveTo>("open both grippers simultaneously", joint);
    open_both->setGroup("both_grippers");
    open_both->setGoal(std::map<std::string, double>{
      {"left_gripper_left_finger_joint", 0.0},
      {"gripper_left_finger_joint", 0.0}});
    task.add(std::move(open_both));

    const std::vector<double> p{
      get_parameter("crate_x").as_double(),
      get_parameter("crate_y").as_double(),
      get_parameter("crate_z").as_double()};
    const std::vector<double> s{
      get_parameter("crate_size_x").as_double(),
      get_parameter("crate_size_y").as_double(),
      get_parameter("crate_size_z").as_double()};
    const std::vector<double> offset{
      get_parameter("move_x").as_double(),
      get_parameter("move_y").as_double(),
      get_parameter("move_z").as_double()};
    const std::vector<double> left_rpy{
      get_parameter("left_tcp_roll").as_double(),
      get_parameter("left_tcp_pitch").as_double(),
      get_parameter("left_tcp_yaw").as_double()};
    const std::vector<double> right_rpy{
      get_parameter("right_tcp_roll").as_double(),
      get_parameter("right_tcp_pitch").as_double(),
      get_parameter("right_tcp_yaw").as_double()};
    const double pregrasp_height = get_parameter("pregrasp_clearance").as_double();
    const double grasp_center_offset = get_parameter("grasp_clearance").as_double();
    const double wall_thickness = get_parameter("wall_thickness").as_double();
    const double rim_grasp_depth = get_parameter("rim_grasp_depth").as_double();
    const auto carry_steps_parameter = get_parameter("carry_steps").as_int();
    if (rim_grasp_depth < 0.0 || rim_grasp_depth >= 0.5 * s[2]) {
      throw std::runtime_error(
        "rim_grasp_depth must be >= 0 and less than half the crate height");
    }
    if (wall_thickness <= 0.0 || wall_thickness >= 0.12) {
      throw std::runtime_error("wall_thickness must be greater than 0 and less than 0.12 m");
    }
    if (pregrasp_height <= 0.0 || pregrasp_height > 0.50) {
      throw std::runtime_error("pregrasp_clearance must be greater than 0 and at most 0.50 m");
    }
    if (std::abs(grasp_center_offset) > 0.5 * wall_thickness) {
      throw std::runtime_error(
        "grasp_clearance must stay within half the wall thickness so both fingers straddle it");
    }
    if (carry_steps_parameter < 1 || carry_steps_parameter > 60) {
      throw std::runtime_error("carry_steps must be between 1 and 60");
    }
    const auto carry_steps = static_cast<std::size_t>(carry_steps_parameter);
    const double grasp_z = p[2] + 0.5 * s[2] - rim_grasp_depth;
    // The side-wall primitive is centred half a wall thickness inside the
    // nominal outer crate dimension. Put the TCP at that centre so the wall is
    // between the fingers rather than outside the gripper opening.
    const double wall_center_y = 0.5 * (s[1] - wall_thickness);
    const double grasp_y = wall_center_y + grasp_center_offset;
    // Finger inner gap is 0.12 - 2*q for this URDF. Match it to the wall
    // thickness instead of commanding q=0.06, which would interpenetrate it.
    const double gripper_contact_position = 0.5 * (0.12 - wall_thickness);
    moveit::core::RobotState seed(task.getRobotModel());
    seed.setToDefaultValues();
    const auto* both_arms = task.getRobotModel()->getJointModelGroup("both_arms");
    if (both_arms) {
      seed.setToDefaultValues(both_arms, "both_arms_crate_pre");
    }
    seed.update();
    RCLCPP_INFO(
      get_logger(),
      "Crate center=(%.3f, %.3f, %.3f), size=(%.3f, %.3f, %.3f), wall=%.3f, "
      "overhead pregrasp=(y +/- %.3f, z %.3f), rim grasp=(y +/- %.3f, z %.3f), "
      "gripper q=%.3f, move=(%.3f, %.3f, %.3f)",
      p[0], p[1], p[2], s[0], s[1], s[2], wall_thickness,
      grasp_y, grasp_z + pregrasp_height, grasp_y, grasp_z, gripper_contact_position,
      offset[0], offset[1], offset[2]);
    auto pre_goal = solveDualArmGoal(
      makePose(p[0], p[1] + grasp_y, grasp_z + pregrasp_height, left_rpy),
      makePose(p[0], p[1] - grasp_y, grasp_z + pregrasp_height, right_rpy),
      seed, task.getRobotModel());      
    RCLCPP_INFO(get_logger(), "Pregrasp IK solved for both arms");
    for (const auto& joint : pre_goal) {
      seed.setVariablePosition(joint.first, joint.second);
    }
    seed.update();

    auto grasp_goal = solveDualArmGoal(
      makePose(p[0], p[1] + grasp_y, grasp_z, left_rpy),
      makePose(p[0], p[1] - grasp_y, grasp_z, right_rpy), seed, task.getRobotModel());
    RCLCPP_INFO(get_logger(), "Grasp IK solved for both arms");
    for (const auto& joint : grasp_goal) {
      seed.setVariablePosition(joint.first, joint.second);
    }
    seed.update();

    // Generate closely spaced synchronized dual-arm waypoints. A single OMPL
    // joint-space move can swing the unattached right hand away from the box
    // even when its final goal is correct. At each waypoint both TCPs are
    // solved on the two side-wall centres at the same transport fraction.
    std::vector<std::map<std::string, double>> carry_goals;
    carry_goals.reserve(carry_steps);
    for (std::size_t index = 1; index <= carry_steps; ++index) {
      const double fraction = static_cast<double>(index) / static_cast<double>(carry_steps);
      auto goal = solveDualArmGoal(
        makePose(
          p[0] + fraction * offset[0],
          p[1] + fraction * offset[1] + grasp_y,
          grasp_z + fraction * offset[2], left_rpy),
        makePose(
          p[0] + fraction * offset[0],
          p[1] + fraction * offset[1] - grasp_y,
          grasp_z + fraction * offset[2], right_rpy),
        seed, task.getRobotModel());
      for (const auto& joint_value : goal) {
        seed.setVariablePosition(joint_value.first, joint_value.second);
      }
      seed.update();
      carry_goals.push_back(std::move(goal));
    }
    RCLCPP_INFO(
      get_logger(), "Synchronized carry IK solved for %zu dual-arm waypoints", carry_steps);
    // After release, lift both TCPs vertically to the corresponding overhead
    // points before restoring normal crate collision checking.
    auto retreat_goal = solveDualArmGoal(
      makePose(p[0] + offset[0], p[1] + offset[1] + grasp_y,
               grasp_z + offset[2] + pregrasp_height, left_rpy),
      makePose(p[0] + offset[0], p[1] + offset[1] - grasp_y,
               grasp_z + offset[2] + pregrasp_height, right_rpy),
      seed, task.getRobotModel());
    RCLCPP_INFO(get_logger(), "Post-place retreat IK solved for both arms");

    // Only the four finger links may touch the crate during grasping. Never
    // disable collision checking for the gripper bases or TCP links, otherwise
    // OMPL is allowed to route the complete gripper through a crate wall.
    const std::vector<std::string> touch_links{
      "left_gripper_left_finger_link",
      "left_gripper_right_finger_link",
      "gripper_left_finger_link",
      "gripper_right_finger_link"};
    for (const auto& link_name : touch_links) {
      if (!task.getRobotModel()->getLinkModel(link_name)) {
        throw std::runtime_error("Missing gripper touch link in robot model: " + link_name);
      }
    }

    // The overhead pregrasp target is close to the crate. A direct joint-space
    // interpolation from the current state can sweep a wrist or forearm
    // through the crate, so let OMPL find a collision-free route to it.
    // auto to_pre = std::make_unique<mtc::stages::MoveTo>("both arms to pregrasp", pipeline);
    auto to_pre = std::make_unique<mtc::stages::MoveTo>("both arms to pregrasp", initial_pipeline);
    to_pre->setTimeout(stage_timeout);
    to_pre->setGroup("both_arms");
    to_pre->setGoal(pre_goal);
    task.add(std::move(to_pre));

    // The complete route to the overhead pregrasp is collision checked. Only
    // after reaching that clear pose may the fingers contact the side rims.
    auto allow =
      std::make_unique<mtc::stages::ModifyPlanningScene>("allow finger contact with crate");
    allow->allowCollisions("plastic_hollow_crate", touch_links, true);
    task.add(std::move(allow));


    auto to_grasp =
      std::make_unique<mtc::stages::MoveTo>("both arms grasp upper side rims", pipeline);
    to_grasp->setTimeout(stage_timeout);
    to_grasp->setGroup("both_arms");
    to_grasp->setGoal(grasp_goal);
    task.add(std::move(to_grasp));


    auto close_both =
      std::make_unique<mtc::stages::MoveTo>("close both grippers simultaneously", joint);
    close_both->setGroup("both_grippers");
    close_both->setGoal(std::map<std::string, double>{
      {"left_gripper_left_finger_joint", gripper_contact_position},
      {"gripper_left_finger_joint", gripper_contact_position}});
    task.add(std::move(close_both));

    // One rigid object can only have one parent link. Attach it to the left TCP;
    // the synchronized waypoints below keep the right TCP on the other wall.
    auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach crate");
    attach->attachObject("plastic_hollow_crate", "left_gripper_tcp");
    task.add(std::move(attach));

    for (std::size_t index = 0; index < carry_goals.size(); ++index) {
      auto carry = std::make_unique<mtc::stages::MoveTo>(
        "synchronized carry step " + std::to_string(index + 1) + "/" +
        std::to_string(carry_goals.size()), joint);
      carry->setGroup("both_arms");
      carry->setGoal(carry_goals[index]);
      task.add(std::move(carry));
    }

    auto release_both =
      std::make_unique<mtc::stages::MoveTo>("release both grippers simultaneously", joint);
    release_both->setGroup("both_grippers");
    release_both->setGoal(std::map<std::string, double>{
      {"left_gripper_left_finger_joint", 0.0},
      {"gripper_left_finger_joint", 0.0}});
    task.add(std::move(release_both));

    auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach crate at destination");
    detach->detachObject("plastic_hollow_crate", "left_gripper_tcp");
    task.add(std::move(detach));

    // Keep contact collision allowed while both grippers move away from the
    // just-released side walls. Restore normal collision checking afterwards.
    auto retreat = std::make_unique<mtc::stages::MoveTo>("retreat both arms from placed crate", pipeline);
    retreat->setTimeout(stage_timeout);
    retreat->setGroup("both_arms");
    retreat->setGoal(retreat_goal);
    task.add(std::move(retreat));

    auto forbid = std::make_unique<mtc::stages::ModifyPlanningScene>("restore crate collision checking");
    forbid->allowCollisions("plastic_hollow_crate", touch_links, false);
    task.add(std::move(forbid));
    return task;
  }
  mtc::Task task_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<DualArmCrateCarry>(options);
  std::thread spinner([node]() { rclcpp::spin(node); });
  try {
    rclcpp::sleep_for(std::chrono::seconds(2));
    node->run();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(), "%s", e.what());
  }
  spinner.join();
  return 0;
}
