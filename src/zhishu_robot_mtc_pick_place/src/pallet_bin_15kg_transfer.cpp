#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace mtc = moveit::task_constructor;

namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double value)
{
  while (value > kPi) {
    value -= 2.0 * kPi;
  }
  while (value < -kPi) {
    value += 2.0 * kPi;
  }
  return value;
}

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct BoxSpec
{
  std::string id;
  double x{0.0};
  double y{0.0};
  double z{0.224};
  int row{0};
  int col{0};
};
}  // namespace

class PalletBin15kgTransfer : public rclcpp::Node
{
public:
  explicit PalletBin15kgTransfer(const rclcpp::NodeOptions& options)
  : Node("pallet_bin_15kg_transfer", options)
  {
    const auto declare_if_missing = [this](const std::string& name, const auto& value) {
      if (!this->has_parameter(name)) {
        this->declare_parameter(name, value);
      }
    };

    declare_if_missing("execute", true);
    declare_if_missing("validate_all_nine", true);
    declare_if_missing("full_transfer_box_index", 4);
    declare_if_missing("pre_extract_enabled", false);
    declare_if_missing("skip_pre_extract_stage", false);
    declare_if_missing("pre_extract_box_index", 0);
    declare_if_missing("continue_after_pre_extract_plan", false);
    // One 120 mm pull is enough to open the 2--4 mm tote gap while keeping
    // the hooked TCP inside the right-arm lateral workspace.  Increase only
    // after this first extraction is verified in Gazebo.
    declare_if_missing("pre_extract_distance", 0.12);
    declare_if_missing("target_pose_topic", std::string("/pallet_bin/target_pose"));
    declare_if_missing("target_pose_max_age", 1.0);
    declare_if_missing("relocalization_timeout", 2.0);
    // The split side wall leaves a 30 mm high aperture from local z=0.038 to
    // 0.068 m; target its geometric centre (53 mm above the tote centre).
    declare_if_missing("hook_hole_z_offset", 0.053);
    // Enter the opening by 15 mm from the side-wall plane.  The TCP itself is
    // kept above the hole; the 200 mm finger is the hook member that drops
    // through the aperture.
    declare_if_missing("hook_hole_inset", 0.015);
    // Keep the TCP in the arm's reachable upper hook pose. The extended
    // 275 mm finger reaches down from this pose to the real lower edge of
    // the 30 mm handle aperture.
    declare_if_missing("hook_tcp_hole_clearance", 0.100);
    declare_if_missing("hook_approach_height", 0.11);
    declare_if_missing("hook_approach_outset", 0.055);
    declare_if_missing("hook_seat_drop", 0.0);
    // Small calibrated TCP-side trim: the hook station is about 6 mm
    // rearward of the nominal CAD base origin, and this keeps the waypoint
    // inside the bounded right-arm IK corridor.
    declare_if_missing("hook_pose_x_offset", 0.010);
    // The real front-wall aperture is about 7 mm farther inside than the
    // nominal row-centre transform at the initial odometry pose.
    declare_if_missing("hook_pose_y_offset", -0.007);
    declare_if_missing("hook_tcp_roll", 3.14159265359);
    declare_if_missing("hook_tcp_pitch", -0.55);
    declare_if_missing("hook_tcp_yaw", -1.57079632679);
    declare_if_missing("stop_on_failure", true);
    declare_if_missing("pre_extract_waist_lift", 0.0);
    declare_if_missing("pre_extract_waist_pitch", -0.70);
    declare_if_missing("state_wait_timeout", 30.0);
    declare_if_missing("max_solutions", 3);
    declare_if_missing("planning_attempts", 8);
    declare_if_missing("stage_timeout", 20.0);
    declare_if_missing("planner_id", std::string("RRTConnectkConfigDefault"));
    declare_if_missing("velocity_scaling", 0.12);
    declare_if_missing("acceleration_scaling", 0.10);

    declare_if_missing("base_max_linear", 0.18);
    declare_if_missing("base_max_angular", 0.45);
    declare_if_missing("base_position_tolerance", 0.025);
    declare_if_missing("base_yaw_tolerance", 0.035);
    declare_if_missing("base_motion_timeout", 90.0);
    // Gazebo spawns the chassis at a calibrated work-station offset while
    // the DiffDrive odometry starts at zero.  Keep planning in world-aligned
    // coordinates by adding this fixed physical offset to the odometry;
    // driveToPose still uses the same effective pose for any later motion.
    declare_if_missing("physical_base_offset_x", 0.0);
    declare_if_missing("physical_base_offset_y", 0.0);
    declare_if_missing("source_grasp_x", 0.45);
    declare_if_missing("source_grasp_y", -0.76);

    declare_if_missing("grasp_waist_lift", 0.04);
    declare_if_missing("grasp_waist_pitch", -0.55);
    declare_if_missing("carry_waist_lift", 0.18);
    declare_if_missing("carry_waist_pitch", -0.10);
    declare_if_missing("place_waist_lift", 0.20);
    declare_if_missing("place_waist_pitch", -0.05);
    declare_if_missing("waist_motion_seconds", 2.5);

    declare_if_missing("handle_half_span", 0.195);
    declare_if_missing("grasp_half_span", 0.195);
    declare_if_missing("handle_z_offset", 0.035);
    declare_if_missing("handle_thickness", 0.006);
    declare_if_missing("pregrasp_clearance", 0.12);
    declare_if_missing("load_test_lift", 0.05);
    declare_if_missing("carry_lift", 0.10);
    declare_if_missing("hold_seconds", 3.0);
    declare_if_missing("place_approach_clearance", 0.18);
    declare_if_missing("source_release_drop", 0.010);
    declare_if_missing("bin_release_drop", 0.020);

    declare_if_missing("left_tcp_roll", -1.57079632679);
    declare_if_missing("left_tcp_pitch", 0.0);
    declare_if_missing("left_tcp_yaw", 1.57079632679);
    declare_if_missing("right_tcp_roll", -1.57079632679);
    declare_if_missing("right_tcp_pitch", 0.0);
    declare_if_missing("right_tcp_yaw", -1.57079632679);
    // Move each TCP a few centimetres along the 120 mm handle rail so the
    // inner/right arm can use a reachable elbow branch without leaving the
    // physical handle span (the default remains centred).
    declare_if_missing("grasp_x_offset", 0.0);
    declare_if_missing("place_grasp_x_offset", 0.0);
    declare_if_missing("swap_arm_grasps", false);
    declare_if_missing("dual_ik_attempts", 100);
    declare_if_missing("dual_ik_timeout", 0.50);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    body_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/body_posture_controller/joint_trajectory", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        odom_pose_.x = msg->pose.pose.position.x;
        odom_pose_.y = msg->pose.pose.position.y;
        odom_pose_.yaw = tf2::getYaw(msg->pose.pose.orientation);
        odom_ready_ = true;
      });

    left_wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/left_wrist_ft", 50,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        left_force_norm_ = forceNorm(*msg);
        left_force_ready_ = true;
      });
    right_wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/right_wrist_ft", 50,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        right_force_norm_ = forceNorm(*msg);
        right_force_ready_ = true;
      });

    target_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("target_pose_topic").as_string(), 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (msg->header.frame_id != "base_footprint") {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Ignoring target pose in frame '%s'; expected base_footprint",
            msg->header.frame_id.c_str());
          return;
        }
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_target_pose_ = *msg;
        latest_target_received_ = now();
      });

    // Coordinates are derived from baifang.stp using:
    //   world X <- STEP Y, world Y <- STEP Z, world Z <- 150 mm - STEP X.
    const std::array<double, 3> xs{0.704355, 1.028284, 1.351011};
    const std::array<double, 3> ys{-0.400617, 0.002017, 0.406220};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        boxes_.push_back(BoxSpec{
          "pallet_box_r" + std::to_string(row) + "_c" + std::to_string(col),
          xs[row], ys[col], 0.224, row, col});
      }
    }
  }

  void run()
  {
    waitForOdom();
    waitForCurrentRobotState();
    addEnvironmentToPlanningScene();

    const bool validate_all = get_parameter("validate_all_nine").as_bool();
    const bool execute = get_parameter("execute").as_bool();
    const bool stop_on_failure = get_parameter("stop_on_failure").as_bool();

    if (get_parameter("pre_extract_enabled").as_bool()) {
      const int index = get_parameter("pre_extract_box_index").as_int();
      if (index < 0 || index >= static_cast<int>(boxes_.size())) {
        throw std::runtime_error("pre_extract_box_index must be in [0, 8]");
      }
      auto& box = boxes_[static_cast<std::size_t>(index)];
      if (box.col != 0) {
        throw std::runtime_error(
          "The first extraction must use a negative-Y perimeter tote (column 0)");
      }
      if (get_parameter("skip_pre_extract_stage").as_bool()) {
        // Planning-only geometry probe: use the measured post-pull pose while
        // bypassing the one-arm hook task.  The normal execution path keeps
        // this false and always performs the real pre-extraction.
        box.x = 0.704355;
        box.y = -0.520617;
        box.z = 0.224;
        RCLCPP_WARN(
          get_logger(), "Skipping pre-extraction motion; probing relocated tote at (%.3f, %.3f, %.3f)",
          box.x, box.y, box.z);
      } else {
      RCLCPP_INFO(
        get_logger(), "Pre-extracting perimeter tote %s with the right arm", box.id.c_str());
      if (!goToHookWorkPose(box)) {
        throw std::runtime_error("Failed to dock for perimeter pre-extraction");
      }
      addEnvironmentToPlanningScene();
      setWaist(
        get_parameter("pre_extract_waist_lift").as_double(),
        get_parameter("pre_extract_waist_pitch").as_double());
      const auto base = currentOdom();
      const auto source = worldPointInBase(box.x, box.y, box.z, base);
      const double distance = get_parameter("pre_extract_distance").as_double();
      if (!runRightArmExtraction(box.id, source, distance, execute)) {
        throw std::runtime_error("Right-arm perimeter extraction failed for " + box.id);
      }
      if (!execute && !get_parameter("continue_after_pre_extract_plan").as_bool()) {
        RCLCPP_INFO(
          get_logger(),
          "Pre-extraction planning succeeded; execute:=false leaves the Gazebo tote unchanged");
        return;
      }
      if (!execute) {
        RCLCPP_INFO(
          get_logger(),
          "Continuing planning-only flow after the simulated pre-extraction stage");
      }
      const auto relocated = relocalizeExtractedBox(box, distance);
      const auto base_after_pull = currentOdom();
      const double c = std::cos(base_after_pull.yaw);
      const double s = std::sin(base_after_pull.yaw);
      box.x = base_after_pull.x + c * relocated[0] - s * relocated[1];
      box.y = base_after_pull.y + s * relocated[0] + c * relocated[1];
      box.z = relocated[2];
      RCLCPP_INFO(
        get_logger(), "Relocalized %s world center=(%.3f, %.3f, %.3f)",
        box.id.c_str(), box.x, box.y, box.z);
      addEnvironmentToPlanningScene();
      setWaist(0.0, 0.0);
      }
    }

    if (validate_all) {
      RCLCPP_INFO(
        get_logger(),
        "Starting 3x3 pallet coverage validation (%s)",
        execute ? "15 kg pick/lift/hold/return" : "planning only");
      for (std::size_t index = 0; index < boxes_.size(); ++index) {
        const auto& box = boxes_[index];
        RCLCPP_INFO(
          get_logger(), "[%zu/9] validating %s at world=(%.3f, %.3f, %.3f)",
          index + 1, box.id.c_str(), box.x, box.y, box.z);

        if (!goToSourceWorkPose(box)) {
          if (stop_on_failure) {
            throw std::runtime_error("Failed to dock at source work pose for " + box.id);
          }
          continue;
        }
        addEnvironmentToPlanningScene();
        setWaist(
          get_parameter("grasp_waist_lift").as_double(),
          get_parameter("grasp_waist_pitch").as_double());

        const auto base = currentOdom();
        const auto source = worldPointInBase(box.x, box.y, box.z, base);
        const bool picked = runPickTask(
          box.id, source, get_parameter("load_test_lift").as_double(), execute,
          "load-test pick " + box.id);
        if (!picked) {
          if (stop_on_failure) {
            throw std::runtime_error("Pick validation failed for " + box.id);
          }
          continue;
        }

        if (execute) {
          const double hold = get_parameter("hold_seconds").as_double();
          rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(hold)));
          logWristLoad(box.id);

          // Lower the same 15 kg tote back to its pallet cell. Release a few
          // millimetres above the original pose so Gazebo contact, not teleport,
          // completes the return.
          const auto source_return = std::array<double, 3>{
            source[0], source[1],
            source[2] + get_parameter("source_release_drop").as_double()};
          const bool returned = runPlaceTask(
            box.id, source_return,
            get_parameter("load_test_lift").as_double(), true,
            "load-test return " + box.id);
          if (!returned && stop_on_failure) {
            throw std::runtime_error("Return-to-pallet failed for " + box.id);
          }
          rclcpp::sleep_for(std::chrono::milliseconds(600));
          resetWorldBoxCollision(box);
          moveArmsHome();
        }
        setWaist(0.0, 0.0);
      }
    }

    const int full_index = get_parameter("full_transfer_box_index").as_int();
    if (full_index < 0 || full_index >= static_cast<int>(boxes_.size())) {
      RCLCPP_WARN(get_logger(), "full_transfer_box_index=%d disables full transfer", full_index);
      return;
    }

    const auto& box = boxes_[static_cast<std::size_t>(full_index)];
    RCLCPP_INFO(
      get_logger(),
      "Starting complete pallet -> large-bin transfer with %s (index %d)",
      box.id.c_str(), full_index);

    if (!goToSourceWorkPose(box)) {
      throw std::runtime_error("Failed to dock for full transfer source");
    }
    addEnvironmentToPlanningScene();
    setWaist(
      get_parameter("grasp_waist_lift").as_double(),
      get_parameter("grasp_waist_pitch").as_double());
    auto base = currentOdom();
    const auto source = worldPointInBase(box.x, box.y, box.z, base);
    if (!runPickTask(
          box.id, source, get_parameter("carry_lift").as_double(), execute,
          "full-transfer pick " + box.id)) {
      throw std::runtime_error("Full-transfer pick failed for " + box.id);
    }

    if (!execute) {
      RCLCPP_INFO(
        get_logger(),
        "execute:=false: full-transfer pick plan succeeded; base carry/place was not executed");
      return;
    }

    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(get_parameter("hold_seconds").as_double())));
    logWristLoad(box.id + " before base carry");

    setWaist(
      get_parameter("carry_waist_lift").as_double(),
      get_parameter("carry_waist_pitch").as_double());
    if (!goToBinDock()) {
      throw std::runtime_error("Failed to dock at large bin");
    }
    addEnvironmentToPlanningScene();
    setWaist(
      get_parameter("place_waist_lift").as_double(),
      get_parameter("place_waist_pitch").as_double());

    base = currentOdom();
    // Large bin world centre: (1.03, -1.55), table top 0.55 m.  The STEP bin
    // bottom is ~12 mm. Release 20 mm above the inner floor, then let gravity
    // settle the tote to the floor.
    const double settled_z = 0.55 + 0.012 + 0.5 * 0.148;
    const double release_z = settled_z + get_parameter("bin_release_drop").as_double();
    const auto destination = worldPointInBase(1.03, -1.55, release_z, base);
    if (!runPlaceTask(
          box.id, destination,
          get_parameter("place_approach_clearance").as_double(), true,
          "large-bin insertion " + box.id)) {
      throw std::runtime_error("Large-bin insertion failed for " + box.id);
    }

    rclcpp::sleep_for(std::chrono::seconds(1));
    resetWorldBoxCollision(BoxSpec{box.id, 1.03, -1.55, settled_z, box.row, box.col});
    moveArmsHome();
    setWaist(0.0, 0.0);
    RCLCPP_INFO(
      get_logger(),
      "Complete transfer finished. The physical Gazebo tote was carried by contact/gripper closure; "
      "the MoveIt object was used only for planning-scene attachment and collision checking.");
  }

private:
  static double forceNorm(const geometry_msgs::msg::WrenchStamped& msg)
  {
    const auto& f = msg.wrench.force;
    return std::sqrt(f.x * f.x + f.y * f.y + f.z * f.z);
  }

  Pose2D currentOdom() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    Pose2D pose = odom_pose_;
    pose.x += get_parameter("physical_base_offset_x").as_double();
    pose.y += get_parameter("physical_base_offset_y").as_double();
    return pose;
  }

  std::array<double, 3> relocalizeExtractedBox(
    const BoxSpec& box, double commanded_distance)
  {
    const double timeout = get_parameter("relocalization_timeout").as_double();
    const double max_age = get_parameter("target_pose_max_age").as_double();
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout));
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (latest_target_pose_ &&
          (now() - latest_target_received_).seconds() <= max_age)
        {
          const auto& p = latest_target_pose_->pose.position;
          RCLCPP_INFO(
            get_logger(), "Using fresh RGB-D target pose after hook pull (age %.3f s)",
            (now() - latest_target_received_).seconds());
          return {p.x, p.y, p.z};
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    const auto base = currentOdom();
    auto nominal = worldPointInBase(box.x, box.y, box.z, base);
    nominal[1] -= commanded_distance;
    RCLCPP_WARN(
      get_logger(),
      "No fresh pose arrived on %s; using commanded Gazebo displacement as the simulation fallback",
      get_parameter("target_pose_topic").as_string().c_str());
    return nominal;
  }

  void waitForOdom()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (odom_ready_) {
          RCLCPP_INFO(
            get_logger(), "Mobile-base odometry ready at (%.3f, %.3f, %.3f)",
            odom_pose_.x, odom_pose_.y, odom_pose_.yaw);
          return;
        }
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("Timed out waiting for /odom. Check DiffDrive and ros_gz_bridge.");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    throw std::runtime_error("ROS shutdown while waiting for odometry");
  }

  void waitForCurrentRobotState()
  {
    const double timeout = get_parameter("state_wait_timeout").as_double();
    moveit::planning_interface::MoveGroupInterface move_group(
      shared_from_this(), "both_arms", std::shared_ptr<tf2_ros::Buffer>(),
      rclcpp::Duration::from_seconds(timeout));
    const auto state = move_group.getCurrentState(timeout);
    if (!state) {
      throw std::runtime_error(
        "Timed out waiting for both_arms state. Check joint_state_broadcaster and controllers.");
    }
    RCLCPP_INFO(get_logger(), "MoveIt current state is ready");
  }

  bool driveToPose(const Pose2D& target)
  {
    const double max_linear = get_parameter("base_max_linear").as_double();
    const double max_angular = get_parameter("base_max_angular").as_double();
    const double position_tol = get_parameter("base_position_tolerance").as_double();
    const double yaw_tol = get_parameter("base_yaw_tolerance").as_double();
    const double timeout = get_parameter("base_motion_timeout").as_double();
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timeout));
    rclcpp::WallRate rate(30.0);

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto current = currentOdom();
      const double dx = target.x - current.x;
      const double dy = target.y - current.y;
      const double distance = std::hypot(dx, dy);
      geometry_msgs::msg::Twist cmd;

      if (distance > position_tol) {
        const double heading = std::atan2(dy, dx);
        const double heading_error = normalizeAngle(heading - current.yaw);
        if (std::abs(heading_error) > 0.22) {
          cmd.angular.z = clamp(1.4 * heading_error, -max_angular, max_angular);
        } else {
          cmd.linear.x = std::min(max_linear, 0.8 * distance);
          cmd.angular.z = clamp(1.1 * heading_error, -max_angular, max_angular);
        }
      } else {
        const double yaw_error = normalizeAngle(target.yaw - current.yaw);
        if (std::abs(yaw_error) <= yaw_tol) {
          cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
          RCLCPP_INFO(
            get_logger(), "Base docked at (%.3f, %.3f, %.3f)",
            current.x, current.y, current.yaw);
          return true;
        }
        cmd.angular.z = clamp(1.4 * yaw_error, -max_angular, max_angular);
      }
      cmd_vel_pub_->publish(cmd);
      rate.sleep();
    }

    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    const auto current = currentOdom();
    RCLCPP_ERROR(
      get_logger(),
      "Base docking timed out. current=(%.3f, %.3f, %.3f), target=(%.3f, %.3f, %.3f)",
      current.x, current.y, current.yaw, target.x, target.y, target.yaw);
    return false;
  }

  bool goToSourceWorkPose(const BoxSpec& box)
  {
    // A narrow but collision-free corridor between pallet (y >= -0.60) and
    // table (y <= -1.075). The 0.35 m-wide base keeps >= 50 mm nominal margin.
    constexpr double corridor_y = -0.85;
    if (box.row <= 1) {
      if (box.row == 0 && box.col == 0) {
        // After perimeter extraction, approach from the negative-Y aisle.
        // This keeps the chassis clear of the pallet edge while reducing the
        // dual-arm target's forward reach and preserving both side handles in
        // the robot's lateral workspace.
        const double grasp_x = get_parameter("source_grasp_x").as_double();
        const double grasp_y = get_parameter("source_grasp_y").as_double();
        if (!driveToPose(Pose2D{0.19, grasp_y, 0.0})) {
          return false;
        }
        return driveToPose(Pose2D{grasp_x, grasp_y, 0.0});
      }
      // The first perimeter tote is reached after the hook pull.  Keep the
      // chassis at the same calibrated x=0.19 m station used by the hook so
      // the dual-arm side-handle targets remain inside both-arm IK range.
      const double source_x = box.row == 0 ? 0.19 : 0.18;
      if (!driveToPose(Pose2D{source_x, corridor_y, 0.0})) {
        return false;
      }
      return driveToPose(Pose2D{source_x, box.y, 0.0});
    }

    // Back row: route outside the pallet far edge before moving laterally.
    constexpr double outside_rear_x = 2.10;
    constexpr double rear_dock_x = 1.918;  // 50 mm from pallet rear edge.
    if (!driveToPose(Pose2D{outside_rear_x, corridor_y, 0.0})) {
      return false;
    }
    if (!driveToPose(Pose2D{outside_rear_x, box.y, kPi})) {
      return false;
    }
    return driveToPose(Pose2D{rear_dock_x, box.y, kPi});
  }

  bool goToHookWorkPose(const BoxSpec& box)
  {
    // The launch pose is deliberately aligned with r0_c0.  DiffDrive odom is
    // only a relative estimate and cannot strafe the chassis into the dense
    // negative-Y row without losing the physical Gazebo/world alignment.  A
    // 5 mm rearward trim keeps the hook TCP inside the bounded IK corridor
    // while preserving the requested ~20 cm front station.
    if (box.row == 0 && box.col == 0) {
      constexpr double hook_station_x = 0.185;
      if (!driveToPose(Pose2D{hook_station_x, box.y, 0.0})) {
        return false;
      }
      RCLCPP_INFO(
        get_logger(), "Using trimmed 20 cm hook station for %s: x=%.3f m",
        box.id.c_str(), hook_station_x);
      return true;
    }
    // Start from the 200 mm front station, then advance only to x=0.19 m for
    // the perimeter hook.  The base collision front is about 0.234 m ahead of
    // base_footprint and the pallet near edge is x=0.434 m; the extra 10 mm
    // clearance avoids a planning-scene overlap while keeping the arm within
    // its oblique handle-hole reach.
    constexpr double hook_dock_x = 0.19;
    if (!driveToPose(Pose2D{hook_dock_x, box.y, 0.0})) {
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "Hook-extraction dock for %s: base=(%.3f, %.3f, 0)",
      box.id.c_str(), hook_dock_x, box.y);
    return true;
  }

  bool goToBinDock()
  {
    constexpr double corridor_y = -0.85;
    // The chassis can pass under the 0.55 m tabletop (its upper shell is
    // below the tabletop and the legs are outside this y corridor). Move
    // farther under the table so the large-bin TCP is inside the left-arm
    // reach envelope instead of at the previous 0.62 m boundary.
    constexpr double dock_x = 0.70;
    constexpr double dock_y = -1.55;
    if (!driveToPose(Pose2D{dock_x, corridor_y, -0.5 * kPi})) {
      return false;
    }
    return driveToPose(Pose2D{dock_x, dock_y, 0.0});
  }

  void setWaist(double lift, double pitch)
  {
    lift = clamp(lift, 0.0, 0.26);  // reserve 40 mm from the URDF 0.30 m limit
    pitch = clamp(pitch, -0.70, 0.20);  // reserve margin from -45/+15 deg limits
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = {
      "waist_up_down_joint", "waist_pitch_joint", "head_yaw_joint", "head_pitch_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {lift, pitch, 0.0, 0.0};
    const double seconds = get_parameter("waist_motion_seconds").as_double();
    point.time_from_start.sec = static_cast<int32_t>(std::floor(seconds));
    point.time_from_start.nanosec = static_cast<uint32_t>(
      (seconds - std::floor(seconds)) * 1e9);
    trajectory.points.push_back(point);
    body_pub_->publish(trajectory);
    RCLCPP_INFO(get_logger(), "Command waist lift=%.3f m pitch=%.3f rad", lift, pitch);
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds + 0.35)));
  }

  std::array<double, 3> worldPointInBase(
    double wx, double wy, double wz, const Pose2D& base) const
  {
    const double dx = wx - base.x;
    const double dy = wy - base.y;
    const double c = std::cos(base.yaw);
    const double s = std::sin(base.yaw);
    return {c * dx + s * dy, -s * dx + c * dy, wz};
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
    moveit_msgs::msg::CollisionObject& object,
    double sx, double sy, double sz,
    double x, double y, double z,
    double yaw = 0.0)
  {
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = sx;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = sy;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = sz;
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(makePose(x, y, z, {0.0, 0.0, yaw}));
  }

  void appendWorldBox(
    moveit_msgs::msg::CollisionObject& object,
    double sx, double sy, double sz,
    double wx, double wy, double wz, double world_yaw,
    const Pose2D& base) const
  {
    const auto p = worldPointInBase(wx, wy, wz, base);
    appendBox(
      object, sx, sy, sz, p[0], p[1], p[2],
      normalizeAngle(world_yaw - base.yaw));
  }

  moveit_msgs::msg::CollisionObject makeSimpleBoxObject(
    const BoxSpec& box, const Pose2D& base) const
  {
    moveit_msgs::msg::CollisionObject object;
    // MoveIt keeps this robot model fixed at base_footprint; the Gazebo base
    // pose is therefore folded into every world-object pose before each arm
    // planning step. This avoids adding a simulation-only virtual joint to the
    // production SRDF while still keeping collision geometry correct after the
    // mobile base drives.
    object.header.frame_id = "base_footprint";
    object.id = box.id;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    // Match the Gazebo CAD approximation: the side walls are split around a
    // 120 mm x 30 mm upper handle aperture.  Keeping the same primitives in
    // MoveIt is essential; a single solid 0.30 x 0.40 box would make the
    // hook insertion appear colliding even though Gazebo has a real opening.
    appendWorldBox(object, 0.30, 0.40, 0.012, box.x, box.y, box.z - 0.068, 0.0, base);
    appendWorldBox(object, 0.010, 0.38, 0.136, box.x + 0.145, box.y, box.z, 0.0, base);
    appendWorldBox(object, 0.010, 0.38, 0.136, box.x - 0.145, box.y, box.z, 0.0, base);
    appendWorldBox(object, 0.30, 0.010, 0.106, box.x, box.y + 0.195, box.z - 0.015, 0.0, base);
    appendWorldBox(object, 0.090, 0.010, 0.030, box.x - 0.105, box.y + 0.195, box.z + 0.053, 0.0, base);
    appendWorldBox(object, 0.090, 0.010, 0.030, box.x + 0.105, box.y + 0.195, box.z + 0.053, 0.0, base);
    appendWorldBox(object, 0.30, 0.010, 0.106, box.x, box.y - 0.195, box.z - 0.015, 0.0, base);
    appendWorldBox(object, 0.090, 0.010, 0.030, box.x - 0.105, box.y - 0.195, box.z + 0.053, 0.0, base);
    appendWorldBox(object, 0.090, 0.010, 0.030, box.x + 0.105, box.y - 0.195, box.z + 0.053, 0.0, base);
    // Thin lower lip of each handle opening; this is intentionally much
    // thinner than the old solid rail so a hook can pass through the hole.
    appendWorldBox(object, 0.12, 0.006, 0.006, box.x, box.y + 0.195, box.z + 0.038, 0.0, base);
    appendWorldBox(object, 0.12, 0.006, 0.006, box.x, box.y - 0.195, box.z + 0.038, 0.0, base);
    return object;
  }

  void addEnvironmentToPlanningScene()
  {
    const Pose2D base = currentOdom();
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    moveit_msgs::msg::CollisionObject pallet;
    pallet.header.frame_id = "base_footprint";
    pallet.id = "source_pallet";
    pallet.operation = moveit_msgs::msg::CollisionObject::ADD;
    appendWorldBox(pallet, 1.20, 1.20, 0.15, 1.034, 0.0, 0.075, 0.0, base);
    objects.push_back(pallet);

    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = "base_footprint";
    table.id = "destination_table";
    table.operation = moveit_msgs::msg::CollisionObject::ADD;
    // Conservative solid volume below the 0.55 m tabletop.
    appendWorldBox(table, 0.75, 0.95, 0.55, 1.03, -1.55, 0.275, 0.0, base);
    objects.push_back(table);

    moveit_msgs::msg::CollisionObject bin;
    bin.header.frame_id = "base_footprint";
    bin.id = "large_bin";
    bin.operation = moveit_msgs::msg::CollisionObject::ADD;
    constexpr double yaw = 0.5 * kPi;
    auto local_to_world = [](double lx, double ly) {
      // yaw = +90 deg: world x = cx - ly, world y = cy + lx.
      return std::pair<double, double>{1.03 - ly, -1.55 + lx};
    };
    {
      const auto p = local_to_world(0.0, 0.0);
      appendWorldBox(bin, 0.650, 0.450, 0.012, p.first, p.second, 0.556, yaw, base);
    }
    {
      const auto p = local_to_world(0.319, 0.0);
      appendWorldBox(bin, 0.012, 0.450, 0.288, p.first, p.second, 0.706, yaw, base);
    }
    {
      const auto p = local_to_world(-0.319, 0.0);
      appendWorldBox(bin, 0.012, 0.450, 0.288, p.first, p.second, 0.706, yaw, base);
    }
    {
      const auto p = local_to_world(0.0, 0.219);
      appendWorldBox(bin, 0.626, 0.012, 0.288, p.first, p.second, 0.706, yaw, base);
    }
    {
      const auto p = local_to_world(0.0, -0.219);
      appendWorldBox(bin, 0.626, 0.012, 0.288, p.first, p.second, 0.706, yaw, base);
    }
    objects.push_back(bin);

    for (const auto& box : boxes_) {
      // While an MTC pick is physically being carried, this id is attached to
      // left_gripper_tcp in the planning scene. Do not also recreate it in the
      // world when the base reaches the next dock.
      if (box.id != attached_box_id_) {
        objects.push_back(makeSimpleBoxObject(box, base));
      }
    }

    moveit::planning_interface::PlanningSceneInterface psi;
    if (!psi.applyCollisionObjects(objects)) {
      throw std::runtime_error("Failed to populate MoveIt planning scene");
    }
    RCLCPP_INFO(
      get_logger(),
      "MoveIt scene refreshed in base_footprint at odom base=(%.3f, %.3f, %.3f)",
      base.x, base.y, base.yaw);
  }

  void resetWorldBoxCollision(const BoxSpec& box)
  {
    const Pose2D base = currentOdom();
    moveit::planning_interface::PlanningSceneInterface psi;
    moveit_msgs::msg::CollisionObject remove;
    remove.header.frame_id = "base_footprint";
    remove.id = box.id;
    remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    psi.applyCollisionObject(remove);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    psi.applyCollisionObject(makeSimpleBoxObject(box, base));
  }

  std::map<std::string, double> solveDualArmGoal(
    const geometry_msgs::msg::Pose& left_pose,
    const geometry_msgs::msg::Pose& right_pose,
    const moveit::core::RobotState& seed,
    const moveit::core::RobotModelConstPtr& model) const
  {
    const auto* left = model->getJointModelGroup("left_arm");
    const auto* right = model->getJointModelGroup("right_arm");
    const auto* both = model->getJointModelGroup("both_arms");
    if (!left || !right || !both) {
      throw std::runtime_error("SRDF must contain left_arm, right_arm and both_arms groups");
    }

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

    planning_scene::PlanningScene self_collision_scene(model);
    moveit::core::RobotState state(seed);
    struct MirrorJoint
    {
      const char* left_name;
      const char* right_name;
      double sign;
    };
    const MirrorJoint mirror_joints[] = {
      {"left_shoulder_pitch_joint", "right_shoulder_pitch_joint", -1.0},
      {"left_shoulder_roll_joint", "right_shoulder_roll_joint", -1.0},
      {"left_shoulder_yaw_joint", "right_shoulder_yaw_joint", -1.0},
      {"left_elbow_pitch_joint", "right_elbow_pitch_joint", 1.0},
      {"left_wrist_yaw_joint", "right_wrist_yaw_joint", -1.0},
      {"left_wrist_pitch_joint", "right_wrist_pitch_joint", 1.0},
      {"left_wrist_roll_joint", "right_wrist_roll_joint", -1.0}};

    const std::size_t max_attempts = static_cast<std::size_t>(
      std::max<int64_t>(1, get_parameter("dual_ik_attempts").as_int()));
    std::size_t left_ik_failures = 0;
    std::size_t right_ik_failures = 0;
    std::size_t bounds_failures = 0;
    std::size_t collision_failures = 0;
    for (std::size_t attempt = 0; attempt < max_attempts; ++attempt) {
      moveit::core::RobotState candidate(seed);
      if (attempt > 0) {
        candidate.setToRandomPositions(left);
      }
      candidate.update();
      const double dual_ik_timeout = get_parameter("dual_ik_timeout").as_double();
      if (!candidate.setFromIK(left, left_hand_pose, "L_hand_link", dual_ik_timeout)) {
        ++left_ik_failures;
        continue;
      }
      for (const auto& joint : mirror_joints) {
        candidate.setVariablePosition(
          joint.right_name,
          joint.sign * candidate.getVariablePosition(joint.left_name));
      }
      candidate.update();
      if (!candidate.setFromIK(right, right_hand_pose, "R_hand_link", dual_ik_timeout)) {
        ++right_ik_failures;
        continue;
      }
      candidate.update();

      // The mirrored left-arm solution is a useful seed for the redundant
      // right-arm IK, but it must not be a hard joint-by-joint constraint.
      // Symmetric TCP targets can legitimately use different redundant joint
      // branches; bounds, self-collision, and MTC environment checks decide
      // whether the resulting dual-arm state is usable.
      if (!candidate.satisfiesBounds(both)) {
        ++bounds_failures;
        continue;
      }
      if (self_collision_scene.isStateColliding(candidate, "both_arms", false)) {
        // With the waist pitched forward for a low pallet grasp, the stock
        // waist collision boxes overlap the right elbow/shoulder even though
        // the physical arm clears the waist shell.  Treat only these known
        // crouched-posture pairs as an allowed contact; all arm/arm and
        // arm/tool collisions remain hard failures.
        collision_detection::CollisionRequest request;
        request.group_name = "both_arms";
        request.contacts = true;
        request.max_contacts = 8;
        request.max_contacts_per_pair = 2;
        collision_detection::CollisionResult result;
        self_collision_scene.checkSelfCollision(request, result, candidate);
        bool waist_only = !result.contacts.empty();
        for (const auto& contact : result.contacts) {
          const auto& a = contact.first.first;
          const auto& b = contact.first.second;
          const bool has_waist =
            a == "waist_pitch_link" || b == "waist_pitch_link" ||
            a == "waist_up_down_link" || b == "waist_up_down_link";
          waist_only = waist_only && has_waist;
          if (collision_failures == 0) {
            RCLCPP_WARN(
              get_logger(), "dual-arm IK first self-collision: %s <-> %s",
              a.c_str(), b.c_str());
          }
        }
        if (!waist_only) {
          ++collision_failures;
          continue;
        }
        RCLCPP_INFO_ONCE(
          get_logger(),
          "Allowing waist_pitch_link contact for the crouched dual-arm grasp posture");
      }
      state = candidate;
      std::map<std::string, double> goal;
      for (const auto& name : both->getActiveJointModelNames()) {
        goal[name] = state.getVariablePosition(name);
      }
      return goal;
    }
    std::ostringstream reason;
    reason << "No bounded collision-free dual-arm IK branch found (left_ik="
      << left_ik_failures << ", right_ik=" << right_ik_failures
      << ", bounds=" << bounds_failures << ", collision=" << collision_failures << ")";
    throw std::runtime_error(reason.str());
  }

  std::map<std::string, double> solveRightArmGoal(
    const geometry_msgs::msg::Pose& tcp_pose,
    const moveit::core::RobotState& seed,
    const moveit::core::RobotModelConstPtr& model,
    std::size_t attempts = 120,
    double ik_timeout = 0.20) const
  {
    const auto* right = model->getJointModelGroup("right_arm");
    if (!right) {
      throw std::runtime_error("SRDF does not contain right_arm");
    }
    // Construct the self-collision scene once per IK query.  Rebuilding a
    // PlanningScene for every random seed dominates the hook search time and
    // made the small aperture lattice unnecessarily slow.
    planning_scene::PlanningScene self_collision_scene(model);
    moveit::core::RobotState reference(seed);
    reference.updateLinkTransforms();
    const Eigen::Isometry3d hand_to_tcp =
      reference.getGlobalLinkTransform("R_hand_link").inverse() *
      reference.getGlobalLinkTransform("right_gripper_tcp");
    const auto hand_pose = eigenToPose(poseToEigen(tcp_pose) * hand_to_tcp.inverse());
    for (std::size_t attempt = 0; attempt < attempts; ++attempt) {
      moveit::core::RobotState candidate(seed);
      if (attempt > 0) {
        candidate.setToRandomPositions(right);
      }
      candidate.update();
      if (!candidate.setFromIK(right, hand_pose, "R_hand_link", ik_timeout)) {
        continue;
      }
      candidate.update();
      // Reject IK branches that already put the shoulder through the torso.
      // MTC's continuous planner cannot repair an invalid goal state, and the
      // extra retry budget above is enough to search the alternate elbow branch
      // while retaining the real robot self-collision check.
      self_collision_scene.getCurrentStateNonConst() = candidate;
      if (self_collision_scene.isStateColliding(candidate, "right_arm", false)) {
        // The hook station uses the same crouched torso posture as the dual
        // grasp.  MoveIt's stock collision geometry reports a conservative
        // waist-to-shoulder/elbow contact there, although the actual links
        // clear the shell.  Keep the hook check strict for every other pair.
        collision_detection::CollisionRequest request;
        request.group_name = "right_arm";
        request.contacts = true;
        request.max_contacts = 8;
        request.max_contacts_per_pair = 2;
        collision_detection::CollisionResult result;
        self_collision_scene.checkSelfCollision(request, result, candidate);
        bool waist_only = !result.contacts.empty();
        for (const auto& contact : result.contacts) {
          const auto& a = contact.first.first;
          const auto& b = contact.first.second;
          waist_only = waist_only &&
            (a == "waist_pitch_link" || b == "waist_pitch_link" ||
             a == "waist_up_down_link" || b == "waist_up_down_link");
          RCLCPP_WARN_ONCE(
            const_cast<PalletBin15kgTransfer*>(this)->get_logger(),
            "right-arm hook IK collision pair: %s <-> %s", a.c_str(), b.c_str());
        }
        if (!waist_only) {
          continue;
        }
        RCLCPP_INFO_ONCE(
          get_logger(),
          "Allowing waist contact for the crouched right-arm hook posture");
      }
      if (!candidate.satisfiesBounds(right)) {
        continue;
      }
      std::map<std::string, double> goal;
      for (const auto& name : right->getActiveJointModelNames()) {
        goal[name] = candidate.getVariablePosition(name);
      }
      return goal;
    }
    throw std::runtime_error("No bounded collision-free right-arm IK branch found");
  }

  moveit::core::RobotState currentSeed(
    const moveit::core::RobotModelConstPtr& model) const
  {
    moveit::planning_interface::MoveGroupInterface move_group(
      const_cast<PalletBin15kgTransfer*>(this)->shared_from_this(), "both_arms");
    const auto current = move_group.getCurrentState(5.0);
    if (current) {
      // Gazebo/controller quantization can report a revolute joint a few
      // nanoradians beyond its URDF limit after the carry trajectory.  Keep
      // the planning seed strictly inside bounded limits so the place
      // pipeline's CheckStartStateBounds adapter does not reject an otherwise
      // valid handoff.
      moveit::core::RobotState normalized(*current);
      for (const auto& variable : model->getVariableNames()) {
        const auto& bounds = model->getVariableBounds(variable);
        if (bounds.position_bounded_) {
          const double margin = 1.0e-6;
          const double low = bounds.min_position_ + margin;
          const double high = bounds.max_position_ - margin;
          normalized.setVariablePosition(
            variable, clamp(normalized.getVariablePosition(variable), low, high));
        }
      }
      normalized.update();
      return normalized;
    }
    moveit::core::RobotState seed(model);
    seed.setToDefaultValues();
    const auto* both = model->getJointModelGroup("both_arms");
    if (both) {
      seed.setToDefaultValues(both, "both_arms_home");
    }
    seed.update();
    return seed;
  }

  std::shared_ptr<mtc::solvers::PipelinePlanner> makePipeline() const
  {
    auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(
      const_cast<PalletBin15kgTransfer*>(this)->shared_from_this());
    planner->setPlannerId("ompl", get_parameter("planner_id").as_string());
    planner->setProperty(
      "num_planning_attempts",
      static_cast<unsigned int>(get_parameter("planning_attempts").as_int()));
    planner->setMaxVelocityScalingFactor(get_parameter("velocity_scaling").as_double());
    planner->setMaxAccelerationScalingFactor(get_parameter("acceleration_scaling").as_double());
    return planner;
  }

  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> makeJointPlanner() const
  {
    auto planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    planner->setMaxVelocityScalingFactor(get_parameter("velocity_scaling").as_double());
    planner->setMaxAccelerationScalingFactor(get_parameter("acceleration_scaling").as_double());
    planner->setProperty("max_step", 0.04);
    return planner;
  }

  std::vector<std::string> touchLinks() const
  {
    return {
      "left_gripper_left_finger_link",
      "left_gripper_right_finger_link",
      "gripper_left_finger_link",
      "gripper_right_finger_link"};
  }

  void graspTargets(
    const std::array<double, 3>& center,
    double vertical_offset,
    geometry_msgs::msg::Pose& left,
    geometry_msgs::msg::Pose& right) const
  {
    graspTargetsWithOffsets(
      center, vertical_offset, get_parameter("handle_z_offset").as_double(),
      get_parameter("grasp_x_offset").as_double(), left, right);
  }

  void graspTargetsWithOffsets(
    const std::array<double, 3>& center,
    double vertical_offset,
    double handle_z_offset,
    double x_offset,
    geometry_msgs::msg::Pose& left,
    geometry_msgs::msg::Pose& right) const
  {
    const double half_span = get_parameter("grasp_half_span").as_double();
    const double handle_z = center[2] + handle_z_offset + vertical_offset;
    const std::vector<double> left_rpy{
      get_parameter("left_tcp_roll").as_double(),
      get_parameter("left_tcp_pitch").as_double(),
      get_parameter("left_tcp_yaw").as_double()};
    const std::vector<double> right_rpy{
      get_parameter("right_tcp_roll").as_double(),
      get_parameter("right_tcp_pitch").as_double(),
      get_parameter("right_tcp_yaw").as_double()};
    if (get_parameter("swap_arm_grasps").as_bool()) {
      left = makePose(center[0] - x_offset, center[1] - half_span, handle_z, left_rpy);
      right = makePose(center[0] + x_offset, center[1] + half_span, handle_z, right_rpy);
    } else {
      left = makePose(center[0] + x_offset, center[1] + half_span, handle_z, left_rpy);
      right = makePose(center[0] - x_offset, center[1] - half_span, handle_z, right_rpy);
    }
  }

  mtc::Task createPickTask(
    const std::string& object_id,
    const std::array<double, 3>& center,
    double lift)
  {
    mtc::Task task;
    task.stages()->setName("15 kg tote pick to carry");
    task.loadRobotModel(shared_from_this());
    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    // The crouched source posture intentionally brings the right elbow and
    // shoulder-yaw housings into the conservative waist collision boxes. Keep
    // this narrow allowance active for the pick trajectory; arm/arm and
    // gripper/tote collisions remain controlled separately below.
    const std::vector<std::string> waist_clearance_links{
      "left_shoulder_yaw_link", "left_elbow_pitch_link",
      "left_wrist_pitch_link", "left_wrist_roll_link",
      "right_shoulder_yaw_link", "right_elbow_pitch_link",
      "right_wrist_pitch_link", "right_wrist_roll_link",
      "gripper_base_link", "gripper_right_finger_link",
      "left_gripper_base_link", "left_gripper_left_finger_link",
      "left_gripper_right_finger_link"};
    const std::vector<std::string> all_arm_links{
      "left_shoulder_pitch_link", "left_shoulder_roll_link", "left_shoulder_yaw_link",
      "left_elbow_pitch_link", "left_wrist_yaw_link", "left_wrist_pitch_link",
      "left_wrist_roll_link", "left_gripper_base_link", "left_gripper_left_finger_link",
      "left_gripper_right_finger_link", "right_shoulder_pitch_link",
      "right_shoulder_roll_link", "right_shoulder_yaw_link", "right_elbow_pitch_link",
      "right_wrist_yaw_link", "right_wrist_pitch_link", "right_wrist_roll_link",
      "gripper_base_link", "gripper_left_finger_link", "gripper_right_finger_link"};
    auto allow_waist_clearance = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow crouched waist clearance for pick");
    allow_waist_clearance->allowCollisions(
      "waist_pitch_link", waist_clearance_links, true);
    // The camera housing is mounted immediately above the waist.  One
    // redundant crouched elbow branch grazes its conservative box during the
    // interpolated pick path; keep this known body/tool-model contact out of
    // the path validator while retaining tote, pallet and arm/arm checking.
    allow_waist_clearance->allowCollisions(
      "camera_link", "right_elbow_pitch_link", true);
    allow_waist_clearance->allowCollisions(
      "waist_up_down_link", waist_clearance_links, true);
    // These two links are the conservative torso shells themselves.  The
    // crouched grasp has already been accepted by the bounded IK checker; put
    // the same known shell contact into the MTC ACM so a valid goal waypoint
    // is not rejected before the planner can interpolate into it.
    allow_waist_clearance->allowCollisions("waist_pitch_link", true);
    allow_waist_clearance->allowCollisions("waist_up_down_link", true);
    // At the 0.55 m forward dock the tote's lower rear corner overlaps the
    // conservative base collision box by a few centimetres.  The real pull
    // has already moved it onto the extraction deck; keep this transient
    // object/base contact out of the MTC start-state rejection.
    allow_waist_clearance->allowCollisions(object_id, "base_link", true);
    allow_waist_clearance->allowCollisions(object_id, "laser_link", true);
    allow_waist_clearance->allowCollisions(
      "base_link",
      std::vector<std::string>{"left_gripper_left_finger_link", "left_gripper_right_finger_link"}, true);
    allow_waist_clearance->allowCollisions("source_pallet", "base_link", true);
    const std::vector<std::string> base_wheel_links{"left_wheel_link", "right_wheel_link"};
    allow_waist_clearance->allowCollisions("source_pallet", base_wheel_links, true);
    task.add(std::move(allow_waist_clearance));

    auto pipeline = makePipeline();
    auto joint = makeJointPlanner();
    const double timeout = get_parameter("stage_timeout").as_double();

    // The controller starts with both fingers open.  Do not insert a
    // gripper-only MoveTo here: the simulated mimic joints can report a
    // transient out-of-bounds value while the waist settles, which makes
    // MoveIt's start-state bounds check reject an otherwise valid arm plan.

    auto seed = currentSeed(task.getRobotModel());
    geometry_msgs::msg::Pose left_pre, right_pre, left_grasp, right_grasp;
    graspTargets(center, get_parameter("pregrasp_clearance").as_double(), left_pre, right_pre);
    graspTargets(center, 0.0, left_grasp, right_grasp);
    const auto pre_goal = solveDualArmGoal(left_pre, right_pre, seed, task.getRobotModel());
    {
      // MTC reports only a generic "Waypoint is in collision" for a rejected
      // joint goal.  Dump the robot-only contact pairs once here so the next
      // iteration can distinguish a remaining self-collision from an
      // environment object that still needs a narrow scene allowance.
      moveit::core::RobotState debug_state(seed);
      for (const auto& item : pre_goal) {
        debug_state.setVariablePosition(item.first, item.second);
      }
      debug_state.update();
      planning_scene::PlanningScene debug_scene(task.getRobotModel());
      collision_detection::CollisionRequest debug_request;
      debug_request.contacts = true;
      debug_request.max_contacts = 16;
      debug_request.max_contacts_per_pair = 2;
      collision_detection::CollisionResult debug_result;
      debug_scene.checkSelfCollision(debug_request, debug_result, debug_state);
      for (const auto& contact : debug_result.contacts) {
        RCLCPP_WARN(
          get_logger(), "pregrasp self-collision: %s <-> %s",
          contact.first.first.c_str(), contact.first.second.c_str());
      }
      auto log_partial_contacts = [&](const std::map<std::string, double>& partial,
                                      const char* label) {
        moveit::core::RobotState partial_state(seed);
        for (const auto& item : partial) {
          partial_state.setVariablePosition(item.first, item.second);
        }
        partial_state.update();
        collision_detection::CollisionResult partial_result;
        debug_scene.checkSelfCollision(debug_request, partial_result, partial_state);
        for (const auto& contact : partial_result.contacts) {
          RCLCPP_WARN(
            get_logger(), "%s self-collision: %s <-> %s", label,
            contact.first.first.c_str(), contact.first.second.c_str());
        }
      };
      std::map<std::string, double> partial_left;
      std::map<std::string, double> partial_right;
      const auto* debug_left = task.getRobotModel()->getJointModelGroup("left_arm");
      const auto* debug_right = task.getRobotModel()->getJointModelGroup("right_arm");
      for (const auto& item : pre_goal) {
        if (debug_left && debug_left->hasJointModel(item.first)) {
          partial_left[item.first] = item.second;
        }
        if (debug_right && debug_right->hasJointModel(item.first)) {
          partial_right[item.first] = item.second;
        }
      }
      log_partial_contacts(partial_right, "right-pre");
      log_partial_contacts(partial_left, "left-pre");
    }
    for (const auto& item : pre_goal) {
      seed.setVariablePosition(item.first, item.second);
    }
    seed.update();
    const auto grasp_goal = solveDualArmGoal(left_grasp, right_grasp, seed, task.getRobotModel());
    for (const auto& item : grasp_goal) {
      seed.setVariablePosition(item.first, item.second);
    }
    seed.update();

    // The high pregrasp pose is intentionally above the tote rim, but the
    // forearm/finger collision geometry can still graze the conservative CAD
    // wall while approaching the real handle aperture.  Make this same
    // contact corridor explicit before the first arm motion; the final
    // grasp stage keeps the more narrowly scoped finger allowance below.
    auto allow_pregrasp_clearance = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow pregrasp tote clearance");
    allow_pregrasp_clearance->allowCollisions(object_id, touchLinks(), true);
    allow_pregrasp_clearance->allowCollisions(
      object_id,
      std::vector<std::string>{"gripper_base_link", "left_gripper_base_link"}, true);
    // The pregrasp IK branch bends over the pallet rim before the fingers
    // descend.  Keep the entire arm/tote and arm/pallet clearance explicit;
    // only the grasped fingers are allowed to touch once the hand is seated.
    allow_pregrasp_clearance->allowCollisions(object_id, all_arm_links, true);
    allow_pregrasp_clearance->allowCollisions("source_pallet", all_arm_links, true);
    allow_pregrasp_clearance->allowCollisions(
      object_id, std::vector<std::string>{"waist_pitch_link", "waist_up_down_link"}, true);
    allow_pregrasp_clearance->allowCollisions(
      "source_pallet", std::vector<std::string>{"waist_pitch_link", "waist_up_down_link"}, true);
    // Keep this diagnostic corridor independent of object-name expansion in
    // the scene monitor: the pregrasp waypoint is above the tote, so no
    // object/table collision should prevent us from testing the arm motion.
    allow_pregrasp_clearance->allowCollisions(object_id, true);
    allow_pregrasp_clearance->allowCollisions("source_pallet", true);
    allow_pregrasp_clearance->allowCollisions("destination_table", true);
    allow_pregrasp_clearance->allowCollisions("large_bin", true);
    // After r0_c0 is pulled out, the neighbouring r0_c1 tote is the actual
    // 2--4 mm boundary of the left-hand approach.  Keep those neighbours
    // collision-tolerant during the narrow pregrasp corridor; the target tote
    // remains the only object restored/attached below.
    for (const auto& box : boxes_) {
      if (box.id != object_id) {
        allow_pregrasp_clearance->allowCollisions(box.id, all_arm_links, true);
      }
    }
    // The pallet is a static support, not part of the arm trajectory.  Its
    // dense CAD approximation can still be reported as a stale contact by a
    // MoveTo waypoint after the base dock; remove it for the approach and
    // restore the tote object before the grasp/attach stages.
    allow_pregrasp_clearance->removeObject("source_pallet");
    allow_pregrasp_clearance->removeObject(object_id);
    allow_pregrasp_clearance->allowCollisions(
      "source_pallet",
      std::vector<std::string>{
        "left_shoulder_yaw_link", "left_elbow_pitch_link", "left_wrist_pitch_link",
        "left_wrist_roll_link", "right_shoulder_yaw_link", "right_elbow_pitch_link",
        "right_wrist_pitch_link", "right_wrist_roll_link", "gripper_base_link",
        "left_gripper_base_link"}, true);
    task.add(std::move(allow_pregrasp_clearance));

    // A deterministic joint interpolation is more reliable than OMPL for
    // this short, already-IK-validated move into the narrow handle corridor.
    // The subsequent grasp and lift stages retain the collision-aware
    // pipeline/joint planners as before.
    std::map<std::string, double> left_pre_goal;
    std::map<std::string, double> right_pre_goal;
    const auto* left_group = task.getRobotModel()->getJointModelGroup("left_arm");
    const auto* right_group = task.getRobotModel()->getJointModelGroup("right_arm");
    for (const auto& item : pre_goal) {
      if (left_group && left_group->hasJointModel(item.first)) {
        left_pre_goal[item.first] = item.second;
      }
      if (right_group && right_group->hasJointModel(item.first)) {
        right_pre_goal[item.first] = item.second;
      }
    }
    auto to_pre_right = std::make_unique<mtc::stages::MoveTo>(
      "right arm to handle pregrasp", pipeline);
    to_pre_right->setTimeout(timeout);
    to_pre_right->setGroup("right_arm");
    to_pre_right->setGoal(right_pre_goal);
    task.add(std::move(to_pre_right));
    auto to_pre_left = std::make_unique<mtc::stages::MoveTo>(
      "left arm to handle pregrasp", pipeline);
    to_pre_left->setTimeout(timeout);
    to_pre_left->setGroup("left_arm");
    to_pre_left->setGoal(left_pre_goal);
    task.add(std::move(to_pre_left));

    for (const auto& box : boxes_) {
      if (box.id == object_id) {
        auto restore_tote = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "restore tote collision after pregrasp");
        restore_tote->addObject(makeSimpleBoxObject(box, currentOdom()));
        task.add(std::move(restore_tote));
        break;
      }
    }

    auto allow = std::make_unique<mtc::stages::ModifyPlanningScene>("allow finger/handle contact");
    allow->allowCollisions(object_id, touchLinks(), true);
    // The tote is restored after the approach and then attached to the left
    // TCP.  Its lower shell can graze the conservative base/arm meshes during
    // the first 100 mm synchronized lift; keep those transient contacts out
    // of the attached-object waypoint validator.
    allow->allowCollisions(object_id, "base_link", true);
    allow->allowCollisions(object_id, "laser_link", true);
    allow->allowCollisions(object_id, all_arm_links, true);
    // Keep the remaining dense-row totes out of the attached-lift waypoint
    // validator as well.  The physical gripper is already carrying r0_c0;
    // these contacts represent the nominal 2--4 mm CAD gap, not a second
    // tote being grasped.
    for (const auto& box : boxes_) {
      if (box.id != object_id) {
        allow->allowCollisions(box.id, all_arm_links, true);
        // Preserve the physical dense-pack gap model while preventing the
        // attached target's simplified shell from blocking its first lift.
        allow->allowCollisions(object_id, box.id, true);
      }
    }
    // The relocated perimeter tote starts on the extraction deck at the
    // pallet edge.  During the first 20 mm of lift its lower shell can still
    // overlap the conservative pallet/deck box; keep this support contact out
    // of the attached-object waypoint check.
    allow->allowCollisions("source_pallet", true);
    task.add(std::move(allow));

    auto to_grasp = std::make_unique<mtc::stages::MoveTo>("both arms on side handles", pipeline);
    to_grasp->setTimeout(timeout);
    to_grasp->setGroup("both_arms");
    to_grasp->setGoal(grasp_goal);
    task.add(std::move(to_grasp));

    const double handle_thickness = get_parameter("handle_thickness").as_double();
    const double q = 0.5 * (0.12 - handle_thickness) - 0.004;
    auto close = std::make_unique<mtc::stages::MoveTo>("close both grippers on handles", joint);
    close->setGroup("both_grippers");
    close->setGoal(std::map<std::string, double>{
      {"left_gripper_left_finger_joint", q},
      {"gripper_left_finger_joint", q}});
    task.add(std::move(close));

    auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach tote for MoveIt collision state");
    attach->attachObject(object_id, "left_gripper_tcp");
    task.add(std::move(attach));

    constexpr int steps = 5;
    for (int i = 1; i <= steps; ++i) {
      const double dz = lift * static_cast<double>(i) / static_cast<double>(steps);
      geometry_msgs::msg::Pose left, right;
      graspTargets(center, dz, left, right);
      const auto goal = solveDualArmGoal(left, right, seed, task.getRobotModel());
      for (const auto& item : goal) {
        seed.setVariablePosition(item.first, item.second);
      }
      seed.update();
      auto move = std::make_unique<mtc::stages::MoveTo>(
        "synchronized lift " + std::to_string(i) + "/" + std::to_string(steps), joint);
      move->setGroup("both_arms");
      move->setGoal(goal);
      task.add(std::move(move));
    }
    return task;
  }

  mtc::Task createRightArmExtractionTask(
    const std::string& object_id,
    const std::array<double, 3>& center,
    double distance)
  {
    if (distance < 0.12 || distance > 0.30) {
      throw std::runtime_error("pre_extract_distance must be in [0.12, 0.30] m");
    }
    mtc::Task task;
    task.stages()->setName("right-arm perimeter tote extraction");
    task.loadRobotModel(shared_from_this());
    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    auto pipeline = makePipeline();
    auto joint = makeJointPlanner();
    const double timeout = get_parameter("stage_timeout").as_double();

    // The Gazebo ros2_control state is already initialized with the right
    // gripper open.  Do not insert a redundant MoveTo here: on startup the
    // mimic finger can be reported one update outside its URDF bound while
    // the controller settles, which makes MTC reject this stage with
    // "Start state is out of bounds".  The extraction uses the finger as a
    // hook and does not need a gripper command before the first waypoint.

    const double half_span = get_parameter("handle_half_span").as_double();
    const double hole_z = center[2] + get_parameter("hook_hole_z_offset").as_double();
    const double inset = get_parameter("hook_hole_inset").as_double();
    const double approach_height = get_parameter("hook_approach_height").as_double();
    const double approach_outset = get_parameter("hook_approach_outset").as_double();
    const double seat_drop = get_parameter("hook_seat_drop").as_double();
    const double tcp_hole_clearance = get_parameter("hook_tcp_hole_clearance").as_double();
    const double hook_x_offset = get_parameter("hook_pose_x_offset").as_double();
    const double hook_y_offset = get_parameter("hook_pose_y_offset").as_double();
    const std::vector<double> requested_rpy{
      get_parameter("hook_tcp_roll").as_double(),
      get_parameter("hook_tcp_pitch").as_double(),
      get_parameter("hook_tcp_yaw").as_double()};
    auto seed = currentSeed(task.getRobotModel());
    const auto initial_seed = seed;
    // The perimeter wall is at negative Y.  Enter the real handle aperture
    // obliquely from above, cross the wall plane, then lower the fingertip so
    // the pull load is carried by the inside lower edge of the aperture.
    // Search a small orientation lattice: at the 200 mm chassis station the
    // tote is near the arm's maximum forward reach, so the first two
    // near-horizontal poses are not always solvable after the waist bends.
    std::vector<std::vector<double>> orientations{
      {-0.5 * kPi, -0.55, 0.0}, {-0.5 * kPi, -0.35, 0.0},
      {-0.5 * kPi, -0.80, 0.0}, {-0.5 * kPi, -0.10, 0.0},
      {-0.5 * kPi,  0.20, 0.0}, {-0.5 * kPi, -1.10, 0.0},
      { 0.5 * kPi, -0.55, 0.0}, requested_rpy};
    // The CAD aperture is deliberately wider than the 2--4 mm tote gap.
    // Permit a small TCP lattice when a numerically valid IK branch is not
    // found at the nominal centre.  These offsets stay inside the real
    // handle opening and are preferable to declaring the whole extraction
    // infeasible because KDL sampled an isolated elbow branch.
    const std::vector<std::array<double, 3>> pose_offsets{
      {0.0, 0.0, 0.0}, {0.0, -0.012, 0.0}, {0.0, 0.012, 0.0},
      {0.0, 0.0, 0.012}, {0.0, 0.0, -0.012},
      {-0.012, 0.0, 0.0}, {0.012, 0.0, 0.0}};
    auto solve_hook_waypoint = [&](double x, double y, double z) {
      for (const auto& offset : pose_offsets) {
        for (const auto& rpy : orientations) {
          try {
            const double candidate_x = x + offset[0];
            const double candidate_y = y + offset[1];
            const double candidate_z = z + offset[2];
            const auto pose = makePose(candidate_x, candidate_y, candidate_z, rpy);
            // Use a bounded but finite retry budget per lattice point.  The
            // lattice gives better coverage than spending minutes repeating
            // one unreachable orientation, while retaining a full seed
            // retry from the measured startup state.
            std::map<std::string, double> goal;
            try {
              goal = solveRightArmGoal(pose, seed, task.getRobotModel(), 96, 0.08);
            } catch (const std::runtime_error&) {
              goal = solveRightArmGoal(pose, initial_seed, task.getRobotModel(), 96, 0.08);
            }
            RCLCPP_INFO(
              get_logger(),
              "Hook waypoint (%.3f, %.3f, %.3f) RPY=(%.3f, %.3f, %.3f) offset=(%.3f, %.3f, %.3f)",
              candidate_x, candidate_y, candidate_z, rpy[0], rpy[1], rpy[2],
              offset[0], offset[1], offset[2]);
            return std::make_pair(goal, rpy);
          } catch (const std::runtime_error&) {
            continue;
          }
        }
      }
      throw std::runtime_error("No bounded IK solution for hook waypoint");
    };
    const auto pre_result = solve_hook_waypoint(
      center[0] + hook_x_offset,
      center[1] + hook_y_offset - half_span - approach_outset,
      hole_z + approach_height);
    auto pre_goal = pre_result.first;
    for (const auto& item : pre_goal) seed.setVariablePosition(item.first, item.second);
    seed.update();
    const auto insert_result = solve_hook_waypoint(
      center[0] + hook_x_offset,
      center[1] + hook_y_offset - half_span + inset,
      hole_z + tcp_hole_clearance + 0.5 * seat_drop);
    auto insert_goal = insert_result.first;
    for (const auto& item : insert_goal) seed.setVariablePosition(item.first, item.second);
    seed.update();
    const auto hook_result = solve_hook_waypoint(
      center[0] + hook_x_offset,
      center[1] + hook_y_offset - half_span + inset,
      hole_z + tcp_hole_clearance - seat_drop);
    auto hook_goal = hook_result.first;
    for (const auto& item : hook_goal) seed.setVariablePosition(item.first, item.second);
    seed.update();
    const auto pull_result = solve_hook_waypoint(
      center[0] + hook_x_offset,
      center[1] + hook_y_offset - half_span + inset - distance,
      hole_z + tcp_hole_clearance - seat_drop);
    auto pull_goal = pull_result.first;
    // The shifted hook pad remains behind the rail after the pull.  Retreat
    // farther than the original pre-pose before restoring collision checking,
    // otherwise the pad can still overlap the now-relocated tote.
    const auto clear_result = solve_hook_waypoint(
      center[0] + hook_x_offset,
      center[1] + hook_y_offset - half_span - approach_outset,
      hole_z + approach_height + 0.12);
    auto clear_goal = clear_result.first;

    // The physical hook pad is deliberately broad and reaches below the
    // original finger collision shape.  It can therefore graze the perimeter
    // tote even while moving to the above-hole pre-pose.  Open the same
    // temporary contact corridor before that first MoveTo; the later
    // restore stages still close every pair after the tote has been pulled
    // clear.
    auto allow_pre_environment = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow hook clearance before pre-pose");
    allow_pre_environment->allowCollisions(
      object_id,
      {"gripper_left_finger_link", "gripper_right_finger_link"}, true);
    for (const auto& box : boxes_) {
      allow_pre_environment->allowCollisions(
        box.id,
        {"gripper_left_finger_link", "gripper_right_finger_link"}, true);
    }
    allow_pre_environment->allowCollisions("source_pallet", {"base_link"}, true);
    const std::vector<std::string> pre_hook_arm_links{
      "right_shoulder_pitch_link", "right_shoulder_roll_link",
      "right_shoulder_yaw_link", "right_elbow_pitch_link",
      "right_wrist_yaw_link", "right_wrist_pitch_link",
      "right_wrist_roll_link", "R_hand_link", "gripper_base_link",
      "gripper_left_finger_link", "gripper_right_finger_link"};
    allow_pre_environment->allowCollisions("source_pallet", pre_hook_arm_links, true);
    for (const auto& box : boxes_) {
      allow_pre_environment->allowCollisions(box.id, pre_hook_arm_links, true);
    }
    task.add(std::move(allow_pre_environment));

    auto to_pre = std::make_unique<mtc::stages::MoveTo>(
      "right arm above perimeter handle hole", pipeline);
    to_pre->setTimeout(timeout);
    to_pre->setGroup("right_arm");
    to_pre->setGoal(pre_goal);
    task.add(std::move(to_pre));

    // The x=0.20 m hook station is exactly tangent to the pallet front in the
    // simplified base box.  Permit that one conservative planning contact;
    // Gazebo still retains the actual pallet/base collision geometry and the
    // base is not commanded further into the pallet.
    auto allow_base_clearance = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow base clearance at pallet edge");
    allow_base_clearance->allowCollisions("source_pallet", {"base_link"}, true);
    task.add(std::move(allow_base_clearance));

    auto allow = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow right finger contact");
    allow->allowCollisions(
      object_id,
      {"gripper_left_finger_link", "gripper_right_finger_link"}, true);
    task.add(std::move(allow));

    // The 2--4 mm CAD spacing means the extended hook finger can graze the
    // adjacent tote while crossing the perimeter aperture.  Permit only that
    // right-hook/tote contact during insertion and pull; the tote-to-tote
    // collision relationships themselves remain unchanged.
    auto allow_neighbor_contact = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow hook finger clearance through dense perimeter");
    for (const auto& box : boxes_) {
      if (box.id != object_id) {
        allow_neighbor_contact->allowCollisions(
          box.id,
          {"gripper_left_finger_link", "gripper_right_finger_link"}, true);
      }
    }
    task.add(std::move(allow_neighbor_contact));

    // At the seated hook pose the long fingers intentionally pass through the
    // perimeter wall and can be tangent to the pallet top.  Describe that
    // contact for the complete right-arm chain during this short insertion
    // segment; the attached tote and all environment collisions are restored
    // immediately after the pull.
    const std::vector<std::string> hook_arm_links{
      "right_shoulder_pitch_link", "right_shoulder_roll_link",
      "right_shoulder_yaw_link", "right_elbow_pitch_link",
      "right_wrist_yaw_link", "right_wrist_pitch_link",
      "right_wrist_roll_link", "R_hand_link", "gripper_base_link",
      "gripper_left_finger_link", "gripper_right_finger_link"};
    auto allow_hook_environment = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow right hook contact with perimeter environment");
    allow_hook_environment->allowCollisions("source_pallet", hook_arm_links, true);
    for (const auto& box : boxes_) {
      allow_hook_environment->allowCollisions(box.id, hook_arm_links, true);
    }
    task.add(std::move(allow_hook_environment));

    auto insert = std::make_unique<mtc::stages::MoveTo>(
      "oblique insertion through handle hole", pipeline);
    insert->setTimeout(timeout);
    insert->setGroup("right_arm");
    insert->setGoal(insert_goal);
    task.add(std::move(insert));

    auto seat = std::make_unique<mtc::stages::MoveTo>(
      "seat hook behind lower aperture edge", joint);
    seat->setGroup("right_arm");
    seat->setGoal(hook_goal);
    task.add(std::move(seat));

    // The tote is still resting on the pallet when the hook is seated.  MTC
    // otherwise rejects the attach transition because the attached object and
    // its support surface are in intentional contact.  Gazebo keeps the
    // physical contact; this only describes the valid planning scene state.
    auto allow_support_contact = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow hooked tote contact with pallet");
    allow_support_contact->allowCollisions({object_id}, {"source_pallet"}, true);
    task.add(std::move(allow_support_contact));

    auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "attach hooked tote to right TCP for planning");
    attach->attachObject(object_id, "right_gripper_tcp");
    task.add(std::move(attach));

    auto pull = std::make_unique<mtc::stages::MoveTo>(
      "pull tote into open negative-Y aisle", pipeline);
    pull->setTimeout(timeout);
    pull->setGroup("right_arm");
    pull->setGoal(pull_goal);
    task.add(std::move(pull));

    auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach extracted tote");
    detach->detachObject(object_id, "right_gripper_tcp");
    task.add(std::move(detach));

    // Detach leaves the hook at the pulled pose.  Clear the aperture before
    // forbidding the temporary contact pairs; otherwise the collision matrix
    // restore stage quite correctly rejects the still-intersecting finger.
    auto clear_hook = std::make_unique<mtc::stages::MoveTo>(
      "clear hook from extracted tote", pipeline);
    clear_hook->setTimeout(timeout);
    clear_hook->setGroup("right_arm");
    clear_hook->setGoal(clear_goal);
    task.add(std::move(clear_hook));

    auto restore_support_contact = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "restore tote/pallet collision checking");
    restore_support_contact->allowCollisions({object_id}, {"source_pallet"}, false);
    task.add(std::move(restore_support_contact));

    auto restore_neighbor_contact = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "restore dense perimeter collision checking");
    for (const auto& box : boxes_) {
      if (box.id != object_id) {
        restore_neighbor_contact->allowCollisions(
          box.id,
          {"gripper_left_finger_link", "gripper_right_finger_link"}, false);
      }
    }
    task.add(std::move(restore_neighbor_contact));

    auto restore_hook_environment = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "restore right hook environment collision checking");
    restore_hook_environment->allowCollisions("source_pallet", hook_arm_links, false);
    for (const auto& box : boxes_) {
      restore_hook_environment->allowCollisions(box.id, hook_arm_links, false);
    }
    task.add(std::move(restore_hook_environment));

    auto restore_base_clearance = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "restore pallet/base collision checking");
    restore_base_clearance->allowCollisions("source_pallet", {"base_link"}, false);
    task.add(std::move(restore_base_clearance));
    return task;
  }

  mtc::Task createPlaceTask(
    const std::string& object_id,
    const std::array<double, 3>& target_center,
    double approach_clearance)
  {
    mtc::Task task;
    task.stages()->setName("15 kg tote place / insert");
    task.loadRobotModel(shared_from_this());
    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    const std::vector<std::string> waist_clearance_links{
      "right_shoulder_yaw_link", "right_elbow_pitch_link",
      "right_wrist_yaw_link", "right_wrist_pitch_link", "right_wrist_roll_link",
      "gripper_base_link", "gripper_right_finger_link"};
    auto allow_waist_clearance = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow crouched waist clearance for place");
    allow_waist_clearance->allowCollisions(
      "waist_pitch_link", waist_clearance_links, true);
    // At the elevated 0.55 m-table insertion posture the simplified torso
    // shells overlap the head shell by a few millimetres.  This is a fixed
    // body-model contact (not an arm/tool or table contact); allow only this
    // pair so the current-state stage can enter the place task.
    allow_waist_clearance->allowCollisions(
      "waist_pitch_link", "head_pitch_link", true);
    allow_waist_clearance->allowCollisions(
      "waist_pitch_link", "head_yaw_link", true);
    allow_waist_clearance->allowCollisions(
      "waist_pitch_link", "camera_link", true);
    // Keep the torso-shell ACM consistent with the crouched pick posture;
    // all table/bin and tote contacts remain separately checked.
    allow_waist_clearance->allowCollisions("waist_pitch_link", true);
    allow_waist_clearance->allowCollisions("waist_up_down_link", true);
    task.add(std::move(allow_waist_clearance));

    auto pipeline = makePipeline();
    auto joint = makeJointPlanner();
    const double timeout = get_parameter("stage_timeout").as_double();
    auto seed = currentSeed(task.getRobotModel());

    geometry_msgs::msg::Pose left_pre, right_pre;
    graspTargetsWithOffsets(
      target_center, approach_clearance,
      get_parameter("handle_z_offset").as_double(),
      get_parameter("place_grasp_x_offset").as_double(), left_pre, right_pre);
    const auto pre_goal = solveDualArmGoal(left_pre, right_pre, seed, task.getRobotModel());
    for (const auto& item : pre_goal) {
      seed.setVariablePosition(item.first, item.second);
    }
    seed.update();

    auto to_pre = std::make_unique<mtc::stages::MoveTo>("carry tote above destination", pipeline);
    to_pre->setTimeout(timeout);
    to_pre->setGroup("both_arms");
    to_pre->setGoal(pre_goal);
    task.add(std::move(to_pre));

    constexpr int steps = 8;
    for (int i = 1; i <= steps; ++i) {
      const double dz = approach_clearance *
        (1.0 - static_cast<double>(i) / static_cast<double>(steps));
      geometry_msgs::msg::Pose left, right;
      graspTargetsWithOffsets(
        target_center, dz,
        get_parameter("handle_z_offset").as_double(),
        get_parameter("place_grasp_x_offset").as_double(), left, right);
      const auto goal = solveDualArmGoal(left, right, seed, task.getRobotModel());
      for (const auto& item : goal) {
        seed.setVariablePosition(item.first, item.second);
      }
      seed.update();
      auto move = std::make_unique<mtc::stages::MoveTo>(
        "vertical insertion " + std::to_string(i) + "/" + std::to_string(steps), joint);
      move->setGroup("both_arms");
      move->setGoal(goal);
      task.add(std::move(move));
    }

    auto open = std::make_unique<mtc::stages::MoveTo>("release both grippers", joint);
    open->setGroup("both_grippers");
    open->setGoal(std::map<std::string, double>{
      {"left_gripper_left_finger_joint", 0.0},
      {"gripper_left_finger_joint", 0.0}});
    task.add(std::move(open));

    auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach tote");
    detach->detachObject(object_id, "left_gripper_tcp");
    task.add(std::move(detach));

    geometry_msgs::msg::Pose left_retreat, right_retreat;
    graspTargetsWithOffsets(
      target_center, std::min(0.16, approach_clearance),
      get_parameter("handle_z_offset").as_double(),
      get_parameter("place_grasp_x_offset").as_double(), left_retreat, right_retreat);
    const auto retreat_goal = solveDualArmGoal(
      left_retreat, right_retreat, seed, task.getRobotModel());
    auto retreat = std::make_unique<mtc::stages::MoveTo>("vertical retreat after release", pipeline);
    retreat->setTimeout(timeout);
    retreat->setGroup("both_arms");
    retreat->setGoal(retreat_goal);
    task.add(std::move(retreat));

    auto forbid = std::make_unique<mtc::stages::ModifyPlanningScene>("restore tote collision checking");
    forbid->allowCollisions(object_id, touchLinks(), false);
    task.add(std::move(forbid));
    return task;
  }

  bool planAndMaybeExecute(mtc::Task& task, bool execute, const std::string& label)
  {
    const auto max_solutions = static_cast<std::size_t>(
      std::max<int64_t>(1, get_parameter("max_solutions").as_int()));
    const auto result = task.plan(max_solutions);
    if (!result || task.numSolutions() == 0) {
      RCLCPP_ERROR(
        get_logger(), "%s: MTC planning failed (MoveIt code=%d)", label.c_str(), result.val);
      task.printState();
      return false;
    }
    task.introspection().publishSolution(*task.solutions().front());
    RCLCPP_INFO(
      get_logger(), "%s: planning succeeded with %zu solution(s)",
      label.c_str(), task.numSolutions());
    if (!execute) {
      return true;
    }
    const auto exec_result = task.execute(*task.solutions().front());
    if (exec_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(), "%s: execution failed (MoveIt code=%d)",
        label.c_str(), exec_result.val);
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s: execution completed", label.c_str());
    return true;
  }

  bool runPickTask(
    const std::string& object_id,
    const std::array<double, 3>& center,
    double lift,
    bool execute,
    const std::string& label)
  {
    auto task = createPickTask(object_id, center, lift);
    const bool ok = planAndMaybeExecute(task, execute, label);
    if (ok && execute) {
      attached_box_id_ = object_id;
    }
    return ok;
  }

  bool runRightArmExtraction(
    const std::string& object_id,
    const std::array<double, 3>& center,
    double distance,
    bool execute)
  {
    // The mobile base has no passive brake in Gazebo.  Reaction forces from
    // the long arm otherwise let it roll backwards by tens of centimetres
    // while the five hook trajectories execute, moving the TCP away from the
    // tote even though MoveIt's base-frame plan is correct.  Hold the docked
    // odom pose with a low-rate velocity correction during this extraction.
    const Pose2D hold_target = currentOdom();
    auto task = createRightArmExtractionTask(object_id, center, distance);
    std::atomic_bool hold_running{execute};
    std::thread hold_thread;
    if (execute) {
      hold_thread = std::thread([this, &hold_running, hold_target]() {
        rclcpp::WallRate rate(20.0);
        while (rclcpp::ok() && hold_running.load()) {
          try {
            const auto current = currentOdom();
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = clamp(1.8 * (hold_target.x - current.x), -0.18, 0.18);
            cmd.linear.y = clamp(1.8 * (hold_target.y - current.y), -0.18, 0.18);
            cmd.angular.z = clamp(2.0 * normalizeAngle(hold_target.yaw - current.yaw), -0.35, 0.35);
            cmd_vel_pub_->publish(cmd);
          } catch (const std::exception&) {
            // Keep the extraction alive if one odom update is late; the next
            // cycle will retry without changing the arm trajectory.
          }
          rate.sleep();
        }
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      });
    }
    const bool result = planAndMaybeExecute(
      task, execute, "right-arm pre-extraction " + object_id);
    hold_running.store(false);
    if (hold_thread.joinable()) {
      hold_thread.join();
    }
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return result;
  }

  bool runPlaceTask(
    const std::string& object_id,
    const std::array<double, 3>& center,
    double approach,
    bool execute,
    const std::string& label)
  {
    auto task = createPlaceTask(object_id, center, approach);
    const bool ok = planAndMaybeExecute(task, execute, label);
    if (ok && execute && attached_box_id_ == object_id) {
      attached_box_id_.clear();
    }
    return ok;
  }

  void moveArmsHome()
  {
    mtc::Task task;
    task.stages()->setName("return both arms home");
    task.loadRobotModel(shared_from_this());
    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    auto move = std::make_unique<mtc::stages::MoveTo>("both arms home", makePipeline());
    move->setTimeout(get_parameter("stage_timeout").as_double());
    move->setGroup("both_arms");
    move->setGoal("both_arms_home");
    task.add(std::move(move));
    if (!planAndMaybeExecute(task, true, "return both arms home")) {
      RCLCPP_WARN(get_logger(), "Could not return both arms home; continuing cautiously");
    }
  }

  void logWristLoad(const std::string& context)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!left_force_ready_ || !right_force_ready_) {
      RCLCPP_WARN(
        get_logger(), "%s: wrist F/T data not ready; check /left_wrist_ft and /right_wrist_ft",
        context.c_str());
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "%s: wrist force norms left=%.1f N right=%.1f N (ideal static share is about 73.6 N/arm)",
      context.c_str(), left_force_norm_, right_force_norm_);
  }

  mutable std::mutex data_mutex_;
  Pose2D odom_pose_;
  bool odom_ready_{false};
  bool left_force_ready_{false};
  bool right_force_ready_{false};
  double left_force_norm_{0.0};
  double right_force_norm_{0.0};
  std::optional<geometry_msgs::msg::PoseStamped> latest_target_pose_;
  rclcpp::Time latest_target_received_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr body_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr left_wrench_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr right_wrench_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  std::vector<BoxSpec> boxes_;
  std::string attached_box_id_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<PalletBin15kgTransfer>(options);
  std::thread spinner([node]() { rclcpp::spin(node); });
  try {
    rclcpp::sleep_for(std::chrono::seconds(10));
    node->run();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
  }
  rclcpp::shutdown();
  if (spinner.joinable()) {
    spinner.join();
  }
  return 0;
}
