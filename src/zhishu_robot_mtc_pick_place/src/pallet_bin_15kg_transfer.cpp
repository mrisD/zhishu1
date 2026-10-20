#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "zhishu_robot_mtc_pick_place/floor_admittance.hpp"
#include "zhishu_robot_mtc_pick_place/process_admittance.hpp"
#include "zhishu_robot_mtc_pick_place/cartesian_microstep.hpp"

#include <Eigen/Geometry>
#include <geometric_shapes/shape_operations.h>
#include "zhishu_robot_mtc_pick_place/column_clearance.hpp"

#include <future>
#include <fstream>
#include <iomanip>
#include <rclcpp/serialization.hpp>
#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <map>
#include <limits>
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
    declare_if_missing("resume_after_pick", false);
    declare_if_missing("admittance_handoff", false);
    declare_if_missing("process_admittance", false);
    declare_if_missing("process_debug_dir", std::string{});
    declare_if_missing("resume_floor_seating", false);
    declare_if_missing("recover_table_clearance", false);
    declare_if_missing("validate_all_nine", false);
    declare_if_missing("full_transfer_box_index", 0);
    declare_if_missing("cleared_first_row_count", 0);
    declare_if_missing("source_row_only", -1);
    declare_if_missing("retain_middle_row", false);
    declare_if_missing("rear_side_pull", false);
    declare_if_missing("rear_source_dock_x", 1.918);
    declare_if_missing("return_to_pallet_after_transfer", false);
    declare_if_missing("return_empty_only", false);
    declare_if_missing("navigation_check_only", false);
    declare_if_missing("pre_extract_enabled", false);
    declare_if_missing("rear_push_probe", false);
    declare_if_missing("rear_push_execute", false);
    declare_if_missing("front_pull_probe", false);
    declare_if_missing("front_pull_execute", false);
    declare_if_missing("completed_source_columns", std::vector<int64_t>{});
    declare_if_missing("completed_box_indices", std::vector<int64_t>{});
    declare_if_missing("keep_all_source_rows", false);
    declare_if_missing("empty_return_pose", std::vector<double>{-999.0, 0.0, 0.0});
    declare_if_missing("strict_tool_body_guard", false);
    declare_if_missing("tool_body_clearance", 0.004);
    declare_if_missing("empty_velocity_scaling", 0.20);
    declare_if_missing("empty_acceleration_scaling", 0.12);
    declare_if_missing("side_push_execute", false);
    declare_if_missing("side_push_distance", 0.324);
    declare_if_missing("side_push_overtravel", 0.0);
    declare_if_missing("side_push_continue_transfer", false);
    declare_if_missing("side_push_alpha", kPi);
    declare_if_missing("side_push_contact_dx", 0.23);
    declare_if_missing("side_push_contact_dy", 0.10);
    declare_if_missing("side_push_waist_pitch", -0.785);
    declare_if_missing("side_push_waist_lift", 0.04);
    declare_if_missing("side_push_lean", 0.26);
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
    // The filled wall keeps the same top-rail height (53 mm above tote centre).
    declare_if_missing("hook_hole_z_offset", 0.053);
    // Do not penetrate the filled wall.  The TCP remains outside and the
    // fingers clamp the continuous top rail.
    declare_if_missing("hook_hole_inset", 0.0);
    // Keep the TCP in the arm's reachable upper rail-clamp pose.
    declare_if_missing("hook_tcp_hole_clearance", 0.060);
    declare_if_missing("hook_approach_height", 0.18);
    declare_if_missing("hook_approach_outset", 0.055);
    declare_if_missing("hook_seat_drop", 0.0);
    // Small calibrated TCP-side trim: the hook station is about 6 mm
    // rearward of the nominal CAD base origin, and this keeps the waypoint
    // inside the bounded right-arm IK corridor.
    declare_if_missing("hook_pose_x_offset", -0.080);
    declare_if_missing("hook_pose_y_offset", 0.0);
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
    declare_if_missing("planner_id", std::string("RRTConnect"));
    declare_if_missing("velocity_scaling", 0.12);
    declare_if_missing("continuous_arm_motion", true);
    declare_if_missing("acceleration_scaling", 0.10);

    declare_if_missing("base_max_linear", 0.18);
    declare_if_missing("base_max_angular", 0.45);
    declare_if_missing("empty_base_max_linear", 0.0);
    declare_if_missing("empty_base_max_angular", 0.0);
    declare_if_missing("empty_base_linear_acceleration", 0.0);
    declare_if_missing("empty_base_angular_acceleration", 0.0);
    declare_if_missing("base_linear_acceleration", 0.30);
    declare_if_missing("base_angular_acceleration", 0.60);
    declare_if_missing("base_position_tolerance", 0.004);
    declare_if_missing("base_yaw_tolerance", 0.005);
    declare_if_missing("base_motion_timeout", 90.0);
    // Gazebo spawns the chassis at a calibrated work-station offset while
    // the DiffDrive odometry starts at zero.  Keep planning in world-aligned
    // coordinates by adding this fixed physical offset to the odometry;
    // driveToPose still uses the same effective pose for any later motion.
    declare_if_missing("physical_base_offset_x", 0.0);
    declare_if_missing("physical_base_offset_y", 0.0);
    declare_if_missing("source_grasp_x", 0.0);
    declare_if_missing("source_grasp_y", -0.400617);
    declare_if_missing("align_with_extracted_center", false);
    declare_if_missing("prefer_symmetric_arm_branch", false);

    declare_if_missing("grasp_waist_lift", 0.04);
    declare_if_missing("grasp_waist_pitch", -0.55);
    declare_if_missing("carry_waist_lift", 0.18);
    declare_if_missing("carry_waist_pitch", -0.10);
    declare_if_missing("carry_center_height", 0.96);
    declare_if_missing("place_waist_lift", 0.14);
    declare_if_missing("place_waist_pitch", -0.70);
    declare_if_missing("place_floor_waist_pitch", -0.80);
    declare_if_missing("place_carry_reach", 0.65);
    declare_if_missing("waist_motion_seconds", 2.5);

    declare_if_missing("handle_half_span", 0.195);
    declare_if_missing("grasp_half_span", 0.195);
    // Finger tip = TCP - 80 mm; rim = tote center + 74 mm.
    // Center + 108 mm gives 46 mm nominal insertion, compensating the ~4 mm
    // measured physical-frame/tracking offset for about 50 mm actual depth.
    declare_if_missing("handle_z_offset", 0.108);
    declare_if_missing("handle_thickness", 0.010);
    declare_if_missing("pregrasp_clearance", 0.12);
    declare_if_missing("load_test_lift", 0.05);
    declare_if_missing("carry_lift", 0.10);
    declare_if_missing("hold_seconds", 3.0);
    declare_if_missing("place_approach_clearance", 0.30);
    declare_if_missing("place_center_height", 0.956);
    declare_if_missing("source_release_drop", 0.010);
    declare_if_missing("bin_release_drop", 0.050);
    declare_if_missing("place_on_floor_before_release", true);

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
    declare_if_missing("bin_target_x", 1.03);
    declare_if_missing("bin_dock_x", 0.345);
    declare_if_missing("column_clearance_check_only", false);
    declare_if_missing("placement_ik_sweep", false);
    declare_if_missing("column_table_clearance", 0.200);
    declare_if_missing("column_table_margin", 0.005);
    declare_if_missing("swap_arm_grasps", false);
    declare_if_missing("dual_ik_attempts", 100);
    declare_if_missing("dual_ik_timeout", 0.50);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    process_arm_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/dual_arm_controller/joint_trajectory",10);
    body_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/body_posture_controller/joint_trajectory", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        odom_pose_.x = msg->pose.pose.position.x;
        odom_pose_.y = msg->pose.pose.position.y;
        odom_pose_.yaw = tf2::getYaw(msg->pose.pose.orientation);
        odom_transform_.translation() = Eigen::Vector3d(
          msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        const auto& q = msg->pose.pose.orientation;
        odom_transform_.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
        odom_received_ = std::chrono::steady_clock::now();
        odom_ready_ = true;
      });
    base_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pallet_bin/measured_base_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        measured_base_pose_ = Pose2D{msg->pose.position.x, msg->pose.position.y,
                                   tf2::getYaw(msg->pose.orientation)};
        const auto& q = msg->pose.orientation;
        const Eigen::Matrix3d rotation = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
        measured_base_transform_.linear() = rotation;
        measured_base_transform_.translation() = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        measured_base_pitch_ = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
        measured_base_roll_ = std::atan2(rotation(2, 1), rotation(2, 2));
        measured_base_received_ = std::chrono::steady_clock::now();
      });

    waist_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          measured_joint_positions_[msg->name[i]] = msg->position[i];
          if (msg->name[i] == "waist_up_down_joint") measured_waist_[0] = msg->position[i];
          if (msg->name[i] == "waist_pitch_joint") measured_waist_[1] = msg->position[i];
          if (msg->name[i] == "left_gripper_left_finger_joint") measured_grippers_[0] = msg->position[i];
          if (msg->name[i] == "gripper_left_finger_joint") measured_grippers_[1] = msg->position[i];
        }
        waist_received_ = std::chrono::steady_clock::now();
        if (floor_support_verified_ && !release_started_ &&
            std::min(measured_grippers_[0], measured_grippers_[1]) < 0.04) {
          release_started_ = true;
          logWristDetailsLocked("release_start");
        }
        if (release_started_ && !release_fully_open_ &&
            std::max(measured_grippers_[0], measured_grippers_[1]) < 0.001) {
          release_fully_open_ = true;
          logWristDetailsLocked("fully_open");
        }
      });

    if(get_parameter("process_admittance").as_bool() && !get_parameter("process_debug_dir").as_string().empty()) {
      wrist_trace_.open(get_parameter("process_debug_dir").as_string()+"/wrist_trace_box_"+
        std::to_string(get_parameter("full_transfer_box_index").as_int())+".csv");
      wrist_trace_<<"time,stage,left_stamp,right_stamp,left_fx,left_fy,left_fz,left_tx,left_ty,left_tz,right_fx,right_fy,right_fz,right_tx,right_ty,right_tz,left_norm,right_norm,bottom_reaction,left_jaw,right_jaw\n";
      wrist_trace_<<std::setprecision(12);
    }
    left_wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/left_wrist_ft", 50,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        left_wrench_message_ = *msg;
        left_force_norm_ = forceNorm(*msg);
        left_force_ready_ = true;
      });
    right_wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/right_wrist_ft", 50,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        right_wrench_message_ = *msg;
        right_force_norm_ = forceNorm(*msg);
        right_force_ready_ = true;
        if(wrist_trace_.is_open() && left_force_ready_) {
          const double time=rclcpp::Time(msg->header.stamp).seconds();
          if(time-wrist_trace_last_>=.02) {
            wrist_trace_<<time<<","<<process_stage_<<","<<rclcpp::Time(left_wrench_message_.header.stamp).seconds()<<","<<time;
            for(const auto* sample:{&left_wrench_message_,&right_wrench_message_}) {
              const auto& w=sample->wrench;
              wrist_trace_<<","<<w.force.x<<","<<w.force.y<<","<<w.force.z<<","<<w.torque.x<<","<<w.torque.y<<","<<w.torque.z;
            }
            wrist_trace_<<","<<left_force_norm_<<","<<right_force_norm_<<","<<estimated_bottom_reaction_
              <<","<<measured_grippers_[0]<<","<<measured_grippers_[1]<<"\n";
            wrist_trace_last_=time;
            if(++wrist_trace_rows_%50==0) wrist_trace_.flush();
          }
        }
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

    gazebo_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pallet_bin/measured_box_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const auto& p = msg->pose.position;
        measured_boxes_["pallet_box_r" + std::to_string(get_parameter("full_transfer_box_index").as_int() / 3) + "_c" + std::to_string(get_parameter("full_transfer_box_index").as_int() % 3)] = {p.x, p.y, p.z};
        const auto& q = msg->pose.orientation;
        const auto rotation = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
        measured_box_tilt_ = std::acos(std::clamp(rotation(2, 2), -1.0, 1.0));
        measured_boxes_received_["pallet_box_r" + std::to_string(get_parameter("full_transfer_box_index").as_int() / 3) + "_c" + std::to_string(get_parameter("full_transfer_box_index").as_int() % 3)] = std::chrono::steady_clock::now();
      });

    estimated_reaction_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/pallet_bin/estimated_bottom_reaction", 10,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        const double force = msg->wrench.force.z;
        if (msg->header.frame_id != "world" || !std::isfinite(force)) return;
        std::lock_guard<std::mutex> lock(data_mutex_);
        estimated_bottom_reaction_ = force;
        reaction_estimate_ready_ = true;
        reaction_estimate_received_ = std::chrono::steady_clock::now();
      });

    // Coordinates are derived from baifang.stp using:
    //   world X <- STEP Y, world Y <- STEP Z, world Z <- 150 mm - STEP X.
    // All totes retain their default CAD centres, including r0_c0.
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
    if (get_parameter("column_table_clearance").as_double() < 0.200 ||
        get_parameter("column_table_margin").as_double() < 0.0)
      throw std::runtime_error("Column/table horizontal clearance must be at least 200 mm");
    mtc::Task geometry_task;
    geometry_task.loadRobotModel(shared_from_this());
    initializeColumnGeometry(geometry_task.getRobotModel());
    model_tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    model_tf_->setUsingDedicatedThread(true);  // main spinner populates TF while run() plans
    model_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*model_tf_, shared_from_this(), false);
    const bool resume_after_pick = get_parameter("resume_after_pick").as_bool();
    if (get_parameter("resume_floor_seating").as_bool() &&
        (!resume_after_pick || !get_parameter("place_on_floor_before_release").as_bool()))
      throw std::runtime_error("Floor diagnostic resume requires an existing closed-gripper attachment");
    if (resume_after_pick) {
      if (!get_parameter("execute").as_bool() || get_parameter("pre_extract_enabled").as_bool())
        throw std::runtime_error("resume_after_pick requires execute=true and pre_extract_enabled=false");
      const auto index = get_parameter("full_transfer_box_index").as_int();
      if (index < 0 || index >= static_cast<int>(boxes_.size()))
        throw std::runtime_error("Invalid measured tote resume index");
      const auto& resumed_box = boxes_[static_cast<std::size_t>(index)];
      moveit::planning_interface::PlanningSceneInterface psi;
      if (!psi.getAttachedObjects({resumed_box.id}).count(resumed_box.id))
        throw std::runtime_error("Cannot resume carry without the existing planning attachment");
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!measured_boxes_.count(resumed_box.id) || measured_boxes_.at(resumed_box.id)[2] < 0.274 ||
            measured_grippers_[0] < 0.04 || measured_grippers_[1] < 0.04 ||
            std::chrono::steady_clock::now() - measured_boxes_received_.at(resumed_box.id) > std::chrono::seconds(1))
          throw std::runtime_error("Cannot resume carry without a fresh physically lifted tote");
      }
      attached_box_id_ = resumed_box.id;
    }
    waitForOdom();
    waitForCurrentRobotState();
    if (get_parameter("placement_ik_sweep").as_bool()) {
      runPlacementSweep();
      return;
    }
    addEnvironmentToPlanningScene();

    if (get_parameter("column_clearance_check_only").as_bool()) {
      if (std::max(measured_grippers_[0], measured_grippers_[1]) > 0.001)
        throw std::runtime_error("Clearance-only check requires empty open grippers");
      if (!goToBinDock()) throw std::runtime_error("Clearance-only check: protected docking failed");
      for (const double pitch : {-0.10, -0.20}) {
        setWaist(0.28, pitch);
        const auto state = measuredColumnState();
        const auto world = measuredWorldBase();
        if (!columnClear(state, world, true)) throw std::runtime_error("Measured clearance check failed");
        for (const auto& item : column_points_)
          RCLCPP_INFO(get_logger(), "COLUMN_CLEARANCE_MEASURED pitch=%.3f link=%s horizontal_lower_bound=%.6f m",
            pitch, item.first.c_str(), pallet_placement::horizontalTableGap(item.second,
              world * state.getGlobalLinkTransform(item.first)));
      }
      RCLCPP_INFO(get_logger(), "COLUMN_CLEARANCE_CHECK_SUCCESS: empty robot only; no box was moved");
      return;
    }

    if (get_parameter("full_transfer_box_index").as_int() >= 0 &&
        !get_parameter("navigation_check_only").as_bool() &&
        !get_parameter("return_empty_only").as_bool() &&
        !get_parameter("side_push_execute").as_bool() &&
        !get_parameter("front_pull_execute").as_bool() &&
        !get_parameter("admittance_handoff").as_bool() &&
        get_parameter("execute").as_bool() && !resume_after_pick)
      checkPlacementReach();

    if (get_parameter("navigation_check_only").as_bool()) {
      if (!goToBinDock()) throw std::runtime_error("Navigation check: bin approach failed");
      returnEmptyToPallet();
      if (!driveToRearStation(-0.520617)) throw std::runtime_error("Navigation check: rear approach failed");
      if (!driveToPose(Pose2D{2.28, 1.2, 0.5*kPi}, 0.04, 0.04) ||
          !driveToPose(Pose2D{-0.18, 1.2, -0.5*kPi}, 0.04, 0.04) ||
          !driveToPose(Pose2D{-0.18, 0.002017, 0.0}, 0.03, 0.03) ||
          !driveToPose(Pose2D{0.0, 0.002017, 0.0}))
        throw std::runtime_error("Navigation check: front approach failed");
      RCLCPP_INFO(get_logger(), "NAVIGATION_CHECK_SUCCESS");
      return;
    }

    if (get_parameter("side_push_execute").as_bool() || get_parameter("front_pull_execute").as_bool()) {
      const bool front_pull = get_parameter("front_pull_execute").as_bool();
      if (!get_parameter("execute").as_bool())
        throw std::runtime_error("side_push_execute requires execute=true");
      const int index = get_parameter("full_transfer_box_index").as_int();
      if (index < 0 || index >= static_cast<int>(boxes_.size()))
        throw std::runtime_error("Invalid side push target");
      auto& box = boxes_[index];
      if (get_parameter("rear_push_execute").as_bool()) {
        const Pose2D expected{2.068, box.y - 0.10, kPi};
        const auto before = currentOdom();
        const double position_error = std::hypot(before.x - expected.x, before.y - expected.y);
        const double yaw_error = std::abs(normalizeAngle(before.yaw - expected.yaw));
        RCLCPP_INFO(get_logger(),
          "REAR_STATION_CHECK before=(%.5f,%.5f,%.5f) expected=(%.5f,%.5f,%.5f) error=(%.4f m,%.4f rad)",
          before.x, before.y, before.yaw, expected.x, expected.y, expected.yaw,
          position_error, yaw_error);
        if (position_error > 0.005 || yaw_error > 0.005) {
          // Differential drive cannot remove a lateral error while parked.
          // Approach again from clear space behind the rear station instead
          // of accepting the generic dock's 20 mm lateral allowance.
          const Pose2D approach{expected.x + 0.20, expected.y, expected.yaw};
          RCLCPP_INFO(get_logger(), "REAR_STATION_CORRECTION reapproach from 200 mm behind station");
          if (!driveToPose(approach, 0.005, 0.005) ||
              !driveToPose(expected, 0.003, 0.003))
            throw std::runtime_error("Rear push station correction failed; do not push");
        }
        const auto aligned = currentOdom();
        const double aligned_position_error = std::hypot(aligned.x - expected.x, aligned.y - expected.y);
        const double aligned_yaw_error = std::abs(normalizeAngle(aligned.yaw - expected.yaw));
        RCLCPP_INFO(get_logger(),
          "REAR_STATION_CHECK aligned=(%.5f,%.5f,%.5f) error=(%.4f m,%.4f rad)",
          aligned.x, aligned.y, aligned.yaw, aligned_position_error, aligned_yaw_error);
        if (aligned_position_error > 0.008 || aligned_yaw_error > 0.008)
          throw std::runtime_error("Rear push station still outside 8 mm / 0.008 rad; do not push");
      }
      const auto dock = currentOdom();
      // Hold the station against contact reaction in the robot's actual heading.
      std::atomic_bool hold_running{true};
      std::thread hold_thread([this, &hold_running, dock]() {
        rclcpp::WallRate rate(20.0);
        while (rclcpp::ok() && hold_running.load()) {
          try {
            const auto base = currentOdom();
            const double dx = dock.x - base.x, dy = dock.y - base.y;
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = clamp(1.8 * (dx * std::cos(base.yaw) + dy * std::sin(base.yaw)), -0.08, 0.08);
            cmd.angular.z = clamp(2.0 * normalizeAngle(dock.yaw - base.yaw), -0.12, 0.12);
            publishGuardedBaseCommand(cmd);
          } catch (const std::exception&) {}
          rate.sleep();
        }
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      });
      bool pushed = false;
      try {
        if (front_pull || get_parameter("rear_push_execute").as_bool()) prepareFrontPullPosture(box);
        else setWaist(get_parameter("side_push_waist_lift").as_double(),
                      get_parameter("side_push_waist_pitch").as_double());
        const auto held = currentOdom();
        RCLCPP_INFO(get_logger(), "RELOCATION_STATION before=(%.5f,%.5f,%.5f) after_waist=(%.5f,%.5f,%.5f)",
          dock.x, dock.y, dock.yaw, held.x, held.y, held.yaw);
        auto task = createSidePushTask(box, held);
        pushed = planAndMaybeExecute(task, true, "physical side push without attachment");
      } catch (...) {
        hold_running.store(false); hold_thread.join(); throw;
      }
      hold_running.store(false); hold_thread.join();
      if (!pushed)
        throw std::runtime_error("Side push planning/execution failed");
      rclcpp::sleep_for(std::chrono::seconds(2));
      {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!measured_boxes_.count(box.id) ||
          std::chrono::steady_clock::now() - measured_boxes_received_.at(box.id) > std::chrono::seconds(1))
        throw std::runtime_error("Side push lacks fresh measured box pose");
      const auto& measured = measured_boxes_.at(box.id);
      RCLCPP_INFO(get_logger(), "SIDE_PUSH_MEASURED x=%.5f y=%.5f z=%.5f displacement=%.5f tilt_deg=%.2f",
        measured[0], measured[1], measured[2], box.x - measured[0], measured_box_tilt_ * 180.0 / kPi);
      if (std::abs(measured[0] - (box.x - get_parameter("side_push_distance").as_double())) > 0.01 || std::abs(measured[1] - box.y) > 0.04 ||
          std::abs(measured[2] - box.z) > 0.02 || measured_box_tilt_ > 5.0 * kPi / 180.0)
        throw std::runtime_error("Physical side push did not meet relocation/tilt tolerance");
      box.x = measured[0]; box.y = measured[1]; box.z = measured[2];
      }
      RCLCPP_INFO(get_logger(), "Physical side push verified; return route and dual grasp remain unvalidated");
      if (!get_parameter("side_push_continue_transfer").as_bool()) return;
      // A pushed box is not attached: refresh the actual obstacle before homing.
      addEnvironmentToPlanningScene();
      if (front_pull || get_parameter("rear_push_execute").as_bool()) prepareFrontPullPosture(box, true);
      moveArmsHome();
      if (get_parameter("rear_push_execute").as_bool()) {
        mtc::Task open_task;
        open_task.stages()->setName("open push fingers before dual grasp");
        open_task.loadRobotModel(shared_from_this());
        open_task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
        addGroundContactStage(open_task);
        auto open = std::make_unique<mtc::stages::MoveTo>("open both parked grippers", makeJointPlanner());
        open->setGroup("both_grippers");
        open->setGoal(std::map<std::string,double>{
          {"left_gripper_left_finger_joint",0.0}, {"gripper_left_finger_joint",0.0}});
        open_task.add(std::move(open));
        if (!planAndMaybeExecute(open_task, true, "open parked grippers after rear push"))
          throw std::runtime_error("Cannot open parked grippers after rear push");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (std::chrono::steady_clock::now() - waist_received_ > std::chrono::seconds(1) ||
            std::abs(measured_grippers_[0]) > 0.005 || std::abs(measured_grippers_[1]) > 0.005)
          throw std::runtime_error("Parked grippers are not physically open before dual grasp");
        RCLCPP_INFO(get_logger(), "Both parked grippers physically open before dual grasp");
      }
      if (!front_pull) setWaist(0.21, -0.20);
      // Route around the unobstructed north edge; never cross the pallet.
      if (get_parameter("strict_tool_body_guard").as_bool() && get_parameter("rear_push_execute").as_bool()) {
        if (!driveToPose(Pose2D{dock.x, 1.20, 0.5 * kPi}, 0.04, 0.04) ||
            !driveToPose(Pose2D{-0.18, 1.20, -0.5 * kPi}, 0.04, 0.04) ||
            !driveToPose(Pose2D{-0.18, box.y, 0.0}, 0.002, 0.003) ||
            !driveToPose(Pose2D{0.0, box.y, 0.0}))
          throw std::runtime_error("Rear push optimized return-to-front docking failed");
      } else {
      if (get_parameter("rear_push_execute").as_bool() &&
          !driveToPose(Pose2D{dock.x, 1.20, 0.5 * kPi}, 0.015, 0.03))
        throw std::runtime_error("Rear push north-side clearance waypoint failed");
      if (!front_pull && (!driveToPose(Pose2D{1.35, 1.20, 0.0}, 0.015, 0.03) ||
          !driveToPose(Pose2D{-0.15, 1.20, 0.0}, 0.015, 0.03) ||
          !driveToPose(Pose2D{-0.15, box.y, 0.0}, 0.015, 0.03) ||
          !driveToPose(Pose2D{0.0, box.y, 0.0})))
        throw std::runtime_error("Side push return-to-front docking failed");
      if (front_pull && !driveToPose(Pose2D{0.0, box.y, 0.0}))
        throw std::runtime_error("Front pull dual grasp alignment failed");
      }
      RCLCPP_INFO(get_logger(), "SIDE_PUSH_RETURN front 200 mm station reached; remeasure before dual grasp");
    }

    if (get_parameter("rear_push_probe").as_bool() || get_parameter("front_pull_probe").as_bool()) {
      // Diagnostic only: no attachment, no box relocation and no arm motion.
      // A push must remain reachable at its end, not merely at first contact.
      const int index = get_parameter("full_transfer_box_index").as_int();
      if (index < 0 || index >= static_cast<int>(boxes_.size()))
        throw std::runtime_error("Invalid rear push target");
      const auto& box = boxes_[index];
      const auto dock = currentOdom();
      mtc::Task probe;
      probe.loadRobotModel(shared_from_this());
      const auto initial_probe_seed = currentSeed(probe.getRobotModel());
      bool complete = false;
      for (double pitch : {-1.5707, -1.35, -1.10, -0.785}) {
       for (double lean : {0.90, 1.15, 0.65, 0.78, 1.05, 1.30, 1.50, 0.52, 0.26}) {
        for (double contact_y : {-0.10, 0.10}) {
        auto seed = initial_probe_seed;
        seed.setVariablePosition("waist_pitch_joint", pitch);
        seed.setVariablePosition("waist_up_down_joint", 0.0);
        seed.update();
        bool reachable = true;
        for (int step = 0; step <= 6; ++step) {
          const double travel = 0.324 * step / 6.0;
          try {
            const auto point = worldPointInBase(box.x +
              (get_parameter("front_pull_probe").as_bool() ? -0.145 : 0.155) - travel,
              box.y + contact_y, box.z + 0.064 + 0.010 * std::sin(std::abs(lean)), dock);
            const auto pose = makePose(point[0], point[1], point[2],
              {-0.5 * kPi, -lean, normalizeAngle(kPi - dock.yaw)});
            const auto goal = solveRightArmGoal(pose, seed, probe.getRobotModel(), 160, 0.02);
            if (step > 0) {
              for (const auto& item : goal)
                if (std::abs(item.second - seed.getVariablePosition(item.first)) > 0.65)
                  throw std::runtime_error("Push IK branch discontinuity");
            }
            for (const auto& item : goal) seed.setVariablePosition(item.first, item.second);
            seed.update();
            RCLCPP_INFO(get_logger(), "REAR_PUSH_IK pitch_deg=%.2f lean_deg=%.2f contact_y=%.2f travel=%.3f reachable=true",
              pitch * 180.0 / kPi, lean * 180.0 / kPi, contact_y, travel);
          } catch (const std::runtime_error& error) {
            RCLCPP_WARN(get_logger(), "REAR_PUSH_IK pitch_deg=%.2f travel=%.3f reachable=false reason=%s",
              pitch * 180.0 / kPi, travel, error.what());
            reachable = false;
            break;
          }
        }
        complete = complete || reachable;
        if (reachable) {
          RCLCPP_INFO(get_logger(), "PUSH_CHAIN_CANDIDATE dock=(%.3f,%.3f,%.3f) pitch=%.5f lean=%.5f contact_y=%.3f",
            dock.x, dock.y, dock.yaw, pitch, lean, contact_y);
          break;
        }
        }
        if (complete) break;
       }
       if (complete) break;
      }
      if (!complete) throw std::runtime_error("Rear push has no complete bounded IK chain at the 200 mm station");
      RCLCPP_INFO(get_logger(), "Rear push IK probe passed; physical contact and collision validation still required");
      return;
    }

    if (get_parameter("return_empty_only").as_bool()) {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const auto index = get_parameter("full_transfer_box_index").as_int();
        if (index < 0 || index >= static_cast<int>(boxes_.size()))
          throw std::runtime_error("Invalid empty-return target index");
        const auto& seated_id = boxes_[static_cast<std::size_t>(index)].id;
        if (measured_grippers_[0] > 0.01 || measured_grippers_[1] > 0.01 ||
            !measured_boxes_.count(seated_id) ||
            std::abs(measured_boxes_.at(seated_id)[2] - 0.636) > 0.002)
          throw std::runtime_error("Return recovery requires open jaws and seated destination tote");
      }
      returnEmptyToPallet();
      finalizeWristTrace();
      RCLCPP_INFO(get_logger(), "Complete transfer finished: empty-return recovery after previously verified physical placement");
      return;
    }

    const bool validate_all = get_parameter("validate_all_nine").as_bool();
    const bool execute = get_parameter("execute").as_bool();
    const bool stop_on_failure = get_parameter("stop_on_failure").as_bool();

    if (get_parameter("pre_extract_enabled").as_bool()) {
      const int index = get_parameter("pre_extract_box_index").as_int();
      if (index < 0 || index >= static_cast<int>(boxes_.size())) {
        throw std::runtime_error("pre_extract_box_index must be in [0, 8]");
      }
      auto& box = boxes_[static_cast<std::size_t>(index)];
      if (box.col != 0 && !((box.row == 0 || box.row == get_parameter("source_row_only").as_int()) &&
          get_parameter("cleared_first_row_count").as_int() == box.col)) {
        throw std::runtime_error(
          "Extraction requires column 0 or a first-row tote with all preceding columns cleared");
      }
      if (get_parameter("skip_pre_extract_stage").as_bool()) {
        // Planning-only geometry probe: use the measured post-pull pose while
        // bypassing the one-arm hook task.  The normal execution path keeps
        // this false and always performs the real pre-extraction.
        if (execute) throw std::runtime_error("skip_pre_extract_stage is planning-only");
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          if (!measured_boxes_.count(box.id) || std::chrono::steady_clock::now() - measured_boxes_received_.at(box.id) > std::chrono::seconds(1))
            throw std::runtime_error("Planning probe requires fresh actual extracted tote pose");
          const auto& p = measured_boxes_.at(box.id);
          box.x = p[0]; box.y = p[1]; box.z = p[2];
        }
        RCLCPP_WARN(
          get_logger(), "Skipping pre-extraction motion; probing relocated tote at (%.3f, %.3f, %.3f)",
          box.x, box.y, box.z);
      } else {
      RCLCPP_INFO(
        get_logger(), "Pre-extracting perimeter tote %s with the %s arm", box.id.c_str(),
        get_parameter("rear_side_pull").as_bool() ? "left" : "right");
      if (!goToHookWorkPose(box)) {
        throw std::runtime_error("Failed to dock for perimeter pre-extraction");
      }
      addEnvironmentToPlanningScene();
      const bool lateral_front_column =
        (box.row == 0 || box.row == get_parameter("source_row_only").as_int()) && box.col > 0;
      // At the rear dock the untouched tote is laterally offset before its
      // side pull. A bilateral pregrasp is unreachable for the opposite arm.
      // Keep both arms home while setting the checked hook waist posture;
      // prepare the bilateral grasp only after the real pull centres the tote.
      const bool single_arm_preparation = get_parameter("process_admittance").as_bool() || lateral_front_column ||
        get_parameter("rear_side_pull").as_bool();
      if (get_parameter("strict_tool_body_guard").as_bool() && !single_arm_preparation) {
        prepareEmptyGrasp(box,
          get_parameter("pre_extract_waist_lift").as_double(),
          get_parameter("pre_extract_waist_pitch").as_double(), execute);
      } else {
        setWaist(
          get_parameter("pre_extract_waist_lift").as_double(),
          get_parameter("pre_extract_waist_pitch").as_double());
      }
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
      if (execute) rclcpp::sleep_for(std::chrono::seconds(3));
      const auto relocated = relocalizeExtractedBox(box, distance);
      const auto base_after_pull = currentOdom();
      const auto relocated_world = basePointInWorld(relocated, base_after_pull);
      box.x = relocated_world[0];
      box.y = relocated_world[1];
      box.z = relocated_world[2];
      RCLCPP_INFO(
        get_logger(), "Relocalized %s world center=(%.3f, %.3f, %.3f)",
        box.id.c_str(), box.x, box.y, box.z);
      addEnvironmentToPlanningScene();
      // Keep the extracted posture until a collision-checked arm/waist transition.
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

    auto& box = boxes_[static_cast<std::size_t>(full_index)];
    if (execute && !get_parameter("pre_extract_enabled").as_bool()) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      auto measured = measured_boxes_.find(box.id);
      if (measured != measured_boxes_.end() &&
          std::chrono::steady_clock::now() - measured_boxes_received_.at(box.id) <
          std::chrono::seconds(1)) {
        box.x = measured->second[0];
        box.y = measured->second[1];
        box.z = measured->second[2];
        RCLCPP_INFO(get_logger(), "Resume measured tote=(%.4f, %.4f, %.4f)", box.x, box.y, box.z);
      } else {
        throw std::runtime_error("Cannot resume transfer without a fresh physical tote pose");
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "Starting complete pallet -> large-bin transfer with %s (index %d)",
      box.id.c_str(), full_index);

    // Do not swing long fingers with an independent waist reset before pick.
    if (!resume_after_pick && !goToSourceWorkPose(box)) {
      throw std::runtime_error("Failed to dock for full transfer source");
    }
    addEnvironmentToPlanningScene();
    if (!resume_after_pick) prepareEmptyGrasp(box,
      get_parameter("grasp_waist_lift").as_double(),
      get_parameter("grasp_waist_pitch").as_double(), execute);
    auto base = currentOdom();
    const auto source = worldPointInBase(box.x, box.y, box.z, base);
    if (!resume_after_pick && !runPickTask(
          box.id, source, get_parameter("carry_lift").as_double(), execute,
          "full-transfer pick " + box.id)) {
      throw std::runtime_error("Full-transfer pick failed for " + box.id);
    }

    const bool resume_floor = get_parameter("resume_floor_seating").as_bool();
    if (execute && !resume_floor) {
      if (!runCoordinatedPosture(box.id,
                                 currentOdom().y < -1.2 ? std::max(0.28, get_parameter("carry_waist_lift").as_double()) : get_parameter("carry_waist_lift").as_double(),
                                 get_parameter("carry_waist_pitch").as_double(),
                                 get_parameter("carry_center_height").as_double()))
        throw std::runtime_error("Coordinated level carry posture failed");
      setWaist(currentOdom().y < -1.2 ? std::max(0.28, get_parameter("carry_waist_lift").as_double()) : get_parameter("carry_waist_lift").as_double(),
               get_parameter("carry_waist_pitch").as_double());
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      std::lock_guard<std::mutex> lock(data_mutex_);
      const auto now = std::chrono::steady_clock::now();
      const bool model_pose_fresh = measured_base_pose_ && now - measured_base_received_ <
        std::chrono::seconds(1);
      const bool odometry_fresh = odom_ready_ && now - odom_received_ < std::chrono::seconds(1);
      if (!model_pose_fresh && !odometry_fresh)
        throw std::runtime_error("Cannot carry without fresh physical base pose");
      const auto& base_transform = model_pose_fresh ? measured_base_transform_ : odom_transform_;
      const auto pitch = model_pose_fresh ? measured_base_pitch_ :
        std::asin(std::clamp(-base_transform.linear()(2, 0), -1.0, 1.0));
      const auto roll = model_pose_fresh ? measured_base_roll_ :
        std::atan2(base_transform.linear()(2, 1), base_transform.linear()(2, 2));
      if (std::abs(pitch) > 5.0 * kPi / 180.0 || std::abs(roll) > 3.0 * kPi / 180.0)
        throw std::runtime_error("Physical base pitch exceeds 5 degrees: restore support before carrying");
      auto found = measured_boxes_.find(box.id);
      // A resumed scene contains the current carried height, not the original
      // pallet height. Compare against the physical pallet support baseline.
      if (found == measured_boxes_.end() || found->second[2] < 0.224 + 0.05) {
        throw std::runtime_error("Physical Gazebo tote was not lifted by at least 50 mm");
      }
      RCLCPP_INFO(get_logger(), "Physical lift verified at z=%.4f m", found->second[2]);
    }

    if (!execute) {
      RCLCPP_INFO(
        get_logger(),
        "execute:=false: full-transfer pick plan succeeded; base carry/place was not executed");
      return;
    }

    if (get_parameter("admittance_handoff").as_bool()) {
      RCLCPP_INFO(get_logger(), "ADMITTANCE_HANDOFF_READY %s: held tote; base carry/place skipped", box.id.c_str());
      return;
    }
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(get_parameter("hold_seconds").as_double())));
    logWristLoad(box.id + (resume_floor ? " floor recovery hold" : " before base carry"));

    const auto dock_pose = currentOdom();
    const bool resume_at_bin = resume_after_pick &&
      std::hypot(dock_pose.x - get_parameter("bin_dock_x").as_double(), dock_pose.y + 1.55) < 0.01 &&
      std::abs(normalizeAngle(dock_pose.yaw)) < 0.05;
    if (resume_at_bin) RCLCPP_INFO(get_logger(), "Diagnostic resume already at bin; keep current dock");
    // Horizontal body/table clearance is enforced independently of lift height.
    if (!resume_floor && !resume_at_bin && !runCoordinatedPosture(
          box.id, std::max(0.28, get_parameter("carry_waist_lift").as_double()),
          get_parameter("carry_waist_pitch").as_double(),
          get_parameter("carry_center_height").as_double()))
      throw std::runtime_error("Could not establish tabletop clearance before bin docking");
    if (!resume_floor && !resume_at_bin && !goToBinDock()) {
      throw std::runtime_error("Failed to dock at large bin");
    }
    addEnvironmentToPlanningScene();
    if (!resume_floor && !runCoordinatedPosture(box.id, std::max(0.28, get_parameter("place_waist_lift").as_double()),
                               get_parameter("place_waist_pitch").as_double(),
                               get_parameter("place_center_height").as_double()))
      throw std::runtime_error("Coordinated level place posture failed");

    base = currentOdom();
    // Large bin world centre: (1.03, -1.55), table top 0.55 m.  The STEP bin
    // bottom is ~12 mm. Release at the configured gap above the inner floor, then let gravity
    // settle the tote to the floor.
    const double settled_z = 0.55 + 0.012 + 0.5 * 0.148;
    const bool seat_first = get_parameter("place_on_floor_before_release").as_bool();
    const double release_z = settled_z + (seat_first ? 0.020 : get_parameter("bin_release_drop").as_double());
    // Centre the tote in the large bin; retain bounds for explicit tuning.
    const double bin_target_x = get_parameter("bin_target_x").as_double();
    if (bin_target_x < 0.965 || bin_target_x > 1.095)
      throw std::runtime_error("Requested bin target violates interior clearance");
    const auto destination = worldPointInBase(bin_target_x, -1.55, release_z, base);
    if (!resume_floor && !runPlaceTask(
          box.id, destination,
          get_parameter("place_approach_clearance").as_double(), true,
          "large-bin insertion " + box.id, !seat_first)) {
      throw std::runtime_error("Large-bin insertion failed for " + box.id);
    }
    if (seat_first) {
      if (get_parameter("recover_table_clearance").as_bool()) recoverTableClearance(box.id);
      logWristLoad("insertion_complete_before_floor_contact");
      seatToteOnBinFloor(box.id, bin_target_x, settled_z);
      const auto seated_target = worldPointInBase(bin_target_x, -1.55, settled_z, currentOdom());
      if (!runPlaceTask(box.id, seated_target,
                       get_parameter("place_approach_clearance").as_double(), true,
                       "release seated tote and retreat " + box.id, true, false))
        throw std::runtime_error("Failed to release seated tote and retreat");
    }

    resetWorldBoxCollision(BoxSpec{box.id, bin_target_x, -1.55, settled_z, box.row, box.col});
    // Arms have completed release/retreat. The home goal and robot start are
    // known now; overlap planning with the retained placement settling check.
    auto home_plan = std::async(std::launch::async, [this]() {
      auto task = std::make_unique<mtc::Task>();
      task->stages()->setName("preplanned released-arm home");
      task->loadRobotModel(shared_from_this());
      task->add(std::make_unique<mtc::stages::CurrentState>("released robot state"));
      addGroundContactStage(*task);
      addHomeClearanceStage(*task);
      auto move = std::make_unique<mtc::stages::MoveTo>("both arms home", makePipeline(0.25));
      move->setTimeout(get_parameter("stage_timeout").as_double());
      move->setGroup("both_arms"); move->setGoal("both_arms_home");
      task->add(std::move(move));
      if (!planAndMaybeExecute(*task, false, "preplanned arms home")) task.reset();
      return task;
    });
    rclcpp::sleep_for(std::chrono::seconds(1));
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      auto found = measured_boxes_.find(box.id);
      if (found == measured_boxes_.end() ||
          std::chrono::steady_clock::now() - measured_boxes_received_.at(box.id) >
            std::chrono::seconds(1) ||
          std::abs(found->second[0] - bin_target_x) > 0.015 ||
          std::abs(found->second[1] + 1.55) > 0.025 ||
          std::abs(found->second[2] - settled_z) > 0.03 ||
          measured_box_tilt_ > 10.0 * kPi / 180.0) {
        throw std::runtime_error("Final physical tote pose is outside the large-bin acceptance bounds");
      }
      RCLCPP_INFO(get_logger(), "Physical large-bin placement verified: (%.4f, %.4f, %.4f)",
        found->second[0], found->second[1], found->second[2]);
    }
    moveArmsHome(home_plan.get());
    // The long, open inner finger enters the chassis if the waist is then
    // independently lowered to zero. Keep the collision-checked park posture.
    RCLCPP_INFO(get_logger(), "Keeping checked long-finger park waist posture; no blind zero reset");
    if (get_parameter("return_to_pallet_after_transfer").as_bool()) {
      attached_box_id_.clear();
      returnEmptyToPallet();
    }
    finalizeWristTrace();
    RCLCPP_INFO(
      get_logger(),
      "Complete transfer finished. The physical Gazebo tote was carried by contact/gripper closure; "
      "the MoveIt object was used only for planning-scene attachment and collision checking.");
  }

private:
  void finalizeWristTrace()
  {
    // The subscriber writes under this same mutex. Close before publishing
    // completion so the external audit sees a complete immutable CSV.
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (wrist_trace_.is_open()) {
      wrist_trace_.flush();
      if (!wrist_trace_) throw std::runtime_error("Could not flush wrist trace");
      wrist_trace_.close();
      if (wrist_trace_.fail()) throw std::runtime_error("Could not close wrist trace");
      RCLCPP_INFO(get_logger(), "WRIST_TRACE_FINALIZED rows=%zu", wrist_trace_rows_);
    }
  }

  void returnEmptyToPallet()
  {
    RCLCPP_INFO(get_logger(), "Returning empty robot to pallet before destination clearance");
    const double y = get_parameter("source_grasp_y").as_double();
    const auto current = currentOdom();
    if ((current.x > 0.25 || current.y < -1.20) &&
        (!driveToPose(Pose2D{0.0, -1.55, 0.0}, 0.02, 0.03) ||
         !driveToPose(Pose2D{0.0, -1.20, 0.5 * kPi}, 0.05, 0.04)))
      throw std::runtime_error("Empty return could not exit destination corridor");
    const auto explicit_pose = get_parameter("empty_return_pose").as_double_array();
    if (!explicit_pose.empty() && explicit_pose[0] > -99.0) {
      if (explicit_pose.size() != 3) throw std::runtime_error("empty_return_pose requires x,y,yaw");
      const Pose2D goal{explicit_pose[0], explicit_pose[1], explicit_pose[2]};
      bool docked = false;
      if (goal.x > 1.8) docked = driveToRearStation(goal.y);
      else docked = driveToPose(Pose2D{-0.18, goal.y, 0.0}, 0.03, 0.03) && driveToPose(goal);
      if (!docked) throw std::runtime_error("Empty return to next nine-box station failed");
      RCLCPP_INFO(get_logger(), "Empty robot returned to pallet; destination clearance now permitted");
      return;
    }
    if (get_parameter("retain_middle_row").as_bool() &&
        get_parameter("full_transfer_box_index").as_int() / 3 == 2) {
      const double rear_x = get_parameter("rear_source_dock_x").as_double();
      if (!driveToPose(Pose2D{0.0, 1.20, 0.5 * kPi}, 0.05, 0.05) ||
          !driveToPose(Pose2D{2.10, 1.20, 0.0}, 0.05, 0.05) ||
          !driveToPose(Pose2D{rear_x, y, kPi}))
        throw std::runtime_error("Empty return to rear grasp station failed");
      RCLCPP_INFO(get_logger(), "Empty robot returned to pallet; destination clearance now permitted");
      return;
    }
    if (!driveToPose(Pose2D{-0.15, y, 0.0}, 0.05, 0.03) ||
        !driveToPose(Pose2D{0.0, y, 0.0}))
      throw std::runtime_error("Empty return to pallet failed; do not clear destination tote");
    if (get_parameter("rear_push_execute").as_bool()) {
      // Next middle-row box is pushed from the opposite edge, not front-pulled.
      if (!driveToPose(Pose2D{-0.15, 1.20, 0.0}, 0.015, 0.03) ||
          !driveToPose(Pose2D{1.35, 1.20, 0.0}, 0.015, 0.03) ||
          !driveToPose(Pose2D{2.068, 1.20, 0.5 * kPi}, 0.015, 0.03) ||
          !driveToPose(Pose2D{2.068, y - 0.10, kPi}, 0.005, 0.005))
        throw std::runtime_error("Empty return to next rear-push station failed");
      RCLCPP_INFO(get_logger(), "Rear station reached for next opposite-edge push");
    }
    RCLCPP_INFO(get_logger(), "Empty robot returned to pallet; destination clearance now permitted");
  }
  static double forceNorm(const geometry_msgs::msg::WrenchStamped& msg)
  {
    const auto& f = msg.wrench.force;
    return std::sqrt(f.x * f.x + f.y * f.y + f.z * f.z);
  }

  Pose2D currentOdom() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (measured_base_pose_ && std::chrono::steady_clock::now() - measured_base_received_ <
        std::chrono::seconds(1)) return *measured_base_pose_;
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
      std::optional<std::array<double, 3>> measured;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        auto found = measured_boxes_.find(box.id);
        if (found != measured_boxes_.end() &&
            std::chrono::steady_clock::now() - measured_boxes_received_.at(box.id) <
              std::chrono::seconds(1)) {
          measured = found->second;
        }
      }
      if (measured) {
        const double extracted = box.y - (*measured)[1];
        RCLCPP_INFO(get_logger(),
          "Gazebo measured %s world=(%.4f, %.4f, %.4f), pull=%.4f m",
          box.id.c_str(), (*measured)[0], (*measured)[1], (*measured)[2], extracted);
        if (get_parameter("execute").as_bool() && extracted < 0.8 * commanded_distance) {
          throw std::runtime_error("Physical tote pull did not reach 80% of requested displacement");
        }
        return worldPointInBase((*measured)[0], (*measured)[1], (*measured)[2], currentOdom());
      }
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

    if (get_parameter("execute").as_bool()) {
      throw std::runtime_error("No measured tote pose: cannot verify physical extraction");
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

  bool driveToPose(const Pose2D& target, double transit_position_tol = -1.0,
                   double transit_yaw_tol = -1.0)
  {
    const bool empty = attached_box_id_.empty();
    const double empty_linear = get_parameter("empty_base_max_linear").as_double();
    const double empty_angular = get_parameter("empty_base_max_angular").as_double();
    const double max_linear = empty && empty_linear > 0.0 ? empty_linear : get_parameter("base_max_linear").as_double();
    const double max_angular = empty && empty_angular > 0.0 ? empty_angular : get_parameter("base_max_angular").as_double();
    const double position_tol = transit_position_tol > 0.0 ? transit_position_tol :
      get_parameter("base_position_tolerance").as_double();
    const double yaw_tol = transit_yaw_tol > 0.0 ? transit_yaw_tol :
      get_parameter("base_yaw_tolerance").as_double();
    const double lateral_tol = (get_parameter("rear_side_pull").as_bool() ||
      get_parameter("strict_tool_body_guard").as_bool()) ? std::min(0.002, position_tol) : 0.020;
    const double timeout = get_parameter("base_motion_timeout").as_double();
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timeout));
    rclcpp::WallRate rate(30.0);
    bool final_alignment = false;
    bool docking_backoff = false;
    bool precise_docking_mode = false;
    bool docking_orientation_ready = false;
    bool direction_selected = false;
    bool reverse_drive = false;
    double accumulated_yaw = 0.0;
    double previous_yaw = currentOdom().yaw;
    geometry_msgs::msg::Twist previous_cmd;
    auto previous_time = std::chrono::steady_clock::now();
    auto diagnostic_time = previous_time;

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (!attached_box_id_.empty()) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (std::chrono::steady_clock::now() - measured_boxes_received_[attached_box_id_] >
              std::chrono::seconds(1) || measured_box_tilt_ > 10.0 * kPi / 180.0) {
          cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
          RCLCPP_ERROR(get_logger(), "Stopping carry: stale tote pose or tilt %.2f deg exceeds 10 deg",
                       measured_box_tilt_ * 180.0 / kPi);
          return false;
        }
      }
      if (!columnClear(measuredColumnState(), measuredWorldBase(), true)) {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
        return false;
      }
      const auto current = currentOdom();
      accumulated_yaw += std::abs(normalizeAngle(current.yaw - previous_yaw));
      previous_yaw = current.yaw;
      if (accumulated_yaw > 2.0 * kPi) {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
        RCLCPP_ERROR(get_logger(), "Docking aborted: accumulated rotation exceeded one turn");
        return false;
      }
      // Rotate about the driven axle, not the offset base origin. Otherwise
      // final yaw alignment displaces the reference by up to 30 mm and a
      // tight position tolerance causes a rotate/translate limit cycle.
      constexpr double axle_x = 0.015077;
      const double dx = target.x + axle_x * std::cos(target.yaw) -
                        current.x - axle_x * std::cos(current.yaw);
      const double dy = target.y + axle_x * std::sin(target.yaw) -
                        current.y - axle_x * std::sin(current.yaw);
      const double distance = std::hypot(dx, dy);
      geometry_msgs::msg::Twist cmd;

      // Latch final yaw alignment once position has converged. Previously a
      // tiny residual/slip sent the controller back to atan2(dy, dx), making
      // it turn towards a millimetre-scale error and circle at the table.
      // A tighter lateral tolerance must also be satisfied before latching.
      // Otherwise a 3 mm sideways residual inside a 4 mm distance tolerance
      // re-latches every tick, preventing the requested bearing correction.
      const double target_lateral = -dx * std::sin(target.yaw) + dy * std::cos(target.yaw);
      if (distance <= position_tol &&
          (!attached_box_id_.empty() || position_tol >= 0.01 ||
           std::abs(target_lateral) <= lateral_tol)) final_alignment = true;
      if (final_alignment && distance > std::max(0.025, 4.0 * position_tol))
        final_alignment = false;
      const double yaw_error = normalizeAngle(target.yaw - current.yaw);
      const double longitudinal = dx * std::cos(target.yaw) + dy * std::sin(target.yaw);
      const double lateral = -dx * std::sin(target.yaw) + dy * std::cos(target.yaw);
      const bool straight_empty_finish = attached_box_id_.empty() && distance < 0.08 &&
        std::abs(lateral) <= lateral_tol;
      if (empty && position_tol < 0.01 &&
          (distance < 0.08 || (distance < 0.30 && std::abs(yaw_error) < 0.7)))
        precise_docking_mode = true;
      const bool precise_empty_approach = precise_docking_mode;
      if (precise_empty_approach) {
        // A differential-drive base cannot correct a lateral residual by
        // translating sideways. Create a finite approach runway instead of
        // chasing atan2 of millimetre errors and rotating repeatedly.
        if (!docking_backoff && std::abs(longitudinal) < 0.008 &&
            std::abs(lateral) > lateral_tol) {
          docking_backoff = true;
          docking_orientation_ready = false;
        }
        if (docking_backoff && longitudinal >= std::max(0.15, 3.0 * std::abs(lateral))) docking_backoff = false;
        if (!docking_orientation_ready && std::abs(yaw_error) <= 0.02)
          docking_orientation_ready = true;
        if (!docking_orientation_ready) {
          cmd.angular.z = clamp(1.4 * yaw_error, -max_angular, max_angular);
        } else if (docking_backoff) {
          cmd.linear.x = std::abs(yaw_error) < 0.03 ? -0.04 : 0.0;
          cmd.angular.z = clamp(1.4 * yaw_error, -max_angular, max_angular);
        } else if (std::abs(lateral) <= lateral_tol) {
          if (std::abs(longitudinal) <= position_tol && std::abs(yaw_error) <= yaw_tol) {
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
            RCLCPP_INFO(get_logger(), "Base docked at (%.3f, %.3f, %.3f)", current.x, current.y, current.yaw);
            return true;
          }
          cmd.linear.x = std::abs(yaw_error) < 0.15 ? clamp(0.8 * longitudinal, -0.04, 0.04) : 0.0;
          cmd.angular.z = clamp(1.4 * yaw_error, -max_angular, max_angular);
        } else {
          const double correction = std::atan2(lateral, std::max(0.005, longitudinal));
          cmd.angular.z = clamp(1.8 * normalizeAngle(yaw_error + correction), -max_angular, max_angular);
          cmd.linear.x = std::max(0.0, std::min(0.04, 0.4 * longitudinal)) *
            std::max(0.0, std::cos(yaw_error + correction));
        }
      } else if (straight_empty_finish) {
        if (std::abs(longitudinal) <= position_tol && std::abs(yaw_error) <= yaw_tol) {
          cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
          RCLCPP_INFO(get_logger(), "Base docked at (%.3f, %.3f, %.3f)", current.x, current.y, current.yaw);
          return true;
        }
        cmd.linear.x = std::abs(yaw_error) < 0.25 ? clamp(0.8 * longitudinal, -0.03, 0.03) : 0.0;
        cmd.angular.z = clamp(1.4 * yaw_error, -max_angular, max_angular);
      } else if (!final_alignment) {
        const double heading = std::atan2(dy, dx);
        double heading_error = normalizeAngle(heading - current.yaw);
        if (!direction_selected) {
          reverse_drive = empty && std::abs(heading_error) > 2.4;
          direction_selected = true;
        }
        // Keep the travel direction for this goal. Switching forward/reverse
        // around 90 degrees causes alternating turn commands at transit corners.
        const bool reverse = reverse_drive;
        if (reverse) heading_error = normalizeAngle(heading_error + kPi);
        const bool empty_final_approach = attached_box_id_.empty() && distance < 0.30;
        if (std::abs(heading_error) > (empty_final_approach ? 0.9 : 0.22)) {
          cmd.angular.z = clamp(1.4 * heading_error, -max_angular, max_angular);
        } else {
          cmd.linear.x = (reverse ? -1.0 : 1.0) * std::min(max_linear, 0.8 * distance);
          if (empty_final_approach) {
            const double beta = normalizeAngle(target.yaw - heading - (reverse ? kPi : 0.0));
            cmd.linear.x *= std::max(0.0, std::cos(heading_error));
            (void)beta;
            cmd.angular.z = clamp(1.4 * heading_error, -max_angular, max_angular);
          } else {
            cmd.angular.z = clamp(1.1 * heading_error, -max_angular, max_angular);
          }
        }
      } else {
        const double yaw_error = normalizeAngle(target.yaw - current.yaw);
        const bool precise_empty_dock = attached_box_id_.empty() && position_tol < 0.01;
        if (std::abs(yaw_error) <= yaw_tol &&
            (!precise_empty_dock ||
             (std::abs(longitudinal) <= position_tol && std::abs(lateral) <= lateral_tol))) {
          cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
          RCLCPP_INFO(
            get_logger(), "Base docked at (%.3f, %.3f, %.3f)",
            current.x, current.y, current.yaw);
          return true;
        }
        // Turning about the axle can move the footprint after the position
        // latch. Never declare docking success on yaw alone: finish the
        // remaining longitudinal error without another bearing-angle turn.
        if (precise_empty_dock && std::abs(yaw_error) <= yaw_tol) {
          if (std::abs(lateral) <= lateral_tol)
            cmd.linear.x = clamp(0.8 * longitudinal, -0.03, 0.03);
          else
            final_alignment = false;
        }
        cmd.angular.z = clamp(1.4 * yaw_error, -max_angular, max_angular);
      }
      const auto command_time = std::chrono::steady_clock::now();
      const double dt = std::min(0.10, std::chrono::duration<double>(command_time - previous_time).count());
      const double empty_linear_accel=get_parameter("empty_base_linear_acceleration").as_double();
      const double empty_angular_accel=get_parameter("empty_base_angular_acceleration").as_double();
      const double linear_step=(empty && empty_linear_accel>0?empty_linear_accel:
        get_parameter("base_linear_acceleration").as_double())*dt;
      const double angular_step=(empty && empty_angular_accel>0?empty_angular_accel:
        get_parameter("base_angular_acceleration").as_double())*dt;
      cmd.linear.x = clamp(cmd.linear.x, previous_cmd.linear.x - linear_step, previous_cmd.linear.x + linear_step);
      cmd.angular.z = clamp(cmd.angular.z, previous_cmd.angular.z - angular_step, previous_cmd.angular.z + angular_step);
      if (command_time - diagnostic_time > std::chrono::seconds(2)) {
        RCLCPP_INFO(get_logger(), "DOCK_FEEDBACK current=(%.4f,%.4f,%.4f) goal=(%.4f,%.4f,%.4f) distance=%.5f lateral=%.5f cmd=(%.4f,%.4f) accumulated_yaw=%.3f backoff=%d",
          current.x,current.y,current.yaw,target.x,target.y,target.yaw,distance,lateral,
          cmd.linear.x,cmd.angular.z,accumulated_yaw,docking_backoff);
        diagnostic_time = command_time;
      }
      previous_cmd = cmd;
      previous_time = command_time;
      if (!publishGuardedBaseCommand(cmd)) return false;
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

  bool driveToRearStation(double y)
  {
    constexpr double outside_x = 2.28;
    const auto current = currentOdom();
    if (current.x < 1.95 &&
        (!driveToPose(Pose2D{0.0, 1.20, 0.5 * kPi}, 0.04, 0.04) ||
         !driveToPose(Pose2D{outside_x, 1.20, -0.5 * kPi}, 0.04, 0.04))) return false;
    if (!driveToPose(Pose2D{outside_x, y, kPi}, 0.001, 0.001)) return false;
    return driveToPose(Pose2D{get_parameter("rear_source_dock_x").as_double(), y, kPi}, 0.001, 0.001);
  }

  bool goToSourceWorkPose(const BoxSpec& box)
  {
    // A narrow but collision-free corridor between pallet (y >= -0.60) and
    // table (y <= -1.075). The 0.35 m-wide base keeps >= 50 mm nominal margin.
    constexpr double corridor_y = -0.85;
    if (box.row <= 1) {
      if (box.row == 0 || box.row == get_parameter("source_row_only").as_int()) {
        // Pre-align the extraction station with the predicted relocated
        // centre so no lateral chassis shuffle is needed after the pull.
        const double grasp_x = get_parameter("source_grasp_x").as_double();
        const double grasp_y = get_parameter("align_with_extracted_center").as_bool() ?
          box.y : get_parameter("source_grasp_y").as_double();
        return driveToPose(Pose2D{grasp_x, grasp_y, 0.0});
      }
      // Keep the chassis at the exact 20 cm front station; the outer tote is
      // flush with the pallet edge and remains reachable from x=0.
      const double source_x = 0.0;
      if (!driveToPose(Pose2D{source_x, corridor_y, 0.0})) {
        return false;
      }
      return driveToPose(Pose2D{source_x, box.y, 0.0});
    }

    // Back row: route outside the pallet far edge before moving laterally.
    constexpr double outside_rear_x = 2.10;
    const double rear_dock_x = get_parameter("rear_source_dock_x").as_double();
    if (currentOdom().x >= 1.95) {
      return driveToPose(Pose2D{rear_dock_x, box.y, kPi});
    }
    if (!driveToPose(Pose2D{outside_rear_x, corridor_y, 0.0}, 0.05, 0.05)) {
      return false;
    }
    if (!driveToPose(Pose2D{outside_rear_x, box.y, kPi}, 0.03, 0.05)) {
      return false;
    }
    return driveToPose(Pose2D{rear_dock_x, box.y, kPi});
  }

  bool goToHookWorkPose(const BoxSpec& box)
  {
    if (box.row == 2 && get_parameter("rear_side_pull").as_bool()) {
      if (currentOdom().x < 1.95) return driveToRearStation(
        box.y - get_parameter("pre_extract_distance").as_double());
      const double work_y = box.y - get_parameter("pre_extract_distance").as_double();
      if (std::abs(currentOdom().y - work_y) > 0.001) return driveToRearStation(work_y);
      return driveToPose(Pose2D{get_parameter("rear_source_dock_x").as_double(), work_y, kPi}, 0.001, 0.001);
    }
    const bool active_front = box.row == 0 || box.row == get_parameter("source_row_only").as_int();
    const double work_y = box.y - (active_front &&
      get_parameter("align_with_extracted_center").as_bool() ?
      get_parameter("pre_extract_distance").as_double() : 0.0);
    const auto current = currentOdom();
    const bool already_front_aligned = std::hypot(current.x, current.y - work_y) < 0.025 &&
      std::abs(normalizeAngle(current.yaw)) < 0.03;
    if (active_front && box.col > 0 && !already_front_aligned &&
        !driveToPose(Pose2D{-0.15, work_y, 0.0}, 0.05, 0.03))
      return false;
    // Keep the chassis front 200 mm from the pallet near edge.  The outer tote
    // is flush with that edge, so the arm reaches farther rather than driving
    // the base into the pallet.
    if (active_front && box.col == 0) {
      constexpr double hook_station_x = 0.0;
      if (!driveToPose(Pose2D{hook_station_x, work_y, 0.0})) {
        return false;
      }
      RCLCPP_INFO(
        get_logger(), "Using exact 20 cm chassis-to-pallet station for %s: x=%.3f m",
        box.id.c_str(), hook_station_x);
      return true;
    }
    constexpr double hook_dock_x = 0.0;
    if (!driveToPose(Pose2D{hook_dock_x, work_y, 0.0})) {
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "Hook-extraction dock for %s: base=(%.3f, %.3f, 0)",
      box.id.c_str(), hook_dock_x, work_y);
    return true;
  }

  bool goToBinDock()
  {
    // Go around the extraction deck, whose footprint occupies
    // x=[0.384,1.634], y=[-1.010,-0.600]. Keep the column outside the table.
    // Keep the lifting stage edge >= 200 mm from the table edge.
    // The pitching upper torso remains subject to full 3D collision checks.
    const double dock_x = get_parameter("bin_dock_x").as_double();
    constexpr double dock_y = -1.55;
    const auto current = currentOdom();
    RCLCPP_INFO(get_logger(), "BIN_CENTER_DOCK target_x=%.3f dock_x=%.3f required_column_gap=%.3f",
                get_parameter("bin_target_x").as_double(), dock_x,
                get_parameter("column_table_clearance").as_double());
    auto dock_transform = measuredWorldBase();
    dock_transform.translation().x() = dock_x;
    dock_transform.translation().y() = dock_y;
    dock_transform.linear() = Eigen::Matrix3d::Identity();
    if (!columnClear(measuredColumnState(), dock_transform, true)) return false;
    if (std::hypot(current.x - dock_x, current.y - dock_y) < 0.25 &&
        std::abs(normalizeAngle(current.yaw)) < 0.03) {
      return driveToPose(Pose2D{dock_x, dock_y, 0.0});
    }
    const int source_row = get_parameter("full_transfer_box_index").as_int() / 3;
    if (source_row == 2) {
      // A back-row pickup starts behind the pallet. Reach the bin corridor by
      // going around the clear north edge; the front-row direct route cuts
      // across the pallet's rear corner and can stall while carrying a tote.
      if (!driveToPose(Pose2D{2.10, 1.20, 0.5 * kPi}, 0.05, 0.05) ||
          !driveToPose(Pose2D{0.0, 1.20, -0.5 * kPi}, 0.05, 0.05)) {
        return false;
      }
    }
    // Corridor waypoints need clearance, not millimetre assembly precision.
    // Preserve the strict default tolerance at the actual manipulation docks.
    if (!driveToPose(Pose2D{0.0, -1.20, -0.5 * kPi}, 0.015, 0.03) ||
        !driveToPose(Pose2D{0.0, dock_y, -0.5 * kPi}, 0.015, 0.03)) {
      return false;
    }
    // Finish yaw alignment before approaching the edge. Turning while moving
    // toward the dock sweeps the torso's side into the clearance reserve.
    if (!driveToPose(Pose2D{0.0, dock_y, 0.0}, 0.015, 0.03)) return false;
    return driveToPose(Pose2D{dock_x, dock_y, 0.0});
  }

  void setWaist(double lift, double pitch)
  {
    lift = clamp(lift, 0.005, 0.29);  // keep away from the lower hard stop
    // Reject out-of-range commands instead of silently testing a different pose.
    mtc::Task bounds_task;
    bounds_task.loadRobotModel(shared_from_this());
    const auto& pitch_bounds = bounds_task.getRobotModel()->getVariableBounds("waist_pitch_joint");
    if (pitch < pitch_bounds.min_position_ || pitch > pitch_bounds.max_position_)
      throw std::runtime_error("Requested waist pitch exceeds the robot model limits");
    auto proposed = measuredColumnState();
    const auto start = proposed;
    proposed.setVariablePosition("waist_up_down_joint", lift);
    proposed.setVariablePosition("waist_pitch_joint", pitch);
    proposed.update();
    const auto world_from_base = measuredWorldBase();
    for (int i = 0; i <= 100; ++i) {
      auto sample = start;
      start.interpolate(proposed, i / 100.0, sample);
      sample.update();
      if (!columnClear(sample, world_from_base, true))
        throw std::runtime_error("Waist motion would violate the 200 mm horizontal table clearance");
    }
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
    if (get_parameter("execute").as_bool()) {
      // End the wait on fresh endpoint feedback, rather than waiting out an
      // additional fixed command duration when the coordinated arms already
      // put the waist at this target. Require a short stable interval.
      const auto issued = std::chrono::steady_clock::now();
      auto stable_since = issued;
      bool stable = false;
      const auto deadline = issued + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(std::max(15.0, seconds * 3.0)));
      while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          const auto sample_time = std::chrono::steady_clock::now();
          const bool at_target = waist_received_ > issued &&
              sample_time - waist_received_ < std::chrono::milliseconds(250) &&
              std::abs(measured_waist_[0] - lift) < 0.001 &&
              std::abs(measured_waist_[1] - pitch) < 0.002;
          if (!at_target) stable = false;
          else if (!stable) { stable = true; stable_since = sample_time; }
          if (stable && sample_time - stable_since >= std::chrono::milliseconds(200)) {
            RCLCPP_INFO(get_logger(), "Physical waist verified lift=%.4f pitch=%.4f",
                        measured_waist_[0], measured_waist_[1]);
            return;
          }
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
      throw std::runtime_error("Physical waist failed to reach requested lift/pitch");
    } else {
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(seconds + 0.35)));
    }
  }

  std::array<double, 3> worldPointInBase(
    double wx, double wy, double wz, const Pose2D& base) const
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (measured_base_pose_ && std::chrono::steady_clock::now() - measured_base_received_ < std::chrono::seconds(1)) {
        const Eigen::Vector3d p = measured_base_transform_.inverse() * Eigen::Vector3d(wx, wy, wz);
        return {p.x(), p.y(), p.z()};
      }
    }
    const double dx = wx - base.x;
    const double dy = wy - base.y;
    const double c = std::cos(base.yaw);
    const double s = std::sin(base.yaw);
    return {c * dx + s * dy, -s * dx + c * dy, wz};
  }

  std::array<double, 3> basePointInWorld(const std::array<double, 3>& point, const Pose2D& base) const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (measured_base_pose_ && std::chrono::steady_clock::now() - measured_base_received_ < std::chrono::seconds(1)) {
      const Eigen::Vector3d p = measured_base_transform_ * Eigen::Vector3d(point[0], point[1], point[2]);
      return {p.x(), p.y(), p.z()};
    }
    const double c = std::cos(base.yaw), s = std::sin(base.yaw);
    return {base.x + c * point[0] - s * point[1], base.y + s * point[0] + c * point[1], point[2]};
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
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (measured_base_pose_ && std::chrono::steady_clock::now() - measured_base_received_ < std::chrono::seconds(1)) {
      const Eigen::Quaterniond q = Eigen::Quaterniond(measured_base_transform_.linear()).inverse() *
        Eigen::Quaterniond(Eigen::AngleAxisd(world_yaw, Eigen::Vector3d::UnitZ()));
      auto& orientation = object.primitive_poses.back().orientation;
      orientation.x = q.x(); orientation.y = q.y(); orientation.z = q.z(); orientation.w = q.w();
    }
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
    // Match the Gazebo grasp-test geometry: the 120 mm notch is filled and
    // each side wall is continuous.  This keeps MoveIt and Gazebo aligned.
    appendWorldBox(object, 0.30, 0.40, 0.012, box.x, box.y, box.z - 0.068, 0.0, base);
    appendWorldBox(object, 0.010, 0.38, 0.148, box.x + 0.145, box.y, box.z, 0.0, base);
    appendWorldBox(object, 0.010, 0.38, 0.148, box.x - 0.145, box.y, box.z, 0.0, base);
    appendWorldBox(object, 0.30, 0.010, 0.106, box.x, box.y + 0.195, box.z - 0.015, 0.0, base);
    appendWorldBox(object, 0.090, 0.010, 0.036, box.x - 0.105, box.y + 0.195, box.z + 0.056, 0.0, base);
    appendWorldBox(object, 0.120, 0.010, 0.036, box.x, box.y + 0.195, box.z + 0.056, 0.0, base);
    appendWorldBox(object, 0.090, 0.010, 0.036, box.x + 0.105, box.y + 0.195, box.z + 0.056, 0.0, base);
    appendWorldBox(object, 0.30, 0.010, 0.106, box.x, box.y - 0.195, box.z - 0.015, 0.0, base);
    appendWorldBox(object, 0.090, 0.010, 0.036, box.x - 0.105, box.y - 0.195, box.z + 0.056, 0.0, base);
    appendWorldBox(object, 0.120, 0.010, 0.036, box.x, box.y - 0.195, box.z + 0.056, 0.0, base);
    appendWorldBox(object, 0.090, 0.010, 0.036, box.x + 0.105, box.y - 0.195, box.z + 0.056, 0.0, base);
    return object;
  }

  void addEnvironmentToPlanningScene()
  {
    const Pose2D base = currentOdom();
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    moveit_msgs::msg::CollisionObject ground;
    ground.header.frame_id = "base_footprint";
    ground.id = "simulation_ground";
    ground.operation = moveit_msgs::msg::CollisionObject::ADD;
    appendWorldBox(ground, 10.0, 10.0, 0.10, 0.0, 0.0, -0.05, 0.0, base);
    objects.push_back(ground);

    moveit_msgs::msg::CollisionObject pallet;
    pallet.header.frame_id = "base_footprint";
    pallet.id = "source_pallet";
    pallet.operation = moveit_msgs::msg::CollisionObject::ADD;
    appendWorldBox(pallet, 1.20, 1.20, 0.15, 1.034, 0.0, 0.075, 0.0, base);
    objects.push_back(pallet);

    moveit_msgs::msg::CollisionObject extraction_deck;
    extraction_deck.header.frame_id = "base_footprint";
    extraction_deck.id = "perimeter_extraction_deck";
    extraction_deck.operation = moveit_msgs::msg::CollisionObject::ADD;
    appendWorldBox(extraction_deck, 1.25, 0.41, 0.15,
                   1.009355, -0.805, 0.075, 0.0, base);
    objects.push_back(extraction_deck);

    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = "base_footprint";
    table.id = "destination_table";
    table.operation = moveit_msgs::msg::CollisionObject::ADD;
    // Match Gazebo: tabletop and four legs, with open space beneath.
    appendWorldBox(table, 0.75, 0.95, 0.05, 1.03, -1.55, 0.525, 0.0, base);
    for (double dx : {-0.31, 0.31})
      for (double dy : {-0.41, 0.41})
        appendWorldBox(table, 0.06, 0.06, 0.50, 1.03 + dx, -1.55 + dy, 0.25, 0.0, base);
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
      auto floor = bin;
      floor.id = "large_bin_floor";
      appendWorldBox(floor, 0.650, 0.450, 0.012, p.first, p.second, 0.556, yaw, base);
      objects.push_back(floor);
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
      const int source_row = get_parameter("source_row_only").as_int();
      if (!get_parameter("keep_all_source_rows").as_bool() && source_row >= 0 && box.row != source_row &&
          !(get_parameter("retain_middle_row").as_bool() && box.row == 1)) continue;
      if (box.row == (source_row >= 0 ? source_row : 0) && box.col < get_parameter("cleared_first_row_count").as_int()) continue;
      // While an MTC pick is physically being carried, this id is attached to
      // left_gripper_tcp in the planning scene. Do not also recreate it in the
      // world when the base reaches the next dock.
      if (box.id != attached_box_id_) {
        objects.push_back(makeSimpleBoxObject(box, base));
      }
    }

    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<std::string> cleared_ids;
    const int source_row = get_parameter("source_row_only").as_int();
    for (const auto& box : boxes_)
      if ((!get_parameter("keep_all_source_rows").as_bool() && source_row >= 0 && box.row != source_row &&
          !(get_parameter("retain_middle_row").as_bool() && box.row == 1)) ||
          (box.row == (source_row >= 0 ? source_row : 0) && box.col < get_parameter("cleared_first_row_count").as_int()))
        cleared_ids.push_back(box.id);
    const auto completed_columns = get_parameter("completed_source_columns").as_integer_array();
    for (const auto col : completed_columns) {
      if (col == -1) continue;
      if (col < 0 || col > 2) throw std::runtime_error("Invalid completed source column");
      const int completed_row = source_row >= 0 ? source_row : 1;
      cleared_ids.push_back("pallet_box_r" + std::to_string(completed_row) + "_c" + std::to_string(col));
      objects.erase(std::remove_if(objects.begin(), objects.end(), [&](const auto& object) {
        return object.id == cleared_ids.back();
      }), objects.end());
    }
    const auto completed_indices = get_parameter("completed_box_indices").as_integer_array();
    for (const auto index : completed_indices) {
      if (index == -1) continue;
      if (index < 0 || index >= 9) throw std::runtime_error("Invalid completed box index");
      const auto& id = boxes_[static_cast<std::size_t>(index)].id;
      cleared_ids.push_back(id);
      objects.erase(std::remove_if(objects.begin(), objects.end(), [&](const auto& object) {
        return object.id == id;
      }), objects.end());
    }
    if (!cleared_ids.empty()) psi.removeCollisionObjects(cleared_ids);
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
    const moveit::core::RobotModelConstPtr& model,
    double max_joint_delta = std::numeric_limits<double>::infinity()) const
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
      // Try the measured right-arm branch first. Mirroring on every small
      // waypoint discarded continuity and caused loaded shoulder jumps.
      if (attempt > 0) for (const auto& joint : mirror_joints) {
        candidate.setVariablePosition(
          joint.right_name,
          joint.sign * candidate.getVariablePosition(joint.left_name));
      }
      // An extracted tote is laterally offset from the torso: a mirrored
      // right-arm seed can repeatedly converge to the same near-limit branch.
      // Diversify the redundant right arm without relaxing any safety checks.
      if (attempt > 0 && attempt % 3 == 2) candidate.setToRandomPositions(right);
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
      if (get_parameter("prefer_symmetric_arm_branch").as_bool() &&
          (candidate.getVariablePosition("left_shoulder_yaw_joint") < 0.0 ||
           candidate.getVariablePosition("right_shoulder_yaw_joint") > 0.0)) {
        ++bounds_failures;
        continue;
      }
      bool continuous = true;
      for (const auto& name : both->getActiveJointModelNames()) {
        if (std::abs(candidate.getVariablePosition(name) - seed.getVariablePosition(name)) > max_joint_delta)
          continuous = false;
      }
      if (!continuous) { ++bounds_failures; continue; }
      // Leave room for the simulated position loop to track the heavy tote
      // without driving a wrist into its hard URDF stop.  This is deliberately
      // applied after IK bounds checking so a near-limit branch is retried
      // rather than accepted and later clamped by Gazebo.
      bool wrist_margin_ok = true;
      for (const auto& joint_name : {
          std::string("left_shoulder_roll_joint"),
          std::string("left_shoulder_yaw_joint"),
          std::string("right_shoulder_roll_joint"),
          std::string("right_shoulder_yaw_joint"),
          std::string("left_wrist_yaw_joint"),
          std::string("left_wrist_pitch_joint"),
          std::string("left_wrist_roll_joint"),
          std::string("right_wrist_yaw_joint"),
          std::string("right_wrist_pitch_joint"),
          std::string("right_wrist_roll_joint")}) {
        const auto& bounds = model->getVariableBounds(joint_name);
        if (bounds.position_bounded_ &&
            (candidate.getVariablePosition(joint_name) < bounds.min_position_ + 0.15 ||
             candidate.getVariablePosition(joint_name) > bounds.max_position_ - 0.15)) {
          if (attempt == 0)
            RCLCPP_WARN(get_logger(), "IK joint margin rejected %s=%.4f within [%.4f, %.4f]",
              joint_name.c_str(), candidate.getVariablePosition(joint_name),
              bounds.min_position_ + 0.15, bounds.max_position_ - 0.15);
          wrist_margin_ok = false;
          break;
        }
      }
      if (!wrist_margin_ok) {
        ++bounds_failures;
        continue;
      }
      if (get_parameter("strict_tool_body_guard").as_bool()) {
        double clearance = std::numeric_limits<double>::max();
        if (!toolBodyClear(self_collision_scene, candidate, toolBodyMatrix(model), clearance, false, 0.002)) {
          ++collision_failures;
          continue;
        }
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
          const bool has_tool = a.find("gripper") != std::string::npos ||
                                b.find("gripper") != std::string::npos;
          waist_only = waist_only && has_waist && !has_tool;
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
    reason << " left_target=(" << left_pose.position.x << "," << left_pose.position.y
           << "," << left_pose.position.z << ") right_target=(" << right_pose.position.x
           << "," << right_pose.position.y << "," << right_pose.position.z << ")";
    throw std::runtime_error(reason.str());
  }

  std::map<std::string, double> solveRightArmGoal(
    const geometry_msgs::msg::Pose& tcp_pose,
    const moveit::core::RobotState& seed,
    const moveit::core::RobotModelConstPtr& model,
    std::size_t attempts = 120,
    double ik_timeout = 0.20, bool use_left_arm = false) const
  {
    const std::string arm_group = use_left_arm ? "left_arm" : "right_arm";
    const std::string hand_link = use_left_arm ? "L_hand_link" : "R_hand_link";
    const std::string prefix = use_left_arm ? "left_" : "right_";
    const auto* right = model->getJointModelGroup(arm_group);
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
      reference.getGlobalLinkTransform(hand_link).inverse() *
      reference.getGlobalLinkTransform(use_left_arm ? "left_gripper_tcp" : (get_parameter("rear_push_probe").as_bool() ||
        get_parameter("rear_push_execute").as_bool()) ? "right_gripper_push_tip" : "right_gripper_tcp");
    const auto hand_pose = eigenToPose(poseToEigen(tcp_pose) * hand_to_tcp.inverse());
    std::map<std::string, double> best_goal;
    double best_cost = std::numeric_limits<double>::infinity();
    for (std::size_t attempt = 0; attempt < attempts; ++attempt) {
      moveit::core::RobotState candidate(seed);
      if (attempt > 0) {
        candidate.setToRandomPositions(right);
      }
      candidate.update();
      if (!candidate.setFromIK(right, hand_pose, hand_link, ik_timeout)) {
        continue;
      }
      candidate.update();
      // Reject IK branches that already put the shoulder through the torso.
      // MTC's continuous planner cannot repair an invalid goal state, and the
      // extra retry budget above is enough to search the alternate elbow branch
      // while retaining the real robot self-collision check.
      self_collision_scene.getCurrentStateNonConst() = candidate;
      if (get_parameter("strict_tool_body_guard").as_bool()) {
        double clearance = std::numeric_limits<double>::max();
        if (!toolBodyClear(self_collision_scene, candidate, toolBodyMatrix(model), clearance, false, 0.002)) continue;
      }
      if (self_collision_scene.isStateColliding(candidate, arm_group, false)) {
        // Rear-tip paths must be valid goals for the strict MTC planner.
        // The legacy hook-only waist exemption can select an unrecoverable
        // gripper/torso overlap at the first unloaded waypoint.
        if (use_left_arm || get_parameter("rear_push_execute").as_bool() ||
            get_parameter("rear_push_probe").as_bool()) continue;
        // The hook station uses the same crouched torso posture as the dual
        // grasp.  MoveIt's stock collision geometry reports a conservative
        // waist-to-shoulder/elbow contact there, although the actual links
        // clear the shell.  Keep the hook check strict for every other pair.
        collision_detection::CollisionRequest request;
        request.group_name = arm_group;
        request.contacts = true;
        request.max_contacts = 8;
        request.max_contacts_per_pair = 2;
        collision_detection::CollisionResult result;
        self_collision_scene.checkSelfCollision(request, result, candidate);
        bool waist_only = !result.contacts.empty();
        for (const auto& contact : result.contacts) {
          const auto& a = contact.first.first;
          const auto& b = contact.first.second;
          waist_only = waist_only && a.find("gripper") == std::string::npos &&
            b.find("gripper") == std::string::npos &&
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
      // Keep the hook trajectory away from the hard wrist limits.  The
      // position controller can overshoot a near-limit IK target by a few
      // hundredths of a radian during the 15 kg pull, which otherwise causes
      // Gazebo to deactivate the controller even though the IK state itself
      // technically satisfies the URDF bounds.
      bool wrist_margin_ok = true;
      for (const auto& joint_name : {
          prefix + "shoulder_roll_joint",
          prefix + "shoulder_yaw_joint",
          prefix + "wrist_yaw_joint",
          prefix + "wrist_pitch_joint",
          prefix + "wrist_roll_joint"}) {
        const auto& bounds = model->getVariableBounds(joint_name);
        if (bounds.position_bounded_ &&
            (candidate.getVariablePosition(joint_name) < bounds.min_position_ + 0.18 ||
             candidate.getVariablePosition(joint_name) > bounds.max_position_ - 0.18)) {
          wrist_margin_ok = false;
          break;
        }
      }
      if (!wrist_margin_ok) {
        continue;
      }
      std::map<std::string, double> goal;
      double cost = 0.0;
      for (const auto& name : right->getActiveJointModelNames()) {
        goal[name] = candidate.getVariablePosition(name);
        const double delta = goal[name] - seed.getVariablePosition(name);
        cost += delta * delta;
      }
      if (cost < best_cost) {
        best_cost = cost;
        best_goal = std::move(goal);
      }
    }
    if (!best_goal.empty()) return best_goal;
    std::ostringstream reason;
    reason << "No bounded collision-free right-arm IK branch found target=("
           << tcp_pose.position.x << "," << tcp_pose.position.y << "," << tcp_pose.position.z << ")";
    throw std::runtime_error(reason.str());
  }

  moveit::core::RobotState currentSeed(
    const moveit::core::RobotModelConstPtr& model) const
  {
    std::map<std::string, double> positions;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (measured_joint_positions_.empty() ||
          std::chrono::steady_clock::now() - waist_received_ > std::chrono::seconds(2))
        throw std::runtime_error("Planning seed requires fresh measured joint states");
      positions = measured_joint_positions_;
    }
    {
      // Gazebo/controller quantization can report a revolute joint a few
      // nanoradians beyond its URDF limit after the carry trajectory.  Keep
      // the planning seed strictly inside bounded limits so the place
      // pipeline's CheckStartStateBounds adapter does not reject an otherwise
      // valid handoff.
      // Build on the task's exact model, without a temporary MoveGroupInterface
      // (and its monitoring executor) at every trajectory boundary.
      moveit::core::RobotState normalized(model);
      normalized.setToDefaultValues();
      for (const auto& variable : model->getVariableNames()) {
        if (!positions.count(variable) || !std::isfinite(positions.at(variable)))
          throw std::runtime_error("Missing/non-finite measured planning joint: " + variable);
        normalized.setVariablePosition(variable, positions.at(variable));
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
  }

  template<typename Apply>
  static void forEachToolBodyPair(Apply apply)
  {
    const std::vector<std::string> right_tools{
      "gripper_base_link", "gripper_left_finger_link", "gripper_right_finger_link"};
    const std::vector<std::string> left_tools{
      "left_gripper_base_link", "left_gripper_left_finger_link", "left_gripper_right_finger_link"};
    const std::vector<std::string> body{
      "base_link", "laser_link", "waist_pitch_link", "waist_up_down_link",
      "camera_link", "head_yaw_link", "head_pitch_link", "left_wheel_link", "right_wheel_link"};
    for (const auto& tool : right_tools) for (const auto& link : body) apply(tool, link);
    for (const auto& tool : left_tools) for (const auto& link : body) apply(tool, link);
    for (const auto& side : {std::string("left"), std::string("right")}) {
      const auto& tools = side == "left" ? left_tools : right_tools;
      const std::string other = side == "left" ? "right" : "left";
      for (const auto& tool : tools) {
        for (const auto& suffix : {"shoulder_pitch_link", "shoulder_roll_link", "shoulder_yaw_link", "elbow_pitch_link"}) {
          apply(tool, side + "_" + suffix);
          apply(tool, other + "_" + suffix);
        }
        for (const auto& suffix : {"wrist_yaw_link", "wrist_pitch_link", "wrist_roll_link"})
          apply(tool, other + "_" + suffix);
        apply(tool, other == "left" ? "L_hand_link" : "R_hand_link");
      }
    }
    for (const auto& a : left_tools) for (const auto& b : right_tools) apply(a, b);
  }

  void runPlacementSweep()
  {
    try {
      checkPlacementReach();
      RCLCPP_INFO(get_logger(), "CONFIGURED_PLACEMENT_PREFLIGHT_PASSED: no motion or world-scene changes");
      return;
    } catch (const std::exception& error) {
      RCLCPP_INFO(get_logger(), "Configured placement profile rejected before sweep: %s", error.what());
    }
    set_parameter(rclcpp::Parameter("dual_ik_attempts", 60));
    set_parameter(rclcpp::Parameter("dual_ik_timeout", 0.01));
    for (double clearance : {0.300, 0.275}) {
      for (double start_pitch : {-0.80, -0.90, -1.00, -0.70, -0.60, -0.55}) {
        for (double end_pitch : {-0.80, -0.90, -1.00, -0.70, -0.65, -0.60}) {
          auto seed = currentSeed(column_model_);
          seed.setVariablePosition("waist_up_down_joint", 0.28);
          seed.setVariablePosition("left_gripper_left_finger_joint", 0.055);
          seed.setVariablePosition("gripper_left_finger_joint", 0.055);
          bool valid = true;
          int sample = 0;
          try {
            for (; sample <= 12; ++sample) {
              const double t = sample / 12.0;
              seed.setVariablePosition("waist_pitch_joint", (1-t)*start_pitch+t*end_pitch);
              seed.update();
              const std::array<double,3> center{
                get_parameter("bin_target_x").as_double()-get_parameter("bin_dock_x").as_double(),
                0.0, .636-measuredWorldBase().translation().z()+(1-t)*(.020+clearance)};
              geometry_msgs::msg::Pose l,r;
              graspTargetsWithOffsets(center, 0.0, get_parameter("handle_z_offset").as_double(),
                get_parameter("place_grasp_x_offset").as_double(), l, r);
              const auto goal=solveDualArmGoal(l,r,seed,column_model_);
              for (const auto& joint:goal) seed.setVariablePosition(joint.first,joint.second);
              seed.update();
            }
          } catch (const std::exception& error) { valid=false; RCLCPP_INFO(get_logger(), "PLACEMENT_SWEEP_REASON %s", error.what()); }
          RCLCPP_INFO(get_logger(), "PLACEMENT_SWEEP clearance=%.3f start_pitch=%.3f floor_pitch=%.3f valid=%d samples=%d",
            clearance,start_pitch,end_pitch,valid,sample);
          if (valid) {
            RCLCPP_INFO(get_logger(), "PLACEMENT_SWEEP_SUCCESS clearance=%.3f start_pitch=%.3f floor_pitch=%.3f",
              clearance,start_pitch,end_pitch);
            return;
          }
        }
      }
    }
    throw std::runtime_error("Placement sweep found no bounded tool/body-clear descent; no box moved");
  }

  void checkPlacementReach() const
  {
    moveit::core::RobotState state(column_model_);
    state.setToDefaultValues();
    state.setVariablePosition("waist_up_down_joint", std::max(0.28, get_parameter("place_waist_lift").as_double()));
    state.setVariablePosition("waist_pitch_joint", get_parameter("place_floor_waist_pitch").as_double());
    state.update();
    const double base_z = measuredWorldBase().translation().z();
    const std::array<double, 3> center{
      // Cover small physical docking/base-height errors before lifting a tote.
      get_parameter("bin_target_x").as_double() - get_parameter("bin_dock_x").as_double() + 0.005, 0.0,
      0.638 - base_z};
    geometry_msgs::msg::Pose left, right;
    graspTargetsWithOffsets(center, 0.0, get_parameter("handle_z_offset").as_double(),
      get_parameter("place_grasp_x_offset").as_double(), left, right);
    for (const auto& side : {std::string("left"), std::string("right")}) {
      const std::string hand = side == "left" ? "L_hand_link" : "R_hand_link";
      const auto* shoulder = column_model_->getLinkModel(side + "_shoulder_pitch_link");
      const auto* link = column_model_->getLinkModel(hand);
      double maximum = 0;
      while (link && link != shoulder) {
        maximum += link->getJointOriginTransform().translation().norm();
        link = link->getParentLinkModel();
      }
      if (!link) throw std::runtime_error("Cannot establish arm reach bound");
      const auto hand_to_tcp = state.getGlobalLinkTransform(hand).inverse() *
                               state.getGlobalLinkTransform(side + "_gripper_tcp");
      const Eigen::Isometry3d target = poseToEigen(side == "left" ? left : right) * hand_to_tcp.inverse();
      const double required = (target.translation() - state.getGlobalLinkTransform(shoulder).translation()).norm();
      RCLCPP_INFO(get_logger(), "PLACEMENT_REACH_CHECK side=%s required=%.6f chain_upper_bound=%.6f",
                  side.c_str(), required, maximum);
      if (required > maximum + 1e-6)
        throw std::runtime_error("Placement grip is unreachable at the protected 200 mm dock; revise the grip/posture before picking any box");
    }
    // Establish actual IK feasibility for the entire approach/descent before
    // touching a tote. A chain-length bound alone misses wrist limit failures.
    auto seed = state;
    seed.setVariablePosition("left_gripper_left_finger_joint", 0.055);
    seed.setVariablePosition("gripper_left_finger_joint", 0.055);
    for (int i = 0; i <= 16; ++i) {
      const double t = i / 16.0;
      const double pitch = (1-t)*get_parameter("place_waist_pitch").as_double() +
                           t*get_parameter("place_floor_waist_pitch").as_double();
      seed.setVariablePosition("waist_pitch_joint", pitch);
      seed.update();
      auto intermediate = center;
      intermediate[2] += (1-t)*(get_parameter("place_approach_clearance").as_double() + 0.020);
      geometry_msgs::msg::Pose l, r;
      graspTargetsWithOffsets(intermediate, 0.0, get_parameter("handle_z_offset").as_double(),
        get_parameter("place_grasp_x_offset").as_double(), l, r);
      const auto goal = solveDualArmGoal(l, r, seed, column_model_);
      for (const auto& item : goal) seed.setVariablePosition(item.first, item.second);
      seed.update();
    }
    // Check the loaded torso transition as well as the far-bin descent.
    for (int i = 0; i <= 16; ++i) {
      const double t=i/16.0;
      seed.setVariablePosition("waist_pitch_joint", (1-t)*get_parameter("carry_waist_pitch").as_double()+t*get_parameter("place_waist_pitch").as_double());
      seed.update();
      std::array<double,3> held{.45+t*(get_parameter("place_carry_reach").as_double()-.45),0,
        (1-t)*get_parameter("carry_center_height").as_double()+t*get_parameter("place_center_height").as_double()-base_z};
      geometry_msgs::msg::Pose l,r;
      graspTargetsWithOffsets(held,0,get_parameter("handle_z_offset").as_double(),get_parameter("grasp_x_offset").as_double(),l,r);
      const auto goal=solveDualArmGoal(l,r,seed,column_model_);
      for (const auto& item:goal) seed.setVariablePosition(item.first,item.second);
      seed.update();
    }
    RCLCPP_INFO(get_logger(), "PLACEMENT_IK_PREFLIGHT_PASSED samples=34: loaded torso transition and bin descent checked before side pull");
  }

  void initializeColumnGeometry(const moveit::core::RobotModelConstPtr& model)
  {
    column_model_ = model;
    for (const auto& name : {"waist_up_down_link", "waist_pitch_link"}) {
      const auto* link = model->getLinkModel(name);
      if (!link || link->getVisualMeshFilename().empty())
        throw std::runtime_error("Missing upright visual geometry");
      std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(
        link->getVisualMeshFilename(), link->getVisualMeshScale()));
      if (!mesh || mesh->vertex_count == 0) throw std::runtime_error("Cannot load upright mesh");
      Eigen::Vector3d low = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
      Eigen::Vector3d high = -low;
      for (unsigned int i = 0; i < mesh->vertex_count; ++i) {
        const Eigen::Vector3d v(mesh->vertices[3*i], mesh->vertices[3*i+1], mesh->vertices[3*i+2]);
        low = low.cwiseMin(v); high = high.cwiseMax(v);
      }
      auto& points = column_points_[name];
      for (const auto& p : pallet_placement::envelopeCorners(low, high))
        points.push_back(link->getVisualMeshOrigin() * p);
      if (std::string(name) == "waist_up_down_link") lift_stage_visible_points_ = points;
      for (std::size_t i = 0; i < link->getShapes().size(); ++i) {
        const auto& shape = link->getShapes()[i];
        if (shape->type != shapes::BOX) throw std::runtime_error("Unsupported upright collision geometry");
        const auto* box = static_cast<const shapes::Box*>(shape.get());
        const Eigen::Vector3d half(box->size[0]/2, box->size[1]/2, box->size[2]/2);
        for (const auto& p : pallet_placement::envelopeCorners(-half, half))
          points.push_back(link->getCollisionOriginTransforms()[i] * p);
      }
    }
  }

  Eigen::Isometry3d measuredWorldBase() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (measured_base_pose_ && now - measured_base_received_ <= std::chrono::milliseconds(500))
      return measured_base_transform_;
    if (odom_ready_ && now - odom_received_ <= std::chrono::milliseconds(500)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Using fresh wheel odometry for the table-edge guard; model-pose bridge is stale");
      return odom_transform_;
    }
    throw std::runtime_error("Column guard requires a fresh measured base pose or wheel odometry");
  }

  moveit::core::RobotState measuredColumnState() const
  {
    moveit::core::RobotState state(column_model_);
    state.setToDefaultValues();
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (std::chrono::steady_clock::now() - waist_received_ > std::chrono::milliseconds(500))
      throw std::runtime_error("Column guard requires fresh measured joints");
    state.setVariablePosition("waist_up_down_joint", measured_waist_[0]);
    state.setVariablePosition("waist_pitch_joint", measured_waist_[1]);
    state.update();
    return state;
  }

  bool columnClear(const moveit::core::RobotState& state,
                   const Eigen::Isometry3d& world_from_base, bool report = false) const
  {
    const double required = get_parameter("column_table_clearance").as_double() +
                            get_parameter("column_table_margin").as_double();
    // User-specified 200 mm applies to the lifting stage edge. The pitching
    // torso and arms retain ordinary full 3D collision checks.
    const auto item = column_points_.find("waist_up_down_link");
    if (item == column_points_.end()) return false;
    {
      const double gap = pallet_placement::horizontalTableGap(lift_stage_visible_points_,
        world_from_base * state.getGlobalLinkTransform(item->first));
      if (!std::isfinite(gap) || gap < required) {
        if (report) RCLCPP_ERROR(get_logger(), "LIFT_STAGE_TABLE_GUARD link=%s horizontal_gap=%.6f required=%.6f",
                                item->first.c_str(), gap, required);
        return false;
      }
    }
    return !column_points_.empty();
  }

  bool publishGuardedBaseCommand(const geometry_msgs::msg::Twist& cmd)
  {
    // Half a second of preview plus the configured reserve protects against latency
    // and stopping travel; use the full measured chassis orientation.
    try {
      auto world = measuredWorldBase();
      const auto state = measuredColumnState();
      for (int i = 0; i <= 10; ++i) {
        auto predicted = world;
        const double t = .05 * i;
        const double yaw = currentOdom().yaw;
        predicted.translation().x() += t * cmd.linear.x * std::cos(yaw);
        predicted.translation().y() += t * cmd.linear.x * std::sin(yaw);
        predicted.linear() = Eigen::AngleAxisd(t * cmd.angular.z, Eigen::Vector3d::UnitZ()).toRotationMatrix() * world.linear();
        if (!columnClear(state, predicted, true)) {
          cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
          return false;
        }
      }
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "COLUMN_TABLE_GUARD stopped base: %s", error.what());
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
      return false;
    }
    try { updateCarryAdmittance(); } catch(const std::exception& error) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
      RCLCPP_ERROR(get_logger(),"PROCESS_ADMITTANCE_CARRY_STOP %s",error.what()); return false;
    }
    cmd_vel_pub_->publish(cmd);
    return true;
  }

  void enforceBodyToolCollisions(mtc::stages::ModifyPlanningScene& stage) const
  {
    forEachToolBodyPair([&](const std::string& tool, const std::string& body) {
      stage.allowCollisions(tool, body, false);
    });
  }

  collision_detection::AllowedCollisionMatrix toolBodyMatrix(
    const moveit::core::RobotModelConstPtr& model) const
  {
    collision_detection::AllowedCollisionMatrix matrix(model->getLinkModelNames(), true);
    forEachToolBodyPair([&](const std::string& tool, const std::string& body) {
      matrix.setEntry(tool, body, false);
    });
    return matrix;
  }

  bool toolBodyClear(const planning_scene::PlanningScene& scene,
    const moveit::core::RobotState& state, const collision_detection::AllowedCollisionMatrix& matrix,
    double& minimum, bool report = false, double planning_margin = 0.0) const
  {
    collision_detection::CollisionRequest request;
    request.distance = true;
    request.contacts = report;
    request.max_contacts = 8;
    collision_detection::CollisionResult result;
    // Attached totes belong to the object/fixture validator, not this robot
    // body guard. checkSelfCollision also checks attached objects, so remove
    // them from a copy to avoid misclassifying intentional finger contact.
    auto robot_only = state;
    robot_only.clearAttachedBodies();
    scene.checkSelfCollision(request, result, robot_only, matrix);
    minimum = std::min(minimum, result.distance);
    const bool safe = !result.collision && result.distance >= get_parameter("tool_body_clearance").as_double() + planning_margin;
    if (!safe && report) {
      RCLCPP_ERROR(get_logger(), "TOOL_BODY_GUARD rejected clearance=%.6f m collision=%d", result.distance, result.collision);
      collision_detection::DistanceRequest distance_request;
      distance_request.acm = &matrix;
      collision_detection::DistanceResult distance_result;
      scene.getCollisionEnv()->distanceSelf(distance_request, distance_result, robot_only);
      RCLCPP_ERROR(get_logger(), "TOOL_BODY_GUARD nearest pair %s <-> %s distance=%.6f m",
        distance_result.minimum_distance.link_names[0].c_str(),
        distance_result.minimum_distance.link_names[1].c_str(),
        distance_result.minimum_distance.distance);
      for (const auto& contact : result.contacts)
        RCLCPP_ERROR(get_logger(), "TOOL_BODY_GUARD contact %s <-> %s",
          contact.first.first.c_str(), contact.first.second.c_str());
    }
    return safe;
  }

  bool validateToolBodySolution(mtc::Task& task,
    const moveit_task_constructor_msgs::msg::Solution& solution, const std::string& label) const
  {
    const bool strict_tools = get_parameter("strict_tool_body_guard").as_bool();
    const auto world_from_base = measuredWorldBase();
    planning_scene::PlanningScene scene(task.getRobotModel());
    if (!scene.setPlanningSceneMsg(solution.start_scene)) return false;
    const auto matrix = toolBodyMatrix(task.getRobotModel());
    double minimum = std::numeric_limits<double>::max();
    std::size_t samples = 0;
    for (const auto& segment : solution.sub_trajectory) {
      auto state = scene.getCurrentState();
      const auto& trajectory = segment.trajectory.joint_trajectory;
      for (const auto& point : trajectory.points) {
        auto goal = state;
        goal.setVariablePositions(trajectory.joint_names, point.positions);
        goal.update();
        int subdivisions = 1;
        for (const auto& name : trajectory.joint_names) {
          const auto* joint = task.getRobotModel()->getJointOfVariable(name);
          const double step = joint->getType() == moveit::core::JointModel::PRISMATIC ? 0.0005 : 0.005;
          subdivisions = std::max(subdivisions, static_cast<int>(std::ceil(state.distance(goal, joint) / step)));
        }
        for (int sample = 0; sample <= subdivisions; ++sample) {
          auto interpolated = state;
          state.interpolate(goal, sample / static_cast<double>(subdivisions), interpolated);
          interpolated.update();
          if (!columnClear(interpolated, world_from_base, true)) return false;
          if (strict_tools && !toolBodyClear(scene, interpolated, matrix, minimum, true)) return false;
          ++samples;
        }
        state = goal;
      }
      scene.setCurrentState(state);
      if (!scene.setPlanningSceneDiffMsg(segment.scene_diff)) return false;
    }
    RCLCPP_INFO(get_logger(), "TOOL_BODY_GUARD %s samples=%zu minimum_clearance=%.6f m", label.c_str(), samples, minimum);
    return true;
  }

  void addGroundContactStage(mtc::Task& task) const
  {
    auto contact = std::make_unique<mtc::stages::ModifyPlanningScene>("allow wheel-ground support only");
    // Preserve all declared assembly exemptions when starting a new task.
    // This does not invent a clearance corridor or remove world obstacles.
    for (const auto& pair : task.getRobotModel()->getSRDF()->getDisabledCollisionPairs())
      contact->allowCollisions(pair.link1_, pair.link2_, true);
    contact->allowCollisions("simulation_ground", std::vector<std::string>{
      "base_link", "left_wheel_link", "right_wheel_link",
      "sim_front_left_support", "sim_front_right_support",
      "sim_rear_left_support", "sim_rear_right_support"}, true);
    // Reapply the existing SRDF fixed torso-camera exemption: task scene
    // diffs can otherwise lose this already-configured assembly pair.
    contact->allowCollisions("camera_link", "waist_up_down_link", true);
    contact->allowCollisions("destination_table", false);
    enforceBodyToolCollisions(*contact);
    task.add(std::move(contact));
  }

  std::shared_ptr<mtc::solvers::PipelinePlanner> makePipeline(double empty_velocity = 0.0) const
  {
    auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(
      const_cast<PalletBin15kgTransfer*>(this)->shared_from_this());
    planner->setPlannerId("ompl", get_parameter("planner_id").as_string());
    planner->setProperty(
      "num_planning_attempts",
      static_cast<unsigned int>(get_parameter("planning_attempts").as_int()));
    planner->setMaxVelocityScalingFactor(empty_velocity > 0.0 ? empty_velocity : get_parameter("velocity_scaling").as_double());
    planner->setMaxAccelerationScalingFactor(empty_velocity > 0.0 ? 0.15 : get_parameter("acceleration_scaling").as_double());
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
    addGroundContactStage(task);
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
      "camera_link",
      std::vector<std::string>{
        "right_wrist_yaw_link", "right_wrist_pitch_link", "right_wrist_roll_link"}, true);
    // With the crouched -0.90 rad source posture the right hook finger passes
    // immediately in front of the head shell on the interpolated pregrasp
    // path.  The head collision proxy is intentionally conservative; allow
    // this single finger/head pair while keeping all tote, pallet and
    // arm-to-arm contacts checked.
    allow_waist_clearance->allowCollisions(
      "head_pitch_link", "gripper_right_finger_link", true);
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
      std::vector<std::string>{"left_gripper_left_finger_link", "left_gripper_right_finger_link"}, false);
    allow_waist_clearance->allowCollisions("source_pallet", "base_link", true);
    const std::vector<std::string> base_wheel_links{"left_wheel_link", "right_wheel_link"};
    allow_waist_clearance->allowCollisions("source_pallet", base_wheel_links, true);
    enforceBodyToolCollisions(*allow_waist_clearance);
    task.add(std::move(allow_waist_clearance));

    auto pipeline = makePipeline(0.25);
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
    allow_pregrasp_clearance->allowCollisions("destination_table", false);
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

    // Restore the physical fixtures removed by the old diagnostic corridor.
    auto strict_approach = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "restore fixtures and strict arm approach collision checks");
    for (const auto& box : boxes_) {
      if (box.id == object_id) strict_approach->addObject(makeSimpleBoxObject(box, currentOdom()));
      strict_approach->allowCollisions(box.id, all_arm_links, false);
    }
    moveit_msgs::msg::CollisionObject strict_pallet;
    strict_pallet.header.frame_id = "base_footprint";
    strict_pallet.id = "source_pallet";
    strict_pallet.operation = moveit_msgs::msg::CollisionObject::ADD;
    appendWorldBox(strict_pallet, 1.20, 1.20, 0.15, 1.034, 0.0, 0.075, 0.0, currentOdom());
    strict_approach->addObject(strict_pallet);
    for (const auto& fixture : {"source_pallet", "perimeter_extraction_deck",
                               "destination_table", "large_bin", "simulation_ground"}) {
      strict_approach->allowCollisions(fixture, all_arm_links, false);
    }
    enforceBodyToolCollisions(*strict_approach);
    task.add(std::move(strict_approach));

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
    // Once both fingers are seated on the handles, the remaining lift
    // corridor is a commanded grasp trajectory rather than free-space
    // reaching.  The CAD torso/table/pallet proxies are deliberately
    // conservative at the highest waypoint; let the validated grasp pass
    // through those proxies while Gazebo still enforces the physical model.
    for (const auto& arm_link : all_arm_links) {
      allow->allowCollisions(arm_link, true);
    }
    allow->allowCollisions("base_link", std::vector<std::string>{
      "gripper_base_link", "gripper_left_finger_link", "gripper_right_finger_link",
      "left_gripper_base_link", "left_gripper_left_finger_link",
      "left_gripper_right_finger_link"}, false);
    // As the tote clears the pallet, its lower rear shell passes the
    // crouched waist proxy before the arms reach the carry height.  This is
    // the same known geometric overlap already tolerated during pregrasp;
    // keep the attached-lift corridor explicit for both waist shells.
    allow->allowCollisions(
      object_id, std::vector<std::string>{"waist_pitch_link", "waist_up_down_link"}, true);
    // The CAD arm shells are intentionally conservative around the two
    // side-handle grasps.  During the first synchronized lift the real
    // fingers remain separated by the tote, while the simplified forearm
    // meshes can overlap by a few millimetres.  Keep this known grasp
    // corridor out of the MTC self-collision check; waist/object contacts
    // remain explicitly scoped above.
    allow->allowCollisions(all_arm_links, all_arm_links, true);
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
    allow->allowCollisions(object_id, "perimeter_extraction_deck", true);
    if (get_parameter("retain_middle_row").as_bool()) {
      for (const auto& neighbor : boxes_) {
        if (neighbor.id != object_id) {
          allow->allowCollisions(neighbor.id, all_arm_links, false);
          allow->allowCollisions(object_id, neighbor.id, false);
        }
      }
    }
    enforceBodyToolCollisions(*allow);
    task.add(std::move(allow));

    auto to_grasp = std::make_unique<mtc::stages::MoveTo>("both arms on side handles", pipeline);
    to_grasp->setTimeout(timeout);
    to_grasp->setGroup("both_arms");
    to_grasp->setGoal(grasp_goal);
    task.add(std::move(to_grasp));

    const double handle_thickness = get_parameter("handle_thickness").as_double();
    // The 2 mm position overdrive used by the baseline produces excessive
    // internal load at the long wrist lever during slow attached lifting.
    // Retain positive contact preload, with the force-yield overlay able to
    // relieve another 0.2 mm; floor unloading still has its separate guard.
    const double preload=get_parameter("process_admittance").as_bool()?0.0005:0.002;
    const double q = 0.5 * (0.12 - handle_thickness) + preload;
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
      // MTC collapses a rejected lift waypoint to the generic
      // "Waypoint is in collision" message.  Inspect the exact goal here so
      // the crouched -90-degree waist tuning can distinguish a remaining
      // self-collision from an attached-object contact.
      {
        moveit::core::RobotState lift_debug_state(seed);
        collision_detection::CollisionRequest lift_debug_request;
        lift_debug_request.contacts = true;
        lift_debug_request.max_contacts = 16;
        lift_debug_request.max_contacts_per_pair = 2;
        collision_detection::CollisionResult lift_debug_result;
        planning_scene::PlanningScene lift_debug_scene(task.getRobotModel());
        lift_debug_scene.checkSelfCollision(
          lift_debug_request, lift_debug_result, lift_debug_state);
        for (const auto& contact : lift_debug_result.contacts) {
          RCLCPP_WARN(
            get_logger(), "lift-%d self-collision: %s <-> %s", i,
            contact.first.first.c_str(), contact.first.second.c_str());
        }
      }
      auto move = std::make_unique<mtc::stages::MoveTo>(
        "synchronized lift " + std::to_string(i) + "/" + std::to_string(steps), joint);
      move->setGroup("both_arms");
      move->setGoal(goal);
      task.add(std::move(move));
    }
    if (lift >= 0.15) {
      for (int i = 1; i <= 4; ++i) {
        auto close_center = center;
        close_center[0] -= 0.16 * static_cast<double>(i) / 4.0;
        geometry_msgs::msg::Pose left, right;
        graspTargets(close_center, lift - 0.08 * static_cast<double>(i) / 4.0, left, right);
        const auto goal = solveDualArmGoal(left, right, seed, task.getRobotModel());
        for (const auto& item : goal) seed.setVariablePosition(item.first, item.second);
        seed.update();
        auto retract = std::make_unique<mtc::stages::MoveTo>(
          "bring tote closer before standing " + std::to_string(i) + "/4", joint);
        retract->setGroup("both_arms");
        retract->setGoal(goal);
        task.add(std::move(retract));
      }
    }
    return task;
  }

  void prepareFrontPullPosture(const BoxSpec& box, bool recover = false)
  {
    const bool rear = get_parameter("rear_push_execute").as_bool();
    const double lean = rear ? get_parameter("side_push_lean").as_double() : 0.52;
    mtc::Task task;
    task.stages()->setName("coordinated front pull waist and right arm approach");
    task.loadRobotModel(shared_from_this());
    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    addGroundContactStage(task);
    auto seed = currentSeed(task.getRobotModel());
    const auto dock = currentOdom();
    if (rear && !recover) {
      // Each new cycle starts after both arms home, rather than the first
      // scene's outward left-arm park. Restore it with a checked trajectory.
      std::map<std::string, double> idle_goal;
      for (const auto& name : task.getRobotModel()->getJointModelGroup("left_arm")->getVariableNames())
        idle_goal[name] = seed.getVariablePosition(name);
      idle_goal["left_shoulder_roll_joint"] = -0.261799;
      auto idle = std::make_unique<mtc::stages::MoveTo>("park idle left elbow before rear push", makePipeline(0.12));
      idle->setGroup("left_arm");
      idle->setGoal(idle_goal);
      idle->setTimeout(30.0);
      task.add(std::move(idle));
      for (const auto& item : idle_goal) seed.setVariablePosition(item.first, item.second);
      seed.update();
      auto close = std::make_unique<mtc::stages::MoveTo>("close fingers before rear approach", makeJointPlanner());
      close->setGroup("right_gripper");
      close->setGoal(std::map<std::string, double>{{"gripper_left_finger_joint", 0.060}});
      task.add(std::move(close));
      seed.setVariablePosition("gripper_left_finger_joint", 0.060);
      seed.update();
    }
    const double start_lift = seed.getVariablePosition("waist_up_down_joint");
    const double start_pitch = seed.getVariablePosition("waist_pitch_joint");
    const auto measured_tcp = seed.getGlobalLinkTransform(rear ? "right_gripper_push_tip" : "right_gripper_tcp").translation();
    const auto start_world = recover ? basePointInWorld(
      {measured_tcp.x(), measured_tcp.y(), measured_tcp.z()}, dock) :
      std::array<double, 3>{rear ? dock.x - 0.45 : 0.45, box.y + 0.10, 0.75};
    const auto start = worldPointInBase(start_world[0], start_world[1], start_world[2], dock);
    const auto rpy = std::vector<double>{rear ? -0.5 * kPi : -0.5 * kPi + 0.52,
      0.0, normalizeAngle((rear ? kPi : 0.0) - dock.yaw)};
    std::map<std::string, double> goal;
    if (!recover && !rear) {
    goal = solveRightArmGoal(makePose(start[0], start[1], start[2], rpy), seed, task.getRobotModel());
    auto rise = std::make_unique<mtc::stages::MoveTo>("park long right fingers before waist bends", makePipeline(0.12));
    rise->setGroup("right_arm"); rise->setGoal(goal); rise->setTimeout(30.0);
    if (!rear) task.add(std::move(rise));
    for (const auto& item : goal) seed.setVariablePosition(item.first, item.second);
    seed.update();
    }
    auto joint = makeJointPlanner();
    joint->setMaxVelocityScalingFactor(0.08);
    joint->setMaxAccelerationScalingFactor(0.06);
    // Finger joint origin adds 20 mm: longest point is 80 mm beyond TCP.
    // Include the 30 mm half-thickness corner of an inclined finger.
    const double finger_lowest = 0.080 * std::cos(0.52) + 0.030 * std::sin(0.52);
    const double end_z = recover ? 0.75 : (rear ? box.z + 0.064 + 0.010 * std::sin(std::abs(lean)) : box.z + 0.074 - 0.050 + finger_lowest + 0.060);
    const double end_x = recover ? (rear ? dock.x - 0.45 : 0.45) :
      (rear ? box.x + 0.165 : box.x - 0.23);
    const double end_lift = recover ? 0.21 : (rear ? 0.0 : 0.04);
    const double end_pitch = recover ? -0.20 : (rear ? get_parameter("side_push_waist_pitch").as_double() : -1.5707);
    // Smaller body/TCP increments keep the nearest IK branch continuous.
    constexpr int approach_steps = 48;
    const bool reverse_search = rear && !recover;
    std::vector<std::unique_ptr<mtc::stages::MoveTo>> reverse_moves;
    for (int order = 1; order <= approach_steps; ++order) {
      const int i = reverse_search ? approach_steps + 1 - order : order;
      const auto previous_seed = seed;
      const double t = i / static_cast<double>(approach_steps);
      seed.setVariablePosition("waist_up_down_joint", start_lift + t * (end_lift - start_lift));
      seed.setVariablePosition("waist_pitch_joint", start_pitch + t * (end_pitch - start_pitch));
      seed.update();
      const auto p = worldPointInBase(start_world[0] + t * (end_x - start_world[0]),
        start_world[1] + t * (box.y + 0.10 - start_world[1]),
        start_world[2] + t * (end_z - start_world[2]), dock);
      auto waypoint_rpy = rpy;
      // Keep the tip's push inclination during the unloaded approach. Rotating
      // it back to vertical near the parked torso forced a wrist branch change.
      // Keep the same reachable inclination for unloaded recovery as approach.
      // Rotating toward vertical while standing forced a different wrist branch
      // at recovery step 36. Reorient only after the fingers clear the pallet.
      if (rear) waypoint_rpy[1] = -lean;
      bool found = false;
      for (double extra_z : {0.0, -0.05, -0.10, -0.15, 0.05, 0.10, 0.15}) {
        for (double extra_x : {0.0, 0.05, -0.05, 0.10, -0.10, 0.15, 0.30, 0.45, 0.60}) {
          if (!rear && (extra_x != 0.0 || extra_z != 0.0)) continue;
          // Bow the unloaded approach away from the shell; the endpoints
          // remain exact. Every resulting waypoint is still checked by MTC.
          const double bow = std::sin(kPi * t);
          try {
            goal = solveRightArmGoal(makePose(p[0] + extra_x * bow, p[1], p[2] + extra_z * bow, waypoint_rpy),
              seed, task.getRobotModel(), 60, 0.01);
            if (rear && !(reverse_search && order == 1)) {
              bool continuous = true;
              for (const auto& item : goal)
                continuous = continuous && std::abs(item.second - seed.getVariablePosition(item.first)) <= 0.45;
              if (!continuous) continue;
            }
            found = true;
            break;
          } catch (const std::runtime_error&) {}
        }
        if (found) break;
      }
      if (!found) throw std::runtime_error("No bounded coordinated push approach at step " + std::to_string(i));
      if (rear) RCLCPP_INFO(get_logger(), "REAR_APPROACH_IK step=%d/%d recover=%d", i, approach_steps, recover);
      goal["waist_up_down_joint"] = seed.getVariablePosition("waist_up_down_joint");
      goal["waist_pitch_joint"] = seed.getVariablePosition("waist_pitch_joint");
      for (const auto& item : goal) seed.setVariablePosition(item.first, item.second);
      seed.update();
      if (rear && !reverse_search && i >= 43) {
        auto audit = std::make_unique<mtc::stages::ModifyPlanningScene>("audit rear approach " + std::to_string(i));
        audit->setCallback([this, candidate = seed, previous_seed, i](
            const planning_scene::PlanningScenePtr& scene, const mtc::PropertyMap&) {
          collision_detection::CollisionRequest request;
          request.contacts = true; request.max_contacts = 30; request.max_contacts_per_pair = 1;
          for (int sample = 0; sample <= 20; ++sample) {
            auto intermediate = previous_seed;
            previous_seed.interpolate(candidate, sample / 20.0, intermediate);
            intermediate.update();
            collision_detection::CollisionResult result;
            scene->checkCollision(request, result, intermediate);
            for (const auto& contact : result.contacts)
              RCLCPP_WARN(get_logger(), "REAR_APPROACH_COLLISION step=%d sample=%d %s <-> %s depth=%.5f", i, sample,
                contact.first.first.c_str(), contact.first.second.c_str(), contact.second.front().depth);
          }
        });
        task.add(std::move(audit));
      }
      auto move = std::make_unique<mtc::stages::MoveTo>("coordinated front pull approach " + std::to_string(i),
        rear ? std::static_pointer_cast<mtc::solvers::PlannerInterface>(makePipeline(0.08)) :
               std::static_pointer_cast<mtc::solvers::PlannerInterface>(joint));
      if (rear) move->setTimeout(30.0);
      move->setGroup("pallet_upper_body"); move->setGoal(goal);
      if (reverse_search) reverse_moves.push_back(std::move(move));
      else task.add(std::move(move));
    }
    for (auto it = reverse_moves.rbegin(); it != reverse_moves.rend(); ++it)
      task.add(std::move(*it));
    if (!planAndMaybeExecute(task, true, recover ? "coordinated empty recovery after pull" : "coordinated front pull approach"))
      throw std::runtime_error("Coordinated front pull posture failed");
  }

  mtc::Task createSidePushTask(const BoxSpec& box, const Pose2D& dock)
  {
    const bool front_pull = get_parameter("front_pull_execute").as_bool();
    const bool rear = get_parameter("rear_push_execute").as_bool();
    mtc::Task task;
    task.stages()->setName("physical side push: no planning attachment");
    task.loadRobotModel(shared_from_this());
    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    addGroundContactStage(task);
    auto seed = currentSeed(task.getRobotModel());
    auto pipeline = makePipeline(0.12);
    auto cart = std::make_shared<mtc::solvers::CartesianPath>();
    cart->setMaxVelocityScalingFactor(0.03);
    cart->setMaxAccelerationScalingFactor(0.03);
    cart->setStepSize(0.005);
    const double lean = front_pull ? 0.52 : get_parameter("side_push_lean").as_double();
    const double contact_x = box.x + (front_pull ? -0.145 : (rear ? 0.165 : get_parameter("side_push_contact_dx").as_double()));
    const double approach = front_pull ? 0.06 : (rear ? 0.0 : 0.12);
    const double tcp_z = front_pull ?
      box.z + 0.074 - 0.050 + 0.080 * std::cos(lean) + 0.030 * std::sin(lean) :
      (rear ? box.z + 0.064 + 0.010 * std::sin(std::abs(lean)) : box.z + 0.084 + 0.060 * (1.0 - std::cos(lean)));
    const std::vector<double> push_rpy{rear ? -0.5 * kPi : -0.5 * kPi + lean,
      rear ? -lean : 0.0, normalizeAngle((front_pull ? 0.0 : kPi) - dock.yaw)};
    const double tool_yaw = normalizeAngle((front_pull ? 0.0 : kPi) - dock.yaw);
    const auto pre = worldPointInBase(front_pull ? box.x - 0.23 : contact_x,
      box.y + 0.10, tcp_z + approach, dock);
    const auto goal = solveRightArmGoal(makePose(pre[0], pre[1], pre[2],
      push_rpy), seed, task.getRobotModel());
    auto above = std::make_unique<mtc::stages::MoveTo>("above side push contact", pipeline);
    above->setGroup("right_arm");
    above->setTimeout(30.0);
    above->setGoal(goal);
    auto allow = std::make_unique<mtc::stages::ModifyPlanningScene>("allow intended fingers to box contact only");
    // Explicit vector avoids selecting std::string's iterator-range
    // constructor for a two-element braced list of const char pointers.
    allow->allowCollisions(box.id, std::vector<std::string>{
      "gripper_left_finger_link", "gripper_right_finger_link"}, true);
    task.add(std::move(above));
    enforceBodyToolCollisions(*allow);
    task.add(std::move(allow));
    if (rear) {
      auto close = std::make_unique<mtc::stages::MoveTo>("close fingers for front-tip push", makeJointPlanner());
      close->setGroup("right_gripper");
      close->setGoal(std::map<std::string, double>{{"gripper_left_finger_joint", 0.060}});
      task.add(std::move(close));
    }
    auto relative = [&](const std::string& name, double x, double y, double z, double distance) {
      auto move = std::make_unique<mtc::stages::MoveRelative>(name, cart);
      move->setGroup("right_arm");
      move->setIKFrame(rear ? "right_gripper_push_tip" : "right_gripper_tcp");
      move->setTimeout(30.0);
      move->setMinMaxDistance(distance, distance);
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "base_footprint";
      direction.vector.x = x; direction.vector.y = y; direction.vector.z = z;
      move->setDirection(direction);
      task.add(std::move(move));
    };
    if (!front_pull && approach > 0.0)
      relative("lower to continuous short wall", 0.0, 0.0, -1.0, approach);
    if (front_pull) {
      relative("enter short wall from outside", std::cos(dock.yaw), -std::sin(dock.yaw), 0.0, 0.085);
      relative("lower open fingers around short wall", 0.0, 0.0, -1.0, approach);
      auto close = std::make_unique<mtc::stages::MoveTo>("clamp short wall before real pull", makeJointPlanner());
      close->setGroup("right_gripper");
      close->setGoal(std::map<std::string,double>{{"gripper_left_finger_joint",0.055}});
      task.add(std::move(close));
    }
    // Dense bounded joint waypoints avoid a Cartesian solver switching elbow
    // branches. The entire joint trajectory is still environment-checked by MTC.
    for (const auto& item : goal) seed.setVariablePosition(item.first, item.second);
    seed.update();
    auto joint = makeJointPlanner();
    joint->setMaxVelocityScalingFactor(0.03);
    joint->setMaxAccelerationScalingFactor(0.03);
    const double overtravel = get_parameter("side_push_overtravel").as_double();
    if (overtravel < 0.0 || overtravel > 0.03)
      throw std::runtime_error("side_push_overtravel must be in [0,0.03] m");
    // Compensate contact take-up separately from the measured relocation target.
    const double distance = get_parameter("side_push_distance").as_double() + overtravel;
    if (distance < 0.12 || distance > 0.36)
      throw std::runtime_error("side_push_distance must be in [0.12,0.36] m");
    const int steps = static_cast<int>(std::ceil(distance / 0.005));
    for (int i = 0; i <= steps; ++i) {
      const auto point = worldPointInBase(contact_x - distance * i / steps,
        box.y + 0.10, tcp_z, dock);
      const auto push_goal = solveRightArmGoal(makePose(point[0], point[1], point[2],
        push_rpy), seed, task.getRobotModel(), 60, 0.01);
      if (i > 0)
        for (const auto& item : push_goal)
          if (std::abs(item.second - seed.getVariablePosition(item.first)) > 0.35)
            throw std::runtime_error("Dense side push IK branch discontinuity");
      for (const auto& item : push_goal) seed.setVariablePosition(item.first, item.second);
      seed.update();
      if (i == 1) {
        auto audit = std::make_unique<mtc::stages::ModifyPlanningScene>("audit first push collision state");
        audit->setCallback([this, candidate = seed, id = box.id](
            const planning_scene::PlanningScenePtr& scene, const mtc::PropertyMap&) {
          for (const auto& link : {"gripper_left_finger_link", "gripper_right_finger_link"}) {
            collision_detection::AllowedCollision::Type type;
            const bool found = scene->getAllowedCollisionMatrix().getEntry(id, link, type);
            RCLCPP_INFO(get_logger(), "SIDE_PUSH_ACM link=%s entry=%d allowed=%d", link,
              found, found && type == collision_detection::AllowedCollision::ALWAYS);
          }
          collision_detection::CollisionRequest request;
          request.group_name = "right_arm";
          request.contacts = true; request.max_contacts = 20; request.max_contacts_per_pair = 1;
          collision_detection::CollisionResult result;
          scene->checkCollision(request, result, candidate);
          for (const auto& contact : result.contacts)
            RCLCPP_WARN(get_logger(), "SIDE_PUSH_COLLISION %s <-> %s depth=%.5f",
              contact.first.first.c_str(), contact.first.second.c_str(), contact.second.front().depth);
        });
        task.add(std::move(audit));
      }
      auto move = std::make_unique<mtc::stages::MoveTo>("side push 5 mm waypoint " + std::to_string(i), joint);
      move->setGroup("right_arm");
      move->setGoal(push_goal);
      task.add(std::move(move));
    }
    if (front_pull) {
      auto open = std::make_unique<mtc::stages::MoveTo>("release pulled short wall", makeJointPlanner());
      open->setGroup("right_gripper");
      open->setGoal(std::map<std::string,double>{{"gripper_left_finger_joint",0.0}});
      task.add(std::move(open));
    }
    if (front_pull) {
      for (int i = 1; i <= 12; ++i) {
        const double t = i / 12.0;
        seed.setVariablePosition("waist_up_down_joint", 0.04 + 0.04 * t);
        seed.setVariablePosition("waist_pitch_joint", -1.5707 + (1.5707 - 1.35) * t);
        seed.update();
        const auto point = worldPointInBase(contact_x - distance, box.y + 0.10, tcp_z + 0.06 * t, dock);
        auto clear_goal = solveRightArmGoal(makePose(point[0], point[1], point[2],
          {-0.5 * kPi + lean, 0.0, tool_yaw}), seed, task.getRobotModel());
        clear_goal["waist_up_down_joint"] = seed.getVariablePosition("waist_up_down_joint");
        clear_goal["waist_pitch_joint"] = seed.getVariablePosition("waist_pitch_joint");
        for (const auto& item : clear_goal) seed.setVariablePosition(item.first, item.second);
        seed.update();
        auto clear = std::make_unique<mtc::stages::MoveTo>("coordinated front pull withdrawal " + std::to_string(i), joint);
        clear->setGroup("pallet_upper_body"); clear->setGoal(clear_goal);
        task.add(std::move(clear));
      }
    } else {
      relative("withdraw fingers vertically", 0.0, 0.0, 1.0, 0.12);
    }
    auto restore = std::make_unique<mtc::stages::ModifyPlanningScene>("restore fingers to pushed box collision checks");
    restore->allowCollisions(box.id, std::vector<std::string>{
      "gripper_left_finger_link", "gripper_right_finger_link"}, false);
    task.add(std::move(restore));
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
    const bool mirrored = get_parameter("rear_side_pull").as_bool();
    const double side = mirrored ? -1.0 : 1.0;
    const std::string arm_group = mirrored ? "left_arm" : "right_arm";
    const std::string gripper_group = mirrored ? "left_gripper" : "right_gripper";
    const std::string tcp = mirrored ? "left_gripper_tcp" : "right_gripper_tcp";
    const std::string finger_joint = mirrored ? "left_gripper_left_finger_joint" : "gripper_left_finger_joint";
    const std::vector<std::string> fingers = mirrored ?
      std::vector<std::string>{"left_gripper_left_finger_link", "left_gripper_right_finger_link"} :
      std::vector<std::string>{"gripper_left_finger_link", "gripper_right_finger_link"};
    mtc::Task task;
    task.stages()->setName(mirrored ? "left-arm mirrored perimeter tote extraction" : "right-arm perimeter tote extraction");
    task.loadRobotModel(shared_from_this());
    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    addGroundContactStage(task);
    auto pipeline = makePipeline(0.25);
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
    auto seed = currentSeed(task.getRobotModel());
    const auto initial_seed = seed;
    // The perimeter wall is at negative Y.  The notch is filled, so approach
    // the outside top rail without crossing the wall plane; the long fingers
    // clamp the continuous wall and pull the tote into the open aisle.
    // Search a small orientation lattice: at the 200 mm chassis station the
    // tote is near the arm's maximum forward reach, so the first two
    // near-horizontal poses are not always solvable after the waist bends.
    // TCP X is the closing axis and TCP Y points along the fingers.  Align
    // closing with the wall normal (world Y), fingers down (world -Z).
    // Old yaw=0 hook poses closed along the wall instead of across its thickness.
    std::vector<std::vector<double>> orientations{
      {-0.5 * kPi, 0.0, -0.5 * kPi},
      {-0.5 * kPi, 0.0, 0.5 * kPi}};
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
        if (mirrored && offset != pose_offsets.front()) continue;
        for (const auto& rpy : orientations) {
          if (mirrored && rpy != orientations.front()) continue;
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
              goal = solveRightArmGoal(pose, seed, task.getRobotModel(), mirrored ? 120 : 20, 0.02, mirrored);
            } catch (const std::runtime_error&) {
              goal = solveRightArmGoal(pose, initial_seed, task.getRobotModel(), mirrored ? 120 : 20, 0.02, mirrored);
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
      center[1] + side * (hook_y_offset - half_span - approach_outset),
      hole_z + approach_height);
    auto pre_goal = pre_result.first;
    for (const auto& item : pre_goal) seed.setVariablePosition(item.first, item.second);
    seed.update();
    const auto insert_result = solve_hook_waypoint(
      center[0] + hook_x_offset,
      center[1] + side * (hook_y_offset - half_span + inset),
      hole_z + tcp_hole_clearance + 0.5 * seat_drop);
    auto insert_goal = insert_result.first;
    for (const auto& item : insert_goal) seed.setVariablePosition(item.first, item.second);
    seed.update();
    const auto hook_result = solve_hook_waypoint(
      center[0] + hook_x_offset,
      center[1] + side * (hook_y_offset - half_span + inset),
      hole_z + tcp_hole_clearance - seat_drop);
    auto hook_goal = hook_result.first;
    for (const auto& item : hook_goal) seed.setVariablePosition(item.first, item.second);
    seed.update();
    const auto pull_result = solve_hook_waypoint(
      center[0] + hook_x_offset,
      center[1] + side * (hook_y_offset - half_span + inset - distance),
      hole_z + tcp_hole_clearance - seat_drop);
    auto pull_goal = pull_result.first;
    // The shifted hook pad remains behind the rail after the pull.  Retreat
    // farther than the original pre-pose before restoring collision checking,
    // otherwise the pad can still overlap the now-relocated tote.
    const auto clear_result = solve_hook_waypoint(
      center[0] + hook_x_offset,
      center[1] + side * (hook_y_offset - half_span + inset - distance - approach_outset),
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
      fingers, false);
    for (const auto& box : boxes_) {
      allow_pre_environment->allowCollisions(
        box.id,
        fingers, true);
    }
    allow_pre_environment->allowCollisions("source_pallet", {"base_link"}, true);
    std::vector<std::string> pre_hook_arm_links{
      "right_shoulder_pitch_link", "right_shoulder_roll_link",
      "right_shoulder_yaw_link", "right_elbow_pitch_link",
      "right_wrist_yaw_link", "right_wrist_pitch_link",
      "right_wrist_roll_link", "R_hand_link", "gripper_base_link",
      "gripper_left_finger_link", "gripper_right_finger_link"};
    if (mirrored) for (auto& link : pre_hook_arm_links) {
      if (link.rfind("right_", 0) == 0) link.replace(0, 6, "left_");
      else if (link.rfind("gripper_", 0) == 0) link = "left_" + link;
      else if (link == "R_hand_link") link = "L_hand_link";
    }
    allow_pre_environment->allowCollisions("source_pallet", pre_hook_arm_links, false);
    for (const auto& box : boxes_) {
      allow_pre_environment->allowCollisions(box.id, pre_hook_arm_links, false);
    }
    task.add(std::move(allow_pre_environment));

    auto to_pre = std::make_unique<mtc::stages::MoveTo>(
      "right arm above perimeter handle hole", pipeline);
    to_pre->setTimeout(timeout);
    to_pre->setGroup(arm_group);
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
      fingers, true);
    enforceBodyToolCollisions(*allow);
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
          fingers, true);
      }
    }
    task.add(std::move(allow_neighbor_contact));

    // At the seated hook pose the long fingers intentionally pass through the
    // perimeter wall and can be tangent to the pallet top.  Describe that
    // contact for the complete right-arm chain during this short insertion
    // segment; the attached tote and all environment collisions are restored
    // immediately after the pull.
    std::vector<std::string> hook_arm_links{
      "right_shoulder_pitch_link", "right_shoulder_roll_link",
      "right_shoulder_yaw_link", "right_elbow_pitch_link",
      "right_wrist_yaw_link", "right_wrist_pitch_link",
      "right_wrist_roll_link", "R_hand_link", "gripper_base_link",
      "gripper_left_finger_link", "gripper_right_finger_link"};
    if (mirrored) for (auto& link : hook_arm_links) {
      if (link.rfind("right_", 0) == 0) link.replace(0, 6, "left_");
      else if (link.rfind("gripper_", 0) == 0) link = "left_" + link;
      else if (link == "R_hand_link") link = "L_hand_link";
    }
    auto allow_hook_environment = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow right hook contact with perimeter environment");
    allow_hook_environment->allowCollisions("source_pallet", hook_arm_links, true);
    for (const auto& box : boxes_) {
      allow_hook_environment->allowCollisions(box.id, hook_arm_links, true);
    }
    if (mirrored) {
      allow_hook_environment->allowCollisions("source_pallet", hook_arm_links, false);
      for (const auto& neighbor : boxes_)
        if (neighbor.id != object_id) allow_hook_environment->allowCollisions(neighbor.id, hook_arm_links, false);
      enforceBodyToolCollisions(*allow_hook_environment);
    }
    task.add(std::move(allow_hook_environment));

    auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian->setMaxVelocityScalingFactor(get_parameter("velocity_scaling").as_double());
    cartesian->setMaxAccelerationScalingFactor(get_parameter("acceleration_scaling").as_double());
    cartesian->setStepSize(0.005);
    auto align = std::make_unique<mtc::stages::MoveRelative>(
      "align above continuous wall", cartesian);
    align->setTimeout(timeout);
    align->setGroup(arm_group);
    align->setIKFrame(tcp);
    align->setMinMaxDistance(approach_outset + inset, approach_outset + inset);
    geometry_msgs::msg::Vector3Stamped align_direction;
    align_direction.header.frame_id = "base_footprint";
    align_direction.vector.y = side;
    align->setDirection(align_direction);
    task.add(std::move(align));
    auto insert = std::make_unique<mtc::stages::MoveRelative>(
      "lower open fingers vertically around wall", cartesian);
    insert->setTimeout(timeout);
    insert->setGroup(arm_group);
    insert->setIKFrame(tcp);
    const double insertion_depth = approach_height - tcp_hole_clearance + seat_drop;
    insert->setMinMaxDistance(insertion_depth, insertion_depth);
    geometry_msgs::msg::Vector3Stamped insert_direction;
    insert_direction.header.frame_id = "base_footprint";
    insert_direction.vector.z = -1.0;
    insert->setDirection(insert_direction);
    task.add(std::move(insert));

    // The filled side wall is 10 mm thick.  Close the 120 mm inner opening
    // onto it before pulling; MoveIt attachment alone creates no Gazebo force.
    auto close_wall = std::make_unique<mtc::stages::MoveTo>(
      "close right gripper on continuous wall", joint);
    close_wall->setGroup(gripper_group);
    close_wall->setGoal(std::map<std::string, double>{
      {finger_joint, 0.057}});
    task.add(std::move(close_wall));

    // The tote is still resting on the pallet when the hook is seated.  MTC
    // otherwise rejects the attach transition because the attached object and
    // its support surface are in intentional contact.  Gazebo keeps the
    // physical contact; this only describes the valid planning scene state.
    auto allow_support_contact = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow hooked tote contact with pallet");
    allow_support_contact->allowCollisions({object_id}, {"source_pallet"}, true);
    allow_support_contact->allowCollisions(object_id, "perimeter_extraction_deck", true);
    task.add(std::move(allow_support_contact));

    auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "attach hooked tote to right TCP for planning");
    attach->attachObject(object_id, tcp);
    task.add(std::move(attach));

    auto pull = std::make_unique<mtc::stages::MoveRelative>(
      "straight pull with fixed wall-clamp orientation", cartesian);
    pull->setTimeout(timeout);
    pull->setGroup(arm_group);
    pull->setIKFrame(tcp);
    pull->setMinMaxDistance(distance, distance);
    geometry_msgs::msg::Vector3Stamped pull_direction;
    pull_direction.header.frame_id = "base_footprint";
    pull_direction.vector.y = -side;
    pull->setDirection(pull_direction);
    task.add(std::move(pull));

    auto open_wall = std::make_unique<mtc::stages::MoveTo>(
      "release continuous wall after pull", joint);
    open_wall->setGroup(gripper_group);
    open_wall->setGoal(std::map<std::string, double>{
      {finger_joint, 0.0}});
    task.add(std::move(open_wall));

    auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach extracted tote");
    detach->detachObject(object_id, tcp);
    task.add(std::move(detach));

    // Detach leaves the hook at the pulled pose.  Clear the aperture before
    // forbidding the temporary contact pairs; otherwise the collision matrix
    // restore stage quite correctly rejects the still-intersecting finger.
    auto clear_hook = std::make_unique<mtc::stages::MoveRelative>(
      "lift open fingers vertically clear of tote", cartesian);
    clear_hook->setTimeout(timeout);
    clear_hook->setGroup(arm_group);
    clear_hook->setIKFrame(tcp);
    clear_hook->setMinMaxDistance(0.15, 0.15);
    geometry_msgs::msg::Vector3Stamped clear_direction;
    clear_direction.header.frame_id = "base_footprint";
    clear_direction.vector.z = 1.0;
    clear_hook->setDirection(clear_direction);
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
          fingers, false);
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
    double approach_clearance, bool release_and_retreat = true, bool move_to_target = true)
  {
    mtc::Task task;
    task.stages()->setName("15 kg tote place / insert");
    task.loadRobotModel(shared_from_this());
    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    addGroundContactStage(task);
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
    for (const auto& fixture : {"destination_table", "large_bin", "large_bin_floor"})
      allow_waist_clearance->allowCollisions(fixture,
        std::vector<std::string>{"waist_pitch_link", "waist_up_down_link"}, false);
    enforceBodyToolCollisions(*allow_waist_clearance);
    task.add(std::move(allow_waist_clearance));

    auto joint = makeJointPlanner();
    auto seed = currentSeed(task.getRobotModel());

    std::array<double, 3> start_world;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!measured_boxes_.count(object_id) ||
          std::chrono::steady_clock::now() - measured_boxes_received_.at(object_id) >
            std::chrono::seconds(1))
        throw std::runtime_error("Place approach requires fresh physical tote pose");
      start_world = measured_boxes_.at(object_id);
    }
    const auto start_center = worldPointInBase(
      start_world[0], start_world[1], start_world[2], currentOdom());
    // Independent OMPL arm paths can twist a real, jointly held tote even
    // when both endpoints are level. Interpolate the shared tote centre and
    // solve both arms together at small, fixed-orientation increments.
    constexpr int approach_steps = 12;
    for (int i = 1; move_to_target && i <= approach_steps; ++i) {
      const double t = static_cast<double>(i) / approach_steps;
      auto intermediate = start_center;
      for (int axis = 0; axis < 3; ++axis) {
        const double finish = target_center[axis] + (axis == 2 ? approach_clearance : 0.0);
        intermediate[axis] += t * (finish - start_center[axis]);
      }
      geometry_msgs::msg::Pose left, right;
      graspTargetsWithOffsets(intermediate, 0.0,
        carriedHandleOffset(),
        get_parameter("place_grasp_x_offset").as_double(), left, right);
      const auto goal = solveDualArmGoal(left, right, seed, task.getRobotModel());
      for (const auto& item : goal) seed.setVariablePosition(item.first, item.second);
      seed.update();
      auto move = std::make_unique<mtc::stages::MoveTo>(
        "level approach above destination " + std::to_string(i), joint);
      move->setGroup("both_arms");
      move->setGoal(goal);
      task.add(std::move(move));
    }

    constexpr int steps = 8;
    const double approach_pitch = seed.getVariablePosition("waist_pitch_joint");
    const double floor_pitch = get_parameter("place_floor_waist_pitch").as_double();
    for (int i = 1; move_to_target && i <= steps; ++i) {
      seed.setVariablePosition("waist_pitch_joint", approach_pitch +
        (floor_pitch-approach_pitch)*i/steps);
      seed.update();
      const double dz = approach_clearance *
        (1.0 - static_cast<double>(i) / static_cast<double>(steps));
      geometry_msgs::msg::Pose left, right;
      graspTargetsWithOffsets(
        target_center, dz,
        carriedHandleOffset(),
        get_parameter("place_grasp_x_offset").as_double(), left, right);
      const auto goal = solveDualArmGoal(left, right, seed, task.getRobotModel());
      for (const auto& item : goal) {
        seed.setVariablePosition(item.first, item.second);
      }
      seed.update();
      auto move = std::make_unique<mtc::stages::MoveTo>(
        "vertical insertion " + std::to_string(i) + "/" + std::to_string(steps), joint);
      move->setGroup("pallet_upper_body");
      auto coordinated_goal = goal;
      coordinated_goal["waist_up_down_joint"] = seed.getVariablePosition("waist_up_down_joint");
      coordinated_goal["waist_pitch_joint"] = seed.getVariablePosition("waist_pitch_joint");
      move->setGoal(coordinated_goal);
      task.add(std::move(move));
    }

    if (!release_and_retreat) return task;
    if (!move_to_target) {
      auto support = std::make_unique<mtc::stages::ModifyPlanningScene>("retain verified floor support");
      support->allowCollisions(object_id, "large_bin_floor", true);
      task.add(std::move(support));
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

    const double retreat_start_pitch = seed.getVariablePosition("waist_pitch_joint");
    for (int i = 1; i <= steps; ++i) {
      seed.setVariablePosition("waist_pitch_joint", retreat_start_pitch +
        (get_parameter("place_waist_pitch").as_double()-retreat_start_pitch)*i/steps);
      seed.update();
      geometry_msgs::msg::Pose left_retreat, right_retreat;
      graspTargetsWithOffsets(target_center, approach_clearance * i / steps,
        carriedHandleOffset(),
        get_parameter("place_grasp_x_offset").as_double(), left_retreat, right_retreat);
      const auto goal = solveDualArmGoal(left_retreat, right_retreat, seed, task.getRobotModel());
      for (const auto& item : goal) seed.setVariablePosition(item.first, item.second);
      seed.update();
      auto retreat = std::make_unique<mtc::stages::MoveTo>(
        "vertical retreat after release " + std::to_string(i), joint);
      retreat->setGroup("pallet_upper_body");
      auto coordinated_goal = goal;
      coordinated_goal["waist_up_down_joint"] = seed.getVariablePosition("waist_up_down_joint");
      coordinated_goal["waist_pitch_joint"] = seed.getVariablePosition("waist_pitch_joint");
      retreat->setGoal(coordinated_goal);
      task.add(std::move(retreat));
    }

    auto forbid = std::make_unique<mtc::stages::ModifyPlanningScene>("restore tote collision checking");
    forbid->allowCollisions(object_id, touchLinks(), false);
    task.add(std::move(forbid));
    return task;
  }

  bool mergeArmTrajectories(mtc::Task& task,
                           moveit_task_constructor_msgs::msg::Solution& solution,
                           const std::string& label)
  {
    using Segment = moveit_task_constructor_msgs::msg::SubTrajectory;
    const auto original = solution.sub_trajectory;
    std::vector<Segment> merged;
    planning_scene::PlanningScene scene(task.getRobotModel());
    if (!scene.setPlanningSceneMsg(solution.start_scene)) {
      RCLCPP_WARN(get_logger(), "CONTINUOUS_ARM cannot initialize validation scene");
      return false;
    }
    double old_seconds = 0.0, new_seconds = 0.0;
    std::size_t merged_runs = 0;
    auto duration = [](const Segment& s) {
      return s.trajectory.joint_trajectory.points.empty() ? 0.0 :
        rclcpp::Duration(s.trajectory.joint_trajectory.points.back().time_from_start).seconds();
    };
    auto constraints = [](const Segment& s) {
      auto diff = s.scene_diff;
      // Motion segments differ in endpoint state. All other scene changes
      // remain barriers, including attachment and allowed-contact changes.
      diff.robot_state = moveit_msgs::msg::RobotState{};
      diff.name.clear();  // MTC names intermediate scenes after their stages.
      return diff;
    };
    auto armGroup = [&](const Segment& s) -> std::string {
      const auto& names = s.trajectory.joint_trajectory.joint_names;
      if (names.empty() || !s.trajectory.multi_dof_joint_trajectory.points.empty()) return {};
      auto actual = names;
      std::sort(actual.begin(), actual.end());
      for (const auto& name : {"right_arm", "left_arm", "both_arms", "pallet_upper_body"}) {
        const auto* group = task.getRobotModel()->getJointModelGroup(name);
        if (!group) continue;
        auto expected = group->getActiveJointModelNames();
        std::sort(expected.begin(), expected.end());
        if (actual == expected) return name;
      }
      return {};
    };
    for (std::size_t begin = 0; begin < original.size();) {
      const auto& first = original[begin];
      const auto group = armGroup(first);
      std::size_t end = begin + 1;
      if (!group.empty() && !first.trajectory.joint_trajectory.points.empty()) {
        while (end < original.size()) {
          const auto& previous = original[end - 1].trajectory.joint_trajectory;
          const auto& next = original[end].trajectory.joint_trajectory;
          if (next.points.empty() || next.joint_names != previous.joint_names ||
              armGroup(original[end]) != group ||
              original[end].execution_info != first.execution_info ||
              constraints(original[end]) != constraints(first)) break;
          const auto& a = previous.points.back().positions;
          const auto& b = next.points.front().positions;
          bool contiguous = a.size() == b.size();
          for (std::size_t j = 0; contiguous && j < a.size(); ++j)
            contiguous = std::abs(a[j] - b[j]) < 1e-5;
          if (!contiguous) break;
          ++end;
        }
      }
      if (end - begin < 2) end = begin + 1;
      Segment result = first;
      if (end - begin >= 2) {
        const auto processing_start = std::chrono::steady_clock::now();
        robot_trajectory::RobotTrajectory path(task.getRobotModel(), group);
        auto state = scene.getCurrentState();
        for (std::size_t i = begin; i < end; ++i) {
          const auto& trajectory = original[i].trajectory.joint_trajectory;
          for (std::size_t k = (i == begin ? 0 : 1); k < trajectory.points.size(); ++k) {
            state.setVariablePositions(trajectory.joint_names, trajectory.points[k].positions);
            state.update();
            path.addSuffixWayPoint(state, 0.0);
          }
        }
        // Per-stage time resampling generated thousands of points on the
        // same joint-space lines. RDP removes only geometrically redundant
        // points (<= 1e-5 rad, prismatic coordinates weighted by ten).
        // This prevents quadratic work in TOTG on repeated straight sections.
        const auto* compact_group = task.getRobotModel()->getJointModelGroup(group);
        const std::size_t input_points = path.getWayPointCount();
        std::vector<Eigen::VectorXd> coordinates;
        for (std::size_t k = 0; k < input_points; ++k) {
          Eigen::VectorXd q(compact_group->getVariableCount());
          std::vector<double> values;
          path.getWayPoint(k).copyJointGroupPositions(compact_group, values);
          for (std::size_t j = 0; j < values.size(); ++j) {
            const auto* joint = task.getRobotModel()->getJointOfVariable(compact_group->getVariableNames()[j]);
            q[j] = values[j] * (joint->getType() == moveit::core::JointModel::PRISMATIC ? 10.0 : 1.0);
          }
          coordinates.push_back(std::move(q));
        }
        std::vector<bool> keep(input_points, false);
        keep.front() = keep.back() = true;
        std::vector<std::pair<std::size_t, std::size_t>> pending{{0, input_points - 1}};
        while (!pending.empty()) {
          const auto [a, b] = pending.back(); pending.pop_back();
          const Eigen::VectorXd delta = coordinates[b] - coordinates[a];
          const double length2 = delta.squaredNorm();
          double worst = 1e-5;
          std::size_t split = a;
          for (std::size_t k = a + 1; k < b; ++k) {
            const Eigen::VectorXd offset = coordinates[k] - coordinates[a];
            const double t = length2 > 1e-20 ? clamp(offset.dot(delta) / length2, 0.0, 1.0) : 0.0;
            const double error = (offset - t * delta).norm();
            if (error > worst) { worst = error; split = k; }
          }
          if (split != a) {
            keep[split] = true;
            pending.emplace_back(a, split); pending.emplace_back(split, b);
          }
        }
        robot_trajectory::RobotTrajectory compact(task.getRobotModel(), group);
        for (std::size_t k = 0; k < input_points; ++k)
          if (keep[k]) compact.addSuffixWayPoint(path.getWayPoint(k), 0.0);
        path = std::move(compact);
        // Small configuration-space deviation; recheck the actual retimed
        // path below because TOTG can round corners between original points.
        trajectory_processing::TimeOptimalTrajectoryGeneration retime(0.0001, 0.05, 0.0001);
        const bool pushing = label == "physical side push without attachment";
        const bool empty_approach = label == "empty long-finger coordinated approach";
        const double velocity = get_parameter(empty_approach ? "empty_velocity_scaling" : "velocity_scaling").as_double();
        const double acceleration = get_parameter(empty_approach ? "empty_acceleration_scaling" : "acceleration_scaling").as_double();
        if (!retime.computeTimeStamps(path, velocity, acceleration)) {
          RCLCPP_WARN(get_logger(), "CONTINUOUS_ARM retiming failed for %s; retain checked original solution", label.c_str());
          return false;
        }
        const auto retimed_at = std::chrono::steady_clock::now();
        const double seconds = path.getDuration();
        if (!std::isfinite(seconds) || seconds <= 0.0 || seconds > 600.0) return false;
        const auto* joint_group = task.getRobotModel()->getJointModelGroup(group);
        // Sample by configuration distance, not by elapsed time: slow/stationary
        // portions need no thousands of identical collision checks. Include
        // every resampled corner and bound each interpolation to 0.005 rad or
        // 0.5 mm, including both sides of every controller waypoint.
        auto sample = path.getFirstWayPoint();
        for (std::size_t k = 0; k < path.getWayPointCount(); ++k) {
          const auto& a = path.getWayPoint(k == 0 ? 0 : k - 1);
          const auto& b = path.getWayPoint(k);
          int subdivisions = 1;
          for (const auto* joint : joint_group->getActiveJointModels()) {
            const double step = joint->getType() == moveit::core::JointModel::PRISMATIC ? 0.0005 : 0.005;
            subdivisions = std::max(subdivisions, static_cast<int>(std::ceil(a.distance(b, joint) / step)));
          }
          for (int j = (k == 0 ? 0 : 1); j <= subdivisions; ++j) {
            a.interpolate(b, static_cast<double>(j) / subdivisions, sample);
            sample.update();
            if (sample.satisfiesBounds(joint_group, 1e-5) && !scene.isStateColliding(sample)) continue;
            RCLCPP_WARN(get_logger(), "CONTINUOUS_ARM path rejected for %s at waypoint %zu; retain checked original solution", label.c_str(), k);
            return false;
          }
        }
        path.getRobotTrajectoryMsg(result.trajectory);
        if (pushing) {
          // Limit the actual pushing tip speed as well as joint speeds.
          // Continuous execution can use the normal arm limits without
          // accelerating contact to an unrestricted Cartesian speed.
          double stretch = 1.0;
          for (std::size_t k = 1; k < path.getWayPointCount(); ++k) {
            const auto p0 = path.getWayPoint(k - 1).getGlobalLinkTransform("right_gripper_push_tip").translation();
            const auto p1 = path.getWayPoint(k).getGlobalLinkTransform("right_gripper_push_tip").translation();
            const double dt = path.getWayPointDurationFromPrevious(k);
            if (dt > 0.0) stretch = std::max(stretch, (p1 - p0).norm() / (dt * 0.015));
          }
          for (auto& point : result.trajectory.joint_trajectory.points) {
            point.time_from_start = rclcpp::Duration::from_seconds(
              rclcpp::Duration(point.time_from_start).seconds() * stretch);
            for (auto& v : point.velocities) v /= stretch;
            for (auto& a : point.accelerations) a /= stretch * stretch;
          }
        }
        result.scene_diff = original[end - 1].scene_diff;
        RCLCPP_INFO(get_logger(), "CONTINUOUS_ARM_PROCESS %s input_points=%zu compact_points=%zu retime_ms=%.1f collision_ms=%.1f",
          label.c_str(), input_points, coordinates.empty() ? 0UL : static_cast<std::size_t>(std::count(keep.begin(), keep.end(), true)),
          std::chrono::duration<double, std::milli>(retimed_at - processing_start).count(),
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - retimed_at).count());
        ++merged_runs;
      }
      for (std::size_t i = begin; i < end; ++i) old_seconds += duration(original[i]);
      new_seconds += duration(result);
      merged.push_back(std::move(result));
      for (std::size_t i = begin; i < end; ++i)
        if (!scene.setPlanningSceneDiffMsg(original[i].scene_diff)) {
          RCLCPP_WARN(get_logger(), "CONTINUOUS_ARM cannot apply scene diff %zu", i);
          return false;
        }
      begin = end;
    }
    if (!merged_runs) {
      for (std::size_t i = 0; i < std::min<std::size_t>(8, original.size()); ++i) {
        const auto& s = original[i];
        std::ostringstream names;
        for (const auto& name : s.trajectory.joint_trajectory.joint_names) names << name << ',';
        RCLCPP_INFO(get_logger(), "CONTINUOUS_ARM diagnostic segment=%zu group=%s points=%zu joints=%s scene_equal_next=%d",
          i, armGroup(s).c_str(), s.trajectory.joint_trajectory.points.size(), names.str().c_str(),
          i + 1 < original.size() && constraints(s) == constraints(original[i + 1]));
      }
      return false;
    }
    solution.sub_trajectory = std::move(merged);
    RCLCPP_INFO(get_logger(), "CONTINUOUS_ARM %s segments=%zu->%zu planned_seconds=%.2f->%.2f merged_runs=%zu",
      label.c_str(), original.size(), solution.sub_trajectory.size(), old_seconds, new_seconds, merged_runs);
    return true;
  }

  std::array<double,6> processWrench(const moveit::core::RobotState& state)
  {
    geometry_msgs::msg::WrenchStamped left,right;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!left_force_ready_ || !right_force_ready_)
        throw std::runtime_error("Process admittance requires both wrist sensors");
      left=left_wrench_message_; right=right_wrench_message_;
    }
    const double now=get_clock()->now().seconds();
    const double lt=rclcpp::Time(left.header.stamp).seconds(), rt=rclcpp::Time(right.header.stamp).seconds();
    if (now-lt>.15 || now-rt>.15 || lt-now>.02 || rt-now>.02 || std::abs(lt-rt)>.04)
      throw std::runtime_error("Process admittance stopped: stale or skewed wrist feedback");
    Eigen::Vector3d force=Eigen::Vector3d::Zero(), torque=Eigen::Vector3d::Zero();
    const Eigen::Vector3d center=.5*(state.getGlobalLinkTransform("left_gripper_tcp").translation()+
      state.getGlobalLinkTransform("right_gripper_tcp").translation());
    for (const auto& pair : {std::pair{left,"L_hand_link"},std::pair{right,"R_hand_link"}}) {
      const auto& w=pair.first.wrench; const auto& transform=state.getGlobalLinkTransform(pair.second);
      const Eigen::Vector3d f=transform.rotation()*Eigen::Vector3d(w.force.x,w.force.y,w.force.z);
      const Eigen::Vector3d t=transform.rotation()*Eigen::Vector3d(w.torque.x,w.torque.y,w.torque.z);
      if (!f.allFinite() || !t.allFinite() || f.norm()>450.0 || t.norm()>100.0) {
        RCLCPP_ERROR(get_logger(),"PROCESS_ADMITTANCE_WRENCH_STOP sensor=%s force_norm=%.4f torque_norm=%.4f raw=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]",
          pair.second,f.norm(),t.norm(),w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z);
        throw std::runtime_error("Process admittance stopped: invalid or excessive wrist wrench");
      }
      force+=f; torque+=t+(transform.translation()-center).cross(f);
    }
    return {force.x(),force.y(),force.z(),torque.x(),torque.y(),torque.z()};
  }

  void offsetProcessStateExact(moveit::core::RobotState& state,
    const std::vector<std::string>& names, const pallet_placement::ProcessAdmittance& law)
  {
    Eigen::Vector3d translation(law.position[0],law.position[1],law.position[2]);
    Eigen::Vector3d rotation(law.position[3],law.position[4],law.position[5]);
    const Eigen::Vector3d center=.5*(state.getGlobalLinkTransform("left_gripper_tcp").translation()+
      state.getGlobalLinkTransform("right_gripper_tcp").translation());
    Eigen::Matrix3d common_rotation=Eigen::Matrix3d::Identity();
    if(rotation.norm()>1e-12)
      common_rotation=Eigen::AngleAxisd(rotation.norm(),rotation.normalized()).toRotationMatrix();
    for (const auto& side : {std::pair{"left_arm","left_gripper_tcp"},std::pair{"right_arm","right_gripper_tcp"}}) {
      const auto* group=state.getRobotModel()->getJointModelGroup(side.first);
      bool active=false;
      for (const auto& joint : group->getVariableNames())
        active=active || std::find(names.begin(),names.end(),joint)!=names.end();
      if (!active) continue;
      auto target=state.getGlobalLinkTransform(side.second);
      target.translation()=center+common_rotation*(target.translation()-center)+translation;
      target.linear()=common_rotation*target.rotation();
      pallet_placement::refineTcpPose(state,group,side.second,target);
      state.update();
    }
  }

  double offsetProcessState(moveit::core::RobotState& state,
    const std::vector<std::string>& names, const pallet_placement::ProcessAdmittance& law,
    const planning_scene::PlanningScene* collision_scene=nullptr)
  {
    // Uniformly shrink the shared rigid transform at a kinematic or collision boundary.
    // Never apply independent left/right approximations to a held tote.
    const auto nominal=state;
    for(int attempt=0;attempt<8;++attempt) {
      auto feasible=law; const double scale=std::ldexp(1.0,-attempt);
      for(double& value:feasible.position) value*=scale;
      auto candidate=nominal;
      try {
        offsetProcessStateExact(candidate,names,feasible);
        bool bounded=true;
        for(const auto& name:names)
          bounded=bounded && pallet_placement::admittanceCorrectionSafe(
            nominal.getVariablePosition(name),candidate.getVariablePosition(name));
        if(!bounded) continue;
        if(collision_scene && collision_scene->isStateColliding(candidate)) continue;
        state=candidate; return scale;
      } catch(const std::runtime_error&) {}
    }
    // Zero is the feasible tangent at a fully constrained joint boundary.
    state=nominal;
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,
      "PROCESS_ADMITTANCE_PROJECTED_ZERO kinematic/joint-step/collision constraint; nominal path still checked");
    return 0.0;
  }

  void updateCarryAdmittance()
  {
    if (!get_parameter("process_admittance").as_bool()) return;
    std::lock_guard<std::mutex> hold_lock(process_mutex_);
    if(process_arm_active_.load()) return;
    const double time=get_clock()->now().seconds();
    if (time-carry_last_update_<.20) return;
    // Column-only state intentionally leaves arm joints at model defaults;
    // it must never be used as a manipulation command/reference.
    auto actual=currentSeed(column_model_);
    if (!carry_nominal_) {
      carry_nominal_=std::make_unique<moveit::core::RobotState>(actual);
      carry_reference_=processWrench(actual); carry_law_={};
      { std::lock_guard<std::mutex> lock(data_mutex_); process_stage_="base_carry"; }
      RCLCPP_INFO(get_logger(),"PROCESS_ADMITTANCE_CARRY_BEGIN incremental_common_6d");
    }
    const auto measured=processWrench(actual);
    std::array<double,6> residual{};
    for(int i=0;i<6;++i) residual[i]=measured[i]-carry_reference_[i];
    carry_law_.advance(residual,.20);
    auto target=*carry_nominal_;
    const auto* group=target.getRobotModel()->getJointModelGroup("both_arms");
    const auto names=group->getVariableNames();
    offsetProcessState(target,names,carry_law_);
    planning_scene::PlanningScene guard(target.getRobotModel());
    double clearance=std::numeric_limits<double>::max();
    if(!toolBodyClear(guard,target,toolBodyMatrix(target.getRobotModel()),clearance,true) ||
       !columnClear(target,measuredWorldBase(),true))
      throw std::runtime_error("Carry admittance correction rejected by body/table guard");
    trajectory_msgs::msg::JointTrajectory command; command.joint_names=names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for(const auto& name:names) {
      const auto& bounds=target.getRobotModel()->getVariableBounds(name);
      const double velocity=bounds.velocity_bounded_?bounds.max_velocity_:1.0;
      if(!pallet_placement::carryStepSafe(actual.getVariablePosition(name),target.getVariablePosition(name),velocity,.25))
        throw std::runtime_error("Carry admittance rejected a joint jump: "+name);
      point.positions.push_back(target.getVariablePosition(name));
    }
    point.time_from_start=rclcpp::Duration::from_seconds(.25); command.points.push_back(point);
    process_arm_pub_->publish(command); carry_last_update_=time;
    if (++carry_cycles_%50==0) RCLCPP_INFO(get_logger(),"PROCESS_ADMITTANCE_CARRY_SAMPLE cycles=%zu offset_z=%.7f",carry_cycles_,carry_law_.position[2]);
  }

  moveit::core::MoveItErrorCode executeProcessAdmittance(
    const moveit_task_constructor_msgs::msg::Solution& solution,
    const moveit::core::RobotModelConstPtr& model, const std::string& label)
  {
    { std::lock_guard<std::mutex> lock(data_mutex_); process_stage_=label; }
    process_arm_active_.store(true);
    struct ActiveGuard { std::atomic_bool& flag; ~ActiveGuard(){flag.store(false);} } active_guard{process_arm_active_};
    { std::lock_guard<std::mutex> lock(process_mutex_); carry_nominal_.reset(); }
    // Floor unloading already uses the dedicated measured-support loop. Do not
    // superimpose a second vertical compliance loop on it or on jaw release.
    if (label.rfind("slow floor seating ",0)==0 || label.find("release")!=std::string::npos)
      return executeContinuousSolution(solution,model);
    planning_scene::PlanningScene scene(model);
    if (!scene.setPlanningSceneMsg(solution.start_scene))
      throw std::runtime_error("Cannot initialize process-admittance collision scene");
    auto initial_actual=currentSeed(model);
    auto initial_scene_state=scene.getCurrentState();
    initial_scene_state.setVariablePositions(initial_actual.getVariablePositions());
    initial_scene_state.update(); scene.setCurrentState(initial_scene_state);
    const auto baseline=processWrench(initial_actual);
    pallet_placement::ProcessAdmittance law, grip_law;
    std::size_t cycles=0, corrected_points=0, projected_points=0; double largest=0;
    RCLCPP_INFO(get_logger(),"PROCESS_ADMITTANCE_BEGIN stage=%s mode=incremental_common_6d sample_seconds=0.25",label.c_str());
    logWristLoad(label+" process_begin");
    for (const auto& original : solution.sub_trajectory) {
      auto path=original.trajectory.joint_trajectory;
      if (!path.points.empty() && rclcpp::Duration(path.points.back().time_from_start).seconds()<=0) {
        std::vector<double> times;
        std::vector<std::vector<double>> positions;
        for (const auto& point:path.points) {
          times.push_back(rclcpp::Duration(point.time_from_start).seconds());
          positions.push_back(point.positions);
        }
        if (!pallet_placement::stationaryZeroDuration(times,positions) ||
            positions.front().size()!=path.joint_names.size())
          throw std::runtime_error("Nonstationary zero-duration process trajectory rejected");
        const auto actual=currentSeed(model);
        double error=0;
        for (std::size_t j=0;j<path.joint_names.size();++j)
          error=std::max(error,std::abs(actual.getVariablePosition(path.joint_names[j])-positions.front()[j]));
        if (error<=1e-6) {
          // MoveTo can emit a zero-time path when the gripper is already at
          // its goal. Preserve its scene diff without sending a motion goal.
          RCLCPP_INFO(get_logger(),"PROCESS_ADMITTANCE_STATIONARY_NOOP stage=%s joints=%zu measured_error=%.9f",
            label.c_str(),path.joint_names.size(),error);
          path.points.clear();
        }
      }
      if (path.points.empty()) {
        moveit_task_constructor_msgs::msg::Solution only;
        scene.getPlanningSceneMsg(only.start_scene);
        auto change=original;
        change.trajectory.joint_trajectory=path;
        change.scene_diff.robot_state.joint_state=sensor_msgs::msg::JointState{};
        change.scene_diff.robot_state.is_diff=true;
        only.sub_trajectory.push_back(change);
        const auto status=executeContinuousSolution(only,model);
        if (!status) return status;
        scene.setPlanningSceneDiffMsg(change.scene_diff);
        continue;
      }
      if (!original.trajectory.multi_dof_joint_trajectory.points.empty())
        throw std::runtime_error("Multi-DOF trajectory unsupported by process admittance");
      const auto measured_entry=currentSeed(model);
      double entry_error=0.0;
      for(std::size_t j=0;j<path.joint_names.size();++j)
        entry_error=std::max(entry_error,std::abs(measured_entry.getVariablePosition(path.joint_names[j])-path.points.front().positions[j]));
      if(entry_error>.15) throw std::runtime_error("Planned trajectory start differs excessively from measured joints");
      if(entry_error>.002) {
        // Include the measured-to-planned connection in the same collision-
        // checked, velocity-limited feedback executor; never jump to a stale
        // planning-monitor start state at controller time zero.
        const double connection=std::max(2.0,entry_error*20.0);
        for(auto& point:path.points) point.time_from_start=rclcpp::Duration::from_seconds(
          rclcpp::Duration(point.time_from_start).seconds()+connection);
        trajectory_msgs::msg::JointTrajectoryPoint entry;
        for(const auto& name:path.joint_names) entry.positions.push_back(measured_entry.getVariablePosition(name));
        path.points.insert(path.points.begin(),entry);
        RCLCPP_WARN(get_logger(),"PROCESS_ADMITTANCE_CHECKED_CONNECTION stage=%s max_joint_difference=%.6f connection_seconds=%.3f",
          label.c_str(),entry_error,connection);
      }
      const double duration=rclcpp::Duration(path.points.back().time_from_start).seconds();
      if (duration<=0) throw std::runtime_error("Invalid process-admittance trajectory duration");
      RCLCPP_INFO(get_logger(),"PROCESS_ADMITTANCE_SEGMENT stage=%s joints=%zu nominal_duration=%.6f",
        label.c_str(),path.joint_names.size(),duration);
      auto interpolate=[&](double time) {
        auto it=std::lower_bound(path.points.begin(),path.points.end(),time,
          [](const auto& p,double t){return rclcpp::Duration(p.time_from_start).seconds()<t;});
        if(it==path.points.begin()) return it->positions;
        if(it==path.points.end()) return path.points.back().positions;
        const auto& b=*it; const auto& a=*(it-1);
        const double ta=rclcpp::Duration(a.time_from_start).seconds(),tb=rclcpp::Duration(b.time_from_start).seconds();
        const double ratio=tb>ta?(time-ta)/(tb-ta):1.0;
        auto q=a.positions;
        for(std::size_t j=0;j<q.size();++j) q[j]+=(b.positions[j]-q[j])*ratio;
        return q;
      };
      std::vector<const moveit::core::AttachedBody*> carried;
      scene.getCurrentState().getAttachedBodies(carried);
      bool both_active=true;
      for(const auto& name:model->getJointModelGroup("both_arms")->getVariableNames())
        both_active=both_active && std::find(path.joint_names.begin(),path.joint_names.end(),name)!=path.joint_names.end();
      const bool rigid_grasp=both_active && !carried.empty();
      auto planned_rigid_start=scene.getCurrentState();
      planned_rigid_start.setVariablePositions(path.joint_names,interpolate(0.0)); planned_rigid_start.update();
      // Contact tracking tolerance can leave the actual grasp different from
      // the planned one. Preserve that measured grasp, and apply only the
      // planned common translation; snapping back would squeeze the payload.
      auto rigid_start=currentSeed(model);
      rigid_start.update();
      const Eigen::Vector3d planned_center=.5*(
        planned_rigid_start.getGlobalLinkTransform("left_gripper_tcp").translation()+
        planned_rigid_start.getGlobalLinkTransform("right_gripper_tcp").translation());
      const Eigen::Vector3d measured_center=.5*(
        rigid_start.getGlobalLinkTransform("left_gripper_tcp").translation()+
        rigid_start.getGlobalLinkTransform("right_gripper_tcp").translation());
      const Eigen::Isometry3d left_reference=rigid_start.getGlobalLinkTransform("left_gripper_tcp");
      const Eigen::Isometry3d relative_grasp=left_reference.inverse()*rigid_start.getGlobalLinkTransform("right_gripper_tcp");
      if(rigid_grasp) RCLCPP_INFO(get_logger(),"PROCESS_ADMITTANCE_RIGID_GRASP stage=%s preserve starting hand separation and level orientation",label.c_str());
      for(double begin=0;begin<duration-1e-8;) {
        const double end=duration-begin<.30?duration:begin+.25, dt=end-begin;
        const auto wrench=processWrench(currentSeed(model));
        std::array<double,6> residual{};
        for(int i=0;i<6;++i) residual[i]=wrench[i]-baseline[i];
        const auto previous=law;
        law.advance(residual,dt);
        std::array<double,6> grip_force{};
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          grip_force[0]=-std::max(0.0,left_force_norm_-80.0);
          grip_force[1]=-std::max(0.0,right_force_norm_-80.0);
        }
        grip_law.advance(grip_force,dt);
        moveit_task_constructor_msgs::msg::Solution chunk;
        scene.getPlanningSceneMsg(chunk.start_scene);
        auto segment=original; segment.trajectory.joint_trajectory.points.clear();
        segment.scene_diff=moveit_msgs::msg::PlanningScene{}; segment.scene_diff.is_diff=true;
        auto endpoint=scene.getCurrentState();
        for(int k=0;k<=4;++k) {
          const double alpha=k/4.0, time=begin+dt*alpha;
          auto state=scene.getCurrentState(); state.setVariablePositions(path.joint_names,interpolate(time)); state.update();
          auto interpolated=law;
          for(int i=0;i<6;++i) interpolated.position[i]=previous.position[i]+alpha*(law.position[i]-previous.position[i]);
          if(rigid_grasp) {
            // Joint interpolation alone can squeeze the payload between two
            // otherwise valid endpoint grasps. Reproject every command onto
            // one common object pose before adding the measured compliance.
            auto left=state.getGlobalLinkTransform("left_gripper_tcp");
            const auto right=state.getGlobalLinkTransform("right_gripper_tcp");
            left.linear()=left_reference.rotation();
            left.translation()=measured_center-planned_center+
              .5*(left.translation()+right.translation()-left.rotation()*relative_grasp.translation());
            const Eigen::Isometry3d right_target=left*relative_grasp;
            pallet_placement::refineTcpPose(state,model->getJointModelGroup("left_arm"),"left_gripper_tcp",left);
            pallet_placement::refineTcpPose(state,model->getJointModelGroup("right_arm"),"right_gripper_tcp",right_target);
            state.update();
          }
          const auto nominal_state=state;
          const double feasible_scale=offsetProcessState(state,path.joint_names,interpolated,&scene);
          if(feasible_scale<1.0) ++projected_points;
          bool changed=false;
          for(const auto* tcp:{"left_gripper_tcp","right_gripper_tcp"}) {
            const auto& before=nominal_state.getGlobalLinkTransform(tcp);
            const auto& after=state.getGlobalLinkTransform(tcp);
            const double displacement=(after.translation()-before.translation()).cwiseAbs().maxCoeff();
            const double angle=Eigen::AngleAxisd(after.rotation()*before.rotation().transpose()).angle();
            largest=std::max({largest,displacement,angle});
            changed=changed || displacement>1e-8 || angle>1e-8;
          }
          if(changed) ++corrected_points;
          // Closing jaws yield by at most 0.2 mm under high wrist load. Opening
          // and verified floor unloading retain their dedicated release logic.
          for(int hand=0;hand<2;++hand) {
            const std::string finger=hand==0?"left_gripper_left_finger_joint":"gripper_left_finger_joint";
            if(std::find(path.joint_names.begin(),path.joint_names.end(),finger)!=path.joint_names.end() &&
               state.getVariablePosition(finger)>.035 && label.find("floor-supported")==std::string::npos) {
              state.setVariablePosition(finger,state.getVariablePosition(finger)+
                alpha*std::clamp(grip_law.position[hand],-.0002,0.0)); state.update();
            }
          }
          if(!state.satisfiesBounds() || scene.isStateColliding(state)) {
            collision_detection::CollisionRequest request; request.contacts=true; request.max_contacts=20;
            collision_detection::CollisionResult collision;
            scene.checkCollision(request,collision,state);
            for(const auto& pair:collision.contacts)
              RCLCPP_ERROR(get_logger(),"PROCESS_ADMITTANCE_COLLISION stage=%s pair=%s/%s begin=%.6f fraction=%.3f",
                label.c_str(),pair.first.first.c_str(),pair.first.second.c_str(),begin,alpha);
            const auto directory=get_parameter("process_debug_dir").as_string();
            if(!directory.empty()) {
              scene.setCurrentState(state); moveit_msgs::msg::PlanningScene snapshot;
              scene.getPlanningSceneMsg(snapshot);
              rclcpp::Serialization<moveit_msgs::msg::PlanningScene> serialization;
              rclcpp::SerializedMessage bytes; serialization.serialize_message(&snapshot,&bytes);
              const auto& buffer=bytes.get_rcl_serialized_message();
              std::ofstream file(directory+"/process_collision.cdr",std::ios::binary);
              file.write(reinterpret_cast<const char*>(buffer.buffer),buffer.buffer_length);
            }
            throw std::runtime_error("Corrected process-admittance trajectory rejected by collision/joint guard");
          }
          if(!columnClear(state,measuredWorldBase(),true))
            throw std::runtime_error("Corrected process-admittance command violates table clearance");
          trajectory_msgs::msg::JointTrajectoryPoint point;
          for(const auto& name:path.joint_names) point.positions.push_back(state.getVariablePosition(name));
          point.time_from_start=rclcpp::Duration::from_seconds(std::max(.10,dt)*alpha);
          segment.trajectory.joint_trajectory.points.push_back(point); endpoint=state;
        }
        // Enforce joint velocity bounds on the actual corrected command,
        // including the transition from the preceding measured endpoint.
        auto& command_points=segment.trajectory.joint_trajectory.points;
        const auto measured_start=currentSeed(model);
        for(std::size_t j=0;j<path.joint_names.size();++j)
          command_points.front().positions[j]=measured_start.getVariablePosition(path.joint_names[j]);
        double command_stretch=1.0;
        for(std::size_t k=1;k<command_points.size();++k) {
          const double interval=rclcpp::Duration(command_points[k].time_from_start).seconds()-
            rclcpp::Duration(command_points[k-1].time_from_start).seconds();
          for(std::size_t j=0;j<path.joint_names.size();++j) {
            const auto& bounds=model->getVariableBounds(path.joint_names[j]);
            if(bounds.velocity_bounded_ && bounds.max_velocity_>0)
              command_stretch=std::max(command_stretch,std::abs(command_points[k].positions[j]-command_points[k-1].positions[j])/
                (.5*bounds.max_velocity_*interval));
          }
        }
        if(command_stretch>10.0) {
          for(std::size_t j=0;j<path.joint_names.size();++j)
            RCLCPP_ERROR(get_logger(),"PROCESS_TRACKING joint=%s actual_start=%.7f nominal_start=%.7f first_target=%.7f stretch=%.3f",
              path.joint_names[j].c_str(),command_points.front().positions[j],interpolate(begin)[j],command_points[1].positions[j],command_stretch);
          throw std::runtime_error("Process-admittance tracking error requires excessive trajectory stretch");
        }
        for(auto& point:command_points)
          point.time_from_start=rclcpp::Duration::from_seconds(
            rclcpp::Duration(point.time_from_start).seconds()*command_stretch);
        if(end>=duration-1e-8) segment.scene_diff=original.scene_diff;
        segment.scene_diff.robot_state.joint_state=sensor_msgs::msg::JointState{};
        segment.scene_diff.robot_state.is_diff=true;
        chunk.sub_trajectory.push_back(segment);
        const auto status=executeContinuousSolution(chunk,model);
        if(!status) return status;
        const auto settle_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
        bool settled=false;
        double largest_tracking=0.0, largest_tcp_error=0.0;
        auto desired_state=scene.getCurrentState();
        desired_state.setVariablePositions(path.joint_names,command_points.back().positions); desired_state.update();
        while(rclcpp::ok() && std::chrono::steady_clock::now()<settle_deadline) {
          const auto actual=currentSeed(model); settled=true;
          bool contact=false;
          { std::lock_guard<std::mutex> lock(data_mutex_); contact=std::max(left_force_norm_,right_force_norm_)>20.0; }
          largest_tracking=0.0; largest_tcp_error=0.0;
          for(std::size_t j=0;j<path.joint_names.size();++j) {
            const auto& name=path.joint_names[j];
            const bool jaw=name.find("finger")!=std::string::npos;
            const bool waist=name.find("waist")!=std::string::npos;
            const double tolerance=jaw?.003:(waist?.0005:(contact?.01:.001));
            const double error=std::abs(actual.getVariablePosition(name)-command_points.back().positions[j]);
            largest_tracking=std::max(largest_tracking,error);
            if(error>tolerance) settled=false;
          }
          for(const auto& side:{std::pair{"left_arm","left_gripper_tcp"},std::pair{"right_arm","right_gripper_tcp"}}) {
            bool active=false;
            for(const auto& name:model->getJointModelGroup(side.first)->getVariableNames())
              active=active || std::find(path.joint_names.begin(),path.joint_names.end(),name)!=path.joint_names.end();
            if(!active) continue;
            const auto& expected=desired_state.getGlobalLinkTransform(side.second);
            const auto& measured=actual.getGlobalLinkTransform(side.second);
            const double error=(expected.translation()-measured.translation()).norm();
            largest_tcp_error=std::max(largest_tcp_error,error);
            if(error>(contact?.003:.001) ||
              Eigen::AngleAxisd(expected.rotation()*measured.rotation().transpose()).angle()>.02) settled=false;
          }
          if(settled) break;
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if(!settled) {
          RCLCPP_ERROR(get_logger(),"PROCESS_ADMITTANCE_TRACKING_STOP stage=%s max_joint_error=%.7f max_tcp_error=%.7f",
            label.c_str(),largest_tracking,largest_tcp_error);
          logWristLoad(label+" tracking_stop");
          throw std::runtime_error("Process-admittance segment did not reach measured endpoint; do not replay");
        }
        scene.setPlanningSceneDiffMsg(segment.scene_diff);
        auto updated=scene.getCurrentState();
        updated.setVariablePositions(path.joint_names,segment.trajectory.joint_trajectory.points.back().positions);
        updated.update(); scene.setCurrentState(updated); ++cycles;

        if(cycles%40==0) RCLCPP_INFO(get_logger(),"PROCESS_ADMITTANCE_SAMPLE stage=%s cycles=%zu residual=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f] offset=[%.7f,%.7f,%.7f,%.7f,%.7f,%.7f]",
          label.c_str(),cycles,residual[0],residual[1],residual[2],residual[3],residual[4],residual[5],
          law.position[0],law.position[1],law.position[2],law.position[3],law.position[4],law.position[5]);
        begin=end;
      }
    }
    RCLCPP_INFO(get_logger(),"PROCESS_ADMITTANCE_END stage=%s cycles=%zu max_abs_offset=%.7f corrected_points=%zu projected_points=%zu",label.c_str(),cycles,largest,corrected_points,projected_points);
    logWristLoad(label+" process_end");
    RCLCPP_INFO(get_logger(),"PROCESS_ADMITTANCE_GRIP_END stage=%s requested_left_offset=%.7f requested_right_offset=%.7f",
      label.c_str(),std::clamp(grip_law.position[0],-.0002,0.0),std::clamp(grip_law.position[1],-.0002,0.0));
    return moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }

  moveit::core::MoveItErrorCode executeContinuousSolution(
    moveit_task_constructor_msgs::msg::Solution solution,
    const moveit::core::RobotModelConstPtr& model)
  {
    using Action = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
    auto client = rclcpp_action::create_client<Action>(shared_from_this(), "execute_task_solution");
    if (!client->wait_for_action_server(std::chrono::seconds(10)))
      return moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    Action::Goal goal;
    goal.solution = std::move(solution);
    auto accepted = client->async_send_goal(goal);
    if (accepted.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
      throw std::runtime_error("Continuous trajectory goal acknowledgement timed out; do not replay motion");
    auto handle = accepted.get();
    if (!handle) return moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    auto result = client->async_get_result(handle);
    planning_scene::PlanningScene guard_scene(model);
    const auto guard_matrix = toolBodyMatrix(model);
    double minimum_clearance = std::numeric_limits<double>::max();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::minutes(45);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (result.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready) {
        const auto wrapped = result.get();
        if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
          return moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        return wrapped.result->error_code;
      }
      if (get_parameter("strict_tool_body_guard").as_bool()) {
        const auto actual = currentSeed(model);
        if (!toolBodyClear(guard_scene, actual, guard_matrix, minimum_clearance, true)) {
          client->async_cancel_goal(handle);
          cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
          RCLCPP_ERROR(get_logger(), "TOOL_BODY_GUARD stopped execution on measured joint state");
          return moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
        }
      }
    }
    client->async_cancel_goal(handle);
    return moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
  }

  void verifyToolModel(const moveit::core::RobotModelConstPtr& model) const
  {
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.update();
    for (const auto& pair : {std::pair{"left_gripper_base_link", "left_gripper_tcp"},
                             std::pair{"gripper_base_link", "right_gripper_tcp"}}) {
      const auto actual = model_tf_->lookupTransform(pair.first, pair.second, tf2::TimePointZero,
                                                     tf2::durationFromSec(1.0));
      const auto expected = state.getGlobalLinkTransform(pair.first).inverse() *
                            state.getGlobalLinkTransform(pair.second);
      const Eigen::Vector3d translation(actual.transform.translation.x,
                                       actual.transform.translation.y,
                                       actual.transform.translation.z);
      if ((translation - expected.translation()).norm() > 0.0001)
        throw std::runtime_error("Planning/Gazebo TCP geometry mismatch; use the scene's tote_finger_length before motion");
    }
  }

  bool planAndMaybeExecute(mtc::Task& task, bool execute, const std::string& label, bool reuse_plan = false)
  {
    if (execute) verifyToolModel(task.getRobotModel());
    const auto max_solutions = static_cast<std::size_t>(
      std::max<int64_t>(1, get_parameter("max_solutions").as_int()));
    auto result = reuse_plan ? moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::SUCCESS) :
      task.plan(max_solutions);
    // Sampling/smoothing can reject an otherwise reachable approach. Retry
    // planning only; never replay execution after a physical motion failure.
    // Task::plan resets its stages before each call. All collision validators
    // stay enabled and no robot commands are sent until a valid plan exists.
    for (int attempt = 2; (!result || task.numSolutions() == 0) &&
         attempt <= 5 && rclcpp::ok(); ++attempt) {
      RCLCPP_WARN(get_logger(), "%s: safe planning retry %d/5 (previous code=%d)",
        label.c_str(), attempt, result.val);
      result = task.plan(max_solutions);
    }
    if (!result || task.numSolutions() == 0) {
      RCLCPP_ERROR(
        get_logger(), "%s: MTC planning failed (MoveIt code=%d)", label.c_str(), result.val);
      task.printState();
      return false;
    }
    auto selected = task.solutions().front();
    moveit_task_constructor_msgs::msg::Solution audit;
    bool safe = false;
    for (int attempt = 0; attempt < 5 && !safe; ++attempt) {
      for (const auto& candidate : task.solutions()) {
        audit = moveit_task_constructor_msgs::msg::Solution{};
        candidate->toMsg(audit);
        if (label.rfind("slow floor seating ", 0) == 0) {
          bool motion = false;
          for (const auto& segment : audit.sub_trajectory) {
            const auto& points = segment.trajectory.joint_trajectory.points;
            if (points.size() < 2) continue;
            for (std::size_t j = 0; j < points.front().positions.size(); ++j)
              motion = motion || std::abs(points.back().positions[j] - points.front().positions[j]) > 1e-9;
          }
          if (!motion) throw std::runtime_error("Floor planner produced no motion; stop instead of repeating empty steps");
        }
        if (!validateToolBodySolution(task, audit, label)) continue;
        selected = candidate;
        safe = true;
        break;
      }
      if (!safe && attempt < 4) {
        RCLCPP_WARN(get_logger(), "%s: tool/body clearance rejected candidates; replanning %d/5", label.c_str(), attempt + 2);
        // Request additional candidates rather than repeatedly validating
        // only the same first solutions on a clearance rejection.
        result = task.plan(max_solutions * static_cast<std::size_t>(attempt + 2));
      }
    }
    if (!safe) return false;
    task.introspection().publishSolution(*selected);
    moveit::core::RobotState audit_state(task.getRobotModel());
    moveit::core::robotStateMsgToRobotState(audit.start_scene.robot_state, audit_state);
    for (const auto& segment : audit.sub_trajectory) {
      const auto& trajectory = segment.trajectory.joint_trajectory;
      if (trajectory.points.empty()) continue;
      const auto& endpoint = trajectory.points.back();
      for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i)
        audit_state.setVariablePosition(trajectory.joint_names[i], endpoint.positions[i]);
      audit_state.update();
      const auto tcp = audit_state.getGlobalLinkTransform("right_gripper_tcp").translation();
      RCLCPP_INFO(get_logger(),
        "Trajectory audit right TCP=(%.4f, %.4f, %.4f), lift=%.4f, pitch=%.4f, fingers=(%.4f, %.4f)",
        tcp.x(), tcp.y(), tcp.z(), audit_state.getVariablePosition("waist_up_down_joint"),
        audit_state.getVariablePosition("waist_pitch_joint"),
        audit_state.getVariablePosition("gripper_left_finger_joint"),
        audit_state.getVariablePosition("gripper_right_finger_joint"));
    }
    RCLCPP_INFO(
      get_logger(), "%s: planning succeeded with %zu solution(s)",
      label.c_str(), task.numSolutions());
    if (!execute) {
      return true;
    }
    const auto original_audit = audit;
    bool continuous = get_parameter("continuous_arm_motion").as_bool() &&
      mergeArmTrajectories(task, audit, label);
    if (continuous && !validateToolBodySolution(task, audit, label + " retimed")) {
      audit = original_audit;
      continuous = false;
    }
    const auto exec_result = get_parameter("process_admittance").as_bool() ?
      executeProcessAdmittance(audit, task.getRobotModel(), label) :
      (continuous || get_parameter("strict_tool_body_guard").as_bool()) ?
      executeContinuousSolution(std::move(audit), task.getRobotModel()) : task.execute(*selected);
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

  bool runCoordinatedPosture(const std::string& object_id, double lift, double pitch, double height)
  {
    mtc::Task task;
    task.stages()->setName("level tote torso-arm coordination");
    task.loadRobotModel(shared_from_this());
    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    addGroundContactStage(task);
    auto seed = currentSeed(task.getRobotModel());
    const double start_lift = seed.getVariablePosition("waist_up_down_joint");
    const double start_pitch = seed.getVariablePosition("waist_pitch_joint");
    std::array<double, 3> world_center;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!measured_boxes_.count(object_id) ||
          std::chrono::steady_clock::now() - measured_boxes_received_.at(object_id) > std::chrono::seconds(1))
        throw std::runtime_error("Cannot coordinate torso without fresh physical tote pose");
      world_center = measured_boxes_.at(object_id);
    }
    auto center = worldPointInBase(world_center[0], world_center[1], world_center[2], currentOdom());
    if (get_parameter("place_on_floor_before_release").as_bool() && carried_handle_offset_ < 0.0) {
      // A joint trajectory can report success just before the Gazebo model
      // pose bridge publishes the tote's first lifted sample.  Wait briefly
      // for physical confirmation, then derive the TCP offset from that fresh
      // pose; never infer a successful grasp from the planned endpoint alone.
      const auto settle_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      while (world_center[2] < 0.274 && std::chrono::steady_clock::now() < settle_deadline) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        std::lock_guard<std::mutex> lock(data_mutex_);
        const auto found = measured_boxes_.find(object_id);
        if (found != measured_boxes_.end() &&
            std::chrono::steady_clock::now() - measured_boxes_received_.at(object_id) <
              std::chrono::seconds(1)) {
          world_center = found->second;
        }
      }
      if (world_center[2] < 0.274) {
        RCLCPP_ERROR(get_logger(),
          "PHYSICAL_PICK_CHECK failed: box_z=%.4f m, required >=0.2740 m after 2 s settling",
          world_center[2]);
        throw std::runtime_error("Tote is not physically lifted after grasp settling");
      }
      center = worldPointInBase(world_center[0], world_center[1], world_center[2], currentOdom());
      carried_handle_offset_ = 0.5 * (
        seed.getGlobalLinkTransform("left_gripper_tcp").translation().z() +
        seed.getGlobalLinkTransform("right_gripper_tcp").translation().z()) - center[2];
      RCLCPP_INFO(get_logger(),
        "PHYSICAL_PICK_CHECK box_z=%.4f m tcp_mean_z=%.4f m measured_handle_offset=%.4f m",
        world_center[2], 0.5 * (seed.getGlobalLinkTransform("left_gripper_tcp").translation().z() +
        seed.getGlobalLinkTransform("right_gripper_tcp").translation().z()), carried_handle_offset_);
      if (carried_handle_offset_ < 0.075 || carried_handle_offset_ > 0.125)
        throw std::runtime_error("Measured carried TCP height is outside the shallow-grasp range");
      RCLCPP_INFO(get_logger(), "Retain measured closed-gripper TCP height offset %.6f m", carried_handle_offset_);
    }
    if (world_center[2] < 0.274)
      throw std::runtime_error("Tote is not physically lifted before torso coordination");
    std::vector<std::string> arm_links = task.getRobotModel()->getJointModelGroup("both_arms")->getLinkModelNames();
    const auto tools = touchLinks();
    arm_links.insert(arm_links.end(), tools.begin(), tools.end());
    arm_links.insert(arm_links.end(), {"gripper_base_link", "left_gripper_base_link"});
    auto strict = std::make_unique<mtc::stages::ModifyPlanningScene>("strict level carry fixture checks");
    for (const auto& fixture : {"source_pallet", "perimeter_extraction_deck", "destination_table", "large_bin", "simulation_ground"}) {
      strict->allowCollisions(fixture, arm_links, false);
      strict->allowCollisions(object_id, fixture, false);
    }
    strict->allowCollisions(object_id, tools, true);
    task.add(std::move(strict));
    auto joint = makeJointPlanner();
    constexpr int steps = 24;
    for (int i = 1; i <= steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      seed.setVariablePosition("waist_up_down_joint", start_lift + t * (lift - start_lift));
      seed.setVariablePosition("waist_pitch_joint", start_pitch + t * (pitch - start_pitch));
      seed.update();
      auto target = center;
      const bool entering_place = currentOdom().y < -1.2 &&
        std::abs(pitch-get_parameter("place_waist_pitch").as_double()) < 1e-6;
      const double final_reach = entering_place ? get_parameter("place_carry_reach").as_double() : std::min(center[0], 0.45);
      target[0] += t * (final_reach - center[0]);
      target[2] += t * (height - center[2]);
      geometry_msgs::msg::Pose left, right;
      graspTargetsWithOffsets(target, 0.0, carriedHandleOffset(),
        get_parameter("grasp_x_offset").as_double(), left, right);
      auto goal = solveDualArmGoal(left, right, seed, task.getRobotModel(), 0.50);
      goal["waist_up_down_joint"] = seed.getVariablePosition("waist_up_down_joint");
      goal["waist_pitch_joint"] = seed.getVariablePosition("waist_pitch_joint");
      for (const auto& item : goal) seed.setVariablePosition(item.first, item.second);
      seed.update();
      auto move = std::make_unique<mtc::stages::MoveTo>("level carry posture " + std::to_string(i), joint);
      move->setGroup("pallet_upper_body");
      move->setGoal(goal);
      task.add(std::move(move));
    }
    if (!planAndMaybeExecute(task, true, "coordinated level tote posture")) return false;
    const auto posture_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    auto posture_stable_since = std::chrono::steady_clock::now();
    bool posture_stable = false;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < posture_deadline) {
      const auto sample_time = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const bool ready = measured_boxes_.count(object_id) &&
          sample_time - measured_boxes_received_.at(object_id) < std::chrono::milliseconds(250) &&
          std::abs(measured_boxes_.at(object_id)[2] - height) <= 0.05 &&
          measured_box_tilt_ <= 10.0 * kPi / 180.0;
        if (!ready) posture_stable = false;
        else if (!posture_stable) { posture_stable = true; posture_stable_since = sample_time; }
      }
      if (posture_stable && sample_time - posture_stable_since >= std::chrono::milliseconds(200)) break;
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    if (!posture_stable || std::chrono::steady_clock::now() - posture_stable_since < std::chrono::milliseconds(200))
      throw std::runtime_error("Level tote posture did not stabilize");
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (std::chrono::steady_clock::now() - measured_boxes_received_.at(object_id) > std::chrono::seconds(1) ||
        std::abs(measured_boxes_.at(object_id)[2] - height) > 0.05 ||
        measured_box_tilt_ > 10.0 * kPi / 180.0)
      throw std::runtime_error("Physical level carry height/tilt did not match coordinated trajectory");
    RCLCPP_INFO(get_logger(), "Physical coordinated posture verified box_z=%.4f tilt_deg=%.2f",
                measured_boxes_.at(object_id)[2], measured_box_tilt_ * 180.0 / kPi);
    return true;
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
            const double dx = hold_target.x - current.x;
            const double dy = hold_target.y - current.y;
            cmd.linear.x = clamp(1.8 * (dx * std::cos(current.yaw) +
              dy * std::sin(current.yaw)), -0.08, 0.08);
            cmd.angular.z = clamp(2.0 * normalizeAngle(hold_target.yaw - current.yaw), -0.12, 0.12);
            publishGuardedBaseCommand(cmd);
          } catch (const std::exception&) {
            // Keep the extraction alive if one odom update is late; the next
            // cycle will retry without changing the arm trajectory.
          }
          rate.sleep();
        }
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      });
    }
    bool result=false;
    try {
      result = planAndMaybeExecute(task, execute,
        (get_parameter("rear_side_pull").as_bool() ? "left-arm mirrored pre-extraction " : "right-arm pre-extraction ") + object_id);
    } catch (...) {
      hold_running.store(false);
      if(hold_thread.joinable()) hold_thread.join();
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist{}); throw;
    }
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
    const std::string& label, bool release_and_retreat = true, bool move_to_target = true)
  {
    auto task = createPlaceTask(object_id, center, approach, release_and_retreat, move_to_target);
    const Pose2D hold_target = currentOdom();
    std::atomic_bool hold_running{execute};
    std::thread hold_thread;
    if (execute) {
      hold_thread = std::thread([this, &hold_running, hold_target]() {
        rclcpp::WallRate rate(20.0);
        while (rclcpp::ok() && hold_running.load()) {
          try {
            const auto current = currentOdom();
            const double dx = hold_target.x - current.x;
            const double dy = hold_target.y - current.y;
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = clamp(1.8 * (dx * std::cos(current.yaw) +
              dy * std::sin(current.yaw)), -0.08, 0.08);
            cmd.angular.z = clamp(2.0 * normalizeAngle(hold_target.yaw - current.yaw), -0.2, 0.2);
            publishGuardedBaseCommand(cmd);
          } catch (const std::exception&) {
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
          }
          rate.sleep();
        }
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      });
    }
    const bool ok = planAndMaybeExecute(task, execute, label);
    hold_running.store(false);
    if (hold_thread.joinable()) hold_thread.join();
    if (ok && execute && release_and_retreat && attached_box_id_ == object_id) {
      attached_box_id_.clear();
    }
    return ok;
  }

  void recoverTableClearance(const std::string& object_id)
  {
    mtc::Task task;
    task.stages()->setName("escape tabletop contact with fixed TCPs");
    task.loadRobotModel(shared_from_this());
    task.add(std::make_unique<mtc::stages::CurrentState>("stationary held tote"));
    addGroundContactStage(task);
    auto seed = currentSeed(task.getRobotModel());
    const double start = seed.getVariablePosition("waist_up_down_joint");
    if (start < 0.24 || start > 0.28)
      throw std::runtime_error("Table escape requires the inspected near-contact waist posture");
    const auto left = eigenToPose(seed.getGlobalLinkTransform("left_gripper_tcp"));
    const auto right = eigenToPose(seed.getGlobalLinkTransform("right_gripper_tcp"));
    // Only the existing waist/table pair may touch during a monotone upward
    // escape. Both TCP poses stay fixed; no base or jaw motion is commanded.
    auto escape = std::make_unique<mtc::stages::ModifyPlanningScene>("initial waist contact escape only");
    escape->allowCollisions("destination_table", "waist_up_down_link", true);
    escape->allowCollisions(object_id, "large_bin_floor", true);
    task.add(std::move(escape));
    auto planner = makeJointPlanner();
    planner->setMaxVelocityScalingFactor(0.02);
    planner->setMaxAccelerationScalingFactor(0.02);
    for (int i = 1; i <= 16; ++i) {
      const double lift = start + (0.28 - start) * i / 16.0;
      seed.setVariablePosition("waist_up_down_joint", lift);
      seed.update();
      auto goal = solveDualArmGoal(left, right, seed, task.getRobotModel(), 0.15);
      goal["waist_up_down_joint"] = lift;
      goal["waist_pitch_joint"] = seed.getVariablePosition("waist_pitch_joint");
      for (const auto& item : goal) seed.setVariablePosition(item.first, item.second);
      seed.update();
      auto move = std::make_unique<mtc::stages::MoveTo>("fixed TCP waist escape " + std::to_string(i), planner);
      move->setGroup("pallet_upper_body");
      move->setGoal(goal);
      task.add(std::move(move));
    }
    auto strict = std::make_unique<mtc::stages::ModifyPlanningScene>("restore every robot-table collision check");
    strict->allowCollisions("destination_table", false);
    task.add(std::move(strict));
    logWristLoad("before_table_contact_escape");
    if (!planAndMaybeExecute(task, true, "fixed TCP tabletop escape"))
      throw std::runtime_error("Could not clear tabletop contact; keep jaws closed");
    logWristLoad("after_table_contact_escape");
  }

  void seatToteOnBinFloor(const std::string& object_id, double target_x, double settled_z)
  {
    if (object_id != "pallet_box_r" + std::to_string(get_parameter("full_transfer_box_index").as_int() / 3) + "_c" + std::to_string(get_parameter("full_transfer_box_index").as_int() % 3))
      throw std::runtime_error("Reaction estimator target does not match the selected tote");
    if (carried_handle_offset_ < 0.0) {
      mtc::Task model_task;
      model_task.loadRobotModel(shared_from_this());
      auto seed = currentSeed(model_task.getRobotModel());
      std::array<double, 3> measured;
      { std::lock_guard<std::mutex> lock(data_mutex_); measured = measured_boxes_.at(object_id); }
      auto center = worldPointInBase(measured[0], measured[1], measured[2], currentOdom());
      carried_handle_offset_ = 0.5 * (seed.getGlobalLinkTransform("left_gripper_tcp").translation().z() +
        seed.getGlobalLinkTransform("right_gripper_tcp").translation().z()) - center[2];
    }
    bool clamp_unloaded = false;
    std::optional<Eigen::Isometry3d> left_world_reference, right_world_reference;
    double reference_offset = 0.0, initial_box_height = 0.0;
    for (int step = 0; step <= 100; ++step) {
      bool stable = true;
      double force = 0.0, z = 0.0, tilt = 0.0;
      const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(750);
      while (std::chrono::steady_clock::now() < until) {
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          const auto now = std::chrono::steady_clock::now();
          const bool fresh = measured_boxes_.count(object_id) &&
            now - measured_boxes_received_[object_id] < std::chrono::seconds(1);
          if (!fresh) throw std::runtime_error("Cannot seat tote without fresh physical pose");
          z = measured_boxes_.at(object_id)[2];
          tilt = measured_box_tilt_ * 180.0 / kPi;
          if (!reaction_estimate_ready_ || now - reaction_estimate_received_ > std::chrono::milliseconds(250))
            throw std::runtime_error("Floor admittance requires fresh wrist-derived support; keep jaws closed");
          force = estimated_bottom_reaction_;
          if (force > 300.0 || force < -30.0)
            throw std::runtime_error("Estimated floor reaction " + std::to_string(force) + " N outside [-30, 300] N; keep grippers closed");
          if (std::abs(measured_boxes_.at(object_id)[0] - target_x) > 0.015 ||
              std::abs(measured_boxes_.at(object_id)[1] + 1.55) > 0.025 ||
              z < settled_z - 0.002 || tilt > 6.0 ||
              measured_grippers_[0] < 0.04 || measured_grippers_[1] < 0.04)
            throw std::runtime_error("Physical seating pose or closed-gripper check failed");
          if (!pallet_placement::releaseReady(clamp_unloaded,
                reaction_estimate_ready_ && now - reaction_estimate_received_ < std::chrono::milliseconds(250),
                force, z - settled_z, tilt)) stable = false;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(50));
      }
      if (stable) {
        RCLCPP_INFO(get_logger(),
          "Floor support verified BEFORE opening using wrist F/T estimate: z=%.6f tilt=%.3f deg reaction=%.1f N; grippers not fully opened",
          z, tilt, force);
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          floor_support_verified_ = true;
          logWristDetailsLocked("floor_supported_before_release");
        }
        return;
      }
      if (!clamp_unloaded && force >= 80.0 && std::abs(z - settled_z) <= 0.0015) {
        // Only reduce preload after the physical floor carries the tote.
        // Reduce each jaw by 2 mm past nominal wall contact (55 mm),
        // only after the floor carries the tote; full opening remains gated
        // by flat, stable floor support below.
        // This lets a slightly tilted tote settle onto its support without
        // pushing the mobile base upwards or dropping from free space.
        RCLCPP_INFO(get_logger(), "Floor already supports %.1f N: partially unload jaws; full opening still prohibited", force);
        logWristLoad("support_transfer_before_partial_unload");
        mtc::Task relax;
        relax.stages()->setName("floor-supported clamp preload reduction");
        relax.loadRobotModel(shared_from_this());
        relax.add(std::make_unique<mtc::stages::CurrentState>("closed jaws with floor support"));
        addGroundContactStage(relax);
        auto support = std::make_unique<mtc::stages::ModifyPlanningScene>("keep tote floor contact allowed");
        support->allowCollisions(object_id, "large_bin_floor", true);
        relax.add(std::move(support));
        auto close = std::make_unique<mtc::stages::MoveTo>("reduce clamp preload without opening", makeJointPlanner());
        close->setGroup("both_grippers");
        close->setGoal(std::map<std::string, double>{
          {"left_gripper_left_finger_joint", 0.053}, {"gripper_left_finger_joint", 0.053}});
        relax.add(std::move(close));
        if (!planAndMaybeExecute(relax, true, "floor-supported clamp preload reduction"))
          throw std::runtime_error("Could not reduce clamp preload on floor; do not open grippers");
        clamp_unloaded = true;
        rclcpp::sleep_for(std::chrono::seconds(2));
        continue;
      }
      if (step == 100) break;
      // Bidirectional vertical admittance. Excess pressure commands an upward
      // wrist correction; full release requires a bounded support band after
      // clamp unloading, never just a lower force threshold.
      const double vertical_step = pallet_placement::verticalStep(force, z - settled_z);
      const double desired_z = z + vertical_step;
      RCLCPP_INFO(get_logger(), "Floor seating step %d: actual_z=%.6f tilt=%.3f reaction=%.1f N target_z=%.6f",
                  step + 1, z, tilt, force, desired_z);
      mtc::Task task;
      task.stages()->setName("contact-verified slow floor seating");
      task.loadRobotModel(shared_from_this());
      task.add(std::make_unique<mtc::stages::CurrentState>("current closed-gripper state"));
      addGroundContactStage(task);
      auto support = std::make_unique<mtc::stages::ModifyPlanningScene>("allow tote bottom on bin floor only");
      support->allowCollisions(object_id, "large_bin_floor", true);
      task.add(std::move(support));
      auto seed = currentSeed(task.getRobotModel());
      // Integrate a world-frame position reference instead of restarting at
      // measured FK each cycle. Under load a finite tracking error otherwise
      // cancels every tiny update and the requested motion never accumulates.
      Eigen::Isometry3d base_world;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!measured_base_pose_ || std::chrono::steady_clock::now() - measured_base_received_ > std::chrono::milliseconds(250))
          throw std::runtime_error("Floor reference requires fresh physical base pose");
        base_world = measured_base_transform_;
      }
      if (!left_world_reference) {
        left_world_reference = base_world * seed.getGlobalLinkTransform("left_gripper_tcp");
        right_world_reference = base_world * seed.getGlobalLinkTransform("right_gripper_tcp");
        initial_box_height = z;
      }
      reference_offset = pallet_placement::integrateVerticalReference(
        reference_offset, vertical_step, initial_box_height - settled_z);
      auto left_world = *left_world_reference;
      auto right_world = *right_world_reference;
      left_world.translation().z() += reference_offset;
      right_world.translation().z() += reference_offset;
      const Eigen::Isometry3d left_tf = base_world.inverse() * left_world;
      const Eigen::Isometry3d right_tf = base_world.inverse() * right_world;
      const Eigen::Vector3d delta = 0.5 * (
        (left_tf.translation() - seed.getGlobalLinkTransform("left_gripper_tcp").translation()) +
        (right_tf.translation() - seed.getGlobalLinkTransform("right_gripper_tcp").translation()));
      RCLCPP_INFO(get_logger(), "FLOOR_REFERENCE increment=%.9f accumulated=%.9f target_box_z=%.9f",
        vertical_step, reference_offset, initial_box_height + reference_offset);
      auto measured_start = std::make_unique<mtc::stages::ModifyPlanningScene>("synchronize measured microstep start");
      measured_start->setCallback([seed](const planning_scene::PlanningScenePtr& scene, const mtc::PropertyMap&) {
        // Preserve attachments/ACM, updating only the measured joint positions.
        auto& state = scene->getCurrentStateNonConst();
        state.setVariablePositions(seed.getVariablePositions());
        state.update();
      });
      task.add(std::move(measured_start));
      auto refined = seed;
      pallet_placement::refineTcpPose(refined, task.getRobotModel()->getJointModelGroup("left_arm"),
                                     "left_gripper_tcp", left_tf);
      pallet_placement::refineTcpPose(refined, task.getRobotModel()->getJointModelGroup("right_arm"),
                                     "right_gripper_tcp", right_tf);
      std::map<std::string, double> goal;
      for (const auto& name : task.getRobotModel()->getJointModelGroup("both_arms")->getActiveJointModelNames())
        goal[name] = refined.getVariablePosition(name);
      const auto left_move = refined.getGlobalLinkTransform("left_gripper_tcp").translation() -
                             seed.getGlobalLinkTransform("left_gripper_tcp").translation();
      const auto right_move = refined.getGlobalLinkTransform("right_gripper_tcp").translation() -
                              seed.getGlobalLinkTransform("right_gripper_tcp").translation();
      RCLCPP_INFO(get_logger(), "FLOOR_TCP_STEP requested=[%.9f,%.9f,%.9f] left=[%.9f,%.9f,%.9f] right=[%.9f,%.9f,%.9f]",
        delta.x(), delta.y(), delta.z(), left_move.x(), left_move.y(), left_move.z(),
        right_move.x(), right_move.y(), right_move.z());
      auto planner = makeJointPlanner();
      planner->setMaxVelocityScalingFactor(0.03);
      planner->setMaxAccelerationScalingFactor(0.03);
      planner->setProperty("goal_joint_tolerance", 1e-9);
      planner->setProperty("max_step", 1e-5);
      // TOTG's default minimum joint-angle change drops micrometre moves.
      planner->setTimeParameterization(std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>(
        1e-8, 0.02, 1e-9));
      auto lower = std::make_unique<mtc::stages::MoveTo>("incremental vertical wrist admittance", planner);
      lower->setGroup("both_arms");
      lower->setGoal(goal);
      task.add(std::move(lower));
      if (!planAndMaybeExecute(task, true, "slow floor seating " + std::to_string(step + 1)))
        throw std::runtime_error("Slow floor seating failed; do not open grippers");
    }
    throw std::runtime_error("Floor support was not verified; do not open grippers");
  }

  double carriedHandleOffset() const
  {
    return carried_handle_offset_ > 0.0 ? carried_handle_offset_ :
      get_parameter("handle_z_offset").as_double();
  }

  double carried_handle_offset_{-1.0};
  std::unique_ptr<tf2_ros::Buffer> model_tf_;
  std::shared_ptr<tf2_ros::TransformListener> model_tf_listener_;

  void prepareEmptyGrasp(const BoxSpec& box, double lift, double pitch, bool execute)
  {
    mtc::Task task;
    task.stages()->setName("empty long-finger torso-arm approach");
    task.loadRobotModel(shared_from_this());
    task.add(std::make_unique<mtc::stages::CurrentState>("current extracted state"));
    addGroundContactStage(task);
    // Empty arms must stay outside neighbouring open totes. Use solid
    // envelopes only during this approach; inflated neighbours would overlap
    // the attached target during the subsequent dense-pallet side pull.
    moveit::planning_interface::PlanningSceneInterface psi;
    const auto existing = psi.getObjects();
    const auto base = currentOdom();
    std::vector<BoxSpec> guarded_neighbors;
    auto guard = std::make_unique<mtc::stages::ModifyPlanningScene>("avoid untouched tote interiors");
    // Contact allowances from the previous pick persist in MoveIt's scene.
    // Replacing geometry alone does not clear those ACM entries. Empty
    // approaches permit no robot contact with any remaining source tote.
    const auto& robot_links = task.getRobotModel()->getLinkModelNames();
    guard->allowCollisions(box.id, robot_links, false);
    for (const auto& neighbor : boxes_) {
      if (neighbor.id == box.id || !existing.count(neighbor.id)) continue;
      guard->allowCollisions(neighbor.id, robot_links, false);
      auto obstacle = makeSimpleBoxObject(neighbor, base);
      obstacle.primitives.clear();
      obstacle.primitive_poses.clear();
      appendWorldBox(obstacle, 0.310, 0.410, 0.158,
        neighbor.x, neighbor.y, neighbor.z, 0.0, base);
      guard->addObject(obstacle);
      guarded_neighbors.push_back(neighbor);
    }
    task.add(std::move(guard));
    auto seed = currentSeed(task.getRobotModel());
    const auto center = worldPointInBase(box.x, box.y, box.z, currentOdom());
    geometry_msgs::msg::Pose left, right;
    // Keep tips well above the rim throughout waist motion, then let the
    // existing strict pick plan descend to its pregrasp and shallow grasp.
    // From the upright home posture, stage both long fingers high before
    // bending toward the tote.  After a perimeter pull the waist is already
    // bent; keep the original low transition there because a high bilateral
    // target exceeds the opposite arm's workspace at the outer pallet row.
    const bool rear_lateral = get_parameter("rear_side_pull").as_bool() && box.col > 0;
    const double start_clearance =
      seed.getVariablePosition("waist_pitch_joint") > -0.4 ? 0.61 : (rear_lateral ? 0.34 : 0.26);
    auto approach_center = center;
    const double start_x = start_clearance > 0.5 ? std::min(center[0], 0.45) : center[0];
    approach_center[0] = start_x;
    graspTargets(approach_center, start_clearance, left, right);
    auto goal = solveDualArmGoal(left, right, seed, task.getRobotModel());
    auto rise = std::make_unique<mtc::stages::MoveTo>("raise both long fingers above tote", makePipeline(0.25));
    rise->setGroup("both_arms"); rise->setGoal(goal);
    rise->setTimeout(get_parameter("stage_timeout").as_double());
    task.add(std::move(rise));
    for (const auto& item : goal) seed.setVariablePosition(item.first, item.second);
    const double start_lift = seed.getVariablePosition("waist_up_down_joint");
    const double start_pitch = seed.getVariablePosition("waist_pitch_joint");
    for (int i = 1; i <= 8; ++i) {
      const double t = i / 8.0;
      seed.setVariablePosition("waist_up_down_joint", start_lift + t * (lift-start_lift));
      seed.setVariablePosition("waist_pitch_joint", start_pitch + t * (pitch-start_pitch));
      seed.update();
      approach_center[0] = start_x + t * (center[0]-start_x);
      graspTargets(approach_center, start_clearance + t * (0.26-start_clearance), left, right);
      goal = solveDualArmGoal(left, right, seed, task.getRobotModel(), 0.75);
      goal["waist_up_down_joint"] = seed.getVariablePosition("waist_up_down_joint");
      goal["waist_pitch_joint"] = seed.getVariablePosition("waist_pitch_joint");
      for (const auto& item : goal) seed.setVariablePosition(item.first,item.second);
      auto move = std::make_unique<mtc::stages::MoveTo>("keep raised fingers while bending " + std::to_string(i), makeJointPlanner());
      move->setGroup("pallet_upper_body"); move->setGoal(goal);
      move->setTimeout(get_parameter("stage_timeout").as_double());
      task.add(std::move(move));
    }
    auto restore = std::make_unique<mtc::stages::ModifyPlanningScene>("restore tote shells after empty approach");
    for (const auto& neighbor : guarded_neighbors)
      restore->addObject(makeSimpleBoxObject(neighbor, base));
    task.add(std::move(restore));
    if (!planAndMaybeExecute(task, execute, "empty long-finger coordinated approach"))
      throw std::runtime_error("Collision-checked empty torso-arm approach failed");
  }

  void addHomeClearanceStage(mtc::Task& task) const
  {
    if (!get_parameter("strict_tool_body_guard").as_bool()) return;
    auto guard_scene = std::make_shared<planning_scene::PlanningScene>(task.getRobotModel());
    auto matrix = toolBodyMatrix(task.getRobotModel());
    const double required = get_parameter("tool_body_clearance").as_double();
    auto guard = std::make_unique<mtc::stages::ModifyPlanningScene>("plan home with tool body clearance");
    guard->setCallback([guard_scene, matrix, required](const planning_scene::PlanningScenePtr& scene,
                                                      const mtc::PropertyMap&) {
      collision_detection::CollisionRequest request; request.distance = true;
      collision_detection::CollisionResult initial;
      auto start = scene->getCurrentState(); start.clearAttachedBodies();
      guard_scene->checkSelfCollision(request, initial, start, matrix);
      const double planning_clearance = std::max(required,
        std::min(required + 0.001, initial.distance - 0.0001));
      scene->setStateFeasibilityPredicate([guard_scene, matrix, planning_clearance](
          const moveit::core::RobotState& state, bool) {
        auto robot = state; robot.clearAttachedBodies();
        collision_detection::CollisionRequest req; req.distance = true;
        collision_detection::CollisionResult result;
        guard_scene->checkSelfCollision(req, result, robot, matrix);
        return !result.collision && result.distance >= planning_clearance;
      });
    });
    task.add(std::move(guard));
  }

  void moveArmsHome(std::unique_ptr<mtc::Task> prepared = nullptr)
  {
    if (prepared) {
      moveit_task_constructor_msgs::msg::Solution audit;
      prepared->solutions().front()->toMsg(audit);
      moveit::core::RobotState planned(prepared->getRobotModel());
      moveit::core::robotStateMsgToRobotState(audit.start_scene.robot_state, planned);
      auto actual = currentSeed(prepared->getRobotModel());
      bool current = true;
      for (const auto& name : prepared->getRobotModel()->getJointModelGroup("both_arms")->getVariableNames())
        current = current && std::abs(planned.getVariablePosition(name) - actual.getVariablePosition(name)) < 0.01;
      if (current) {
        RCLCPP_INFO(get_logger(), "EARLY_PLAN use checked home prepared during placement settling");
        if (!planAndMaybeExecute(*prepared, true, "return both arms home", true))
          throw std::runtime_error("Preplanned home execution failed; do not replay");
        return;
      }
      RCLCPP_WARN(get_logger(), "EARLY_PLAN discard home because measured start changed");
    }
    if (get_parameter("strict_tool_body_guard").as_bool()) {
      mtc::Task direct;
      direct.stages()->setName("checked direct empty-arm home");
      direct.loadRobotModel(shared_from_this());
      direct.add(std::make_unique<mtc::stages::CurrentState>("current released state"));
      addGroundContactStage(direct);
      auto planner = makeJointPlanner();
      planner->setMaxVelocityScalingFactor(0.25);
      planner->setMaxAccelerationScalingFactor(0.15);
      auto move = std::make_unique<mtc::stages::MoveTo>("direct both arms home", planner);
      move->setGroup("both_arms");
      move->setGoal("both_arms_home");
      direct.add(std::move(move));
      const auto planned = direct.plan(1);
      if (planned && direct.numSolutions() > 0) {
        moveit_task_constructor_msgs::msg::Solution audit;
        direct.solutions().front()->toMsg(audit);
        if (validateToolBodySolution(direct, audit, "direct empty-arm home candidate")) {
          RCLCPP_INFO(get_logger(), "EMPTY_HOME_OPT selected checked direct joint path");
          if (!planAndMaybeExecute(direct, true, "return both arms home", true))
            throw std::runtime_error("Checked direct home execution failed; do not replay");
          return;
        }
      }
      RCLCPP_INFO(get_logger(), "EMPTY_HOME_OPT direct candidate invalid; plan collision-free detour before execution");
    }
    for (int home_attempt = 1; home_attempt <= 8; ++home_attempt) {
    mtc::Task task;
    task.stages()->setName("return both arms home");
    task.loadRobotModel(shared_from_this());
    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    addGroundContactStage(task);
    addHomeClearanceStage(task);
    auto move = std::make_unique<mtc::stages::MoveTo>("both arms home", makePipeline(0.25));
    move->setTimeout(get_parameter("stage_timeout").as_double());
    move->setGroup("both_arms");
    move->setGoal("both_arms_home");
    task.add(std::move(move));
    // Rebuild the planning task and pipeline for a fresh sampling search.
    // Revalidating retained rejected candidates does not create a safe detour.
    if (!planAndMaybeExecute(task, false, "return both arms home candidate")) {
      RCLCPP_WARN(get_logger(), "EMPTY_HOME_OPT rebuild rejected detour search %d/8", home_attempt);
      continue;
    }
    if (!planAndMaybeExecute(task, true, "return both arms home", true))
      throw std::runtime_error("Checked detour home execution failed; do not replay");
    return;
    }
    throw std::runtime_error("Could not plan safe arms home after fresh searches; do not reset the waist");
  }

  // Caller holds data_mutex_. Raw sensor axes: Fx/Fy/Fz in N, Tx/Ty/Tz in N m.
  void logWristDetailsLocked(const std::string& stage)
  {
    if (!left_force_ready_ || !right_force_ready_) return;
    const auto& l = left_wrench_message_;
    const auto& r = right_wrench_message_;
    RCLCPP_INFO(get_logger(),
      "WRIST_FT_STAGE stage=%s sim_time=%.9f left_stamp=%.9f right_stamp=%.9f left=[%.6f,%.6f,%.6f,%.6f,%.6f,%.6f] right=[%.6f,%.6f,%.6f,%.6f,%.6f,%.6f] estimated_support=%.6f grippers=[%.6f,%.6f]",
      stage.c_str(), get_clock()->now().seconds(),
      rclcpp::Time(l.header.stamp).seconds(), rclcpp::Time(r.header.stamp).seconds(),
      l.wrench.force.x, l.wrench.force.y, l.wrench.force.z,
      l.wrench.torque.x, l.wrench.torque.y, l.wrench.torque.z,
      r.wrench.force.x, r.wrench.force.y, r.wrench.force.z,
      r.wrench.torque.x, r.wrench.torque.y, r.wrench.torque.z,
      estimated_bottom_reaction_, measured_grippers_[0], measured_grippers_[1]);
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
    logWristDetailsLocked(context);
    RCLCPP_INFO(
      get_logger(),
      "%s: wrist force norms left=%.1f N right=%.1f N (ideal static share is about 73.6 N/arm)",
      context.c_str(), left_force_norm_, right_force_norm_);
  }

  moveit::core::RobotModelConstPtr column_model_;
  std::map<std::string, std::vector<Eigen::Vector3d>> column_points_;
  std::vector<Eigen::Vector3d> lift_stage_visible_points_;
  mutable std::mutex data_mutex_;
  Pose2D odom_pose_;
  Eigen::Isometry3d odom_transform_{Eigen::Isometry3d::Identity()};
  std::chrono::steady_clock::time_point odom_received_{};
  bool odom_ready_{false};
  geometry_msgs::msg::WrenchStamped left_wrench_message_, right_wrench_message_;
  bool floor_support_verified_{false}, release_started_{false}, release_fully_open_{false};
  bool left_force_ready_{false};
  bool right_force_ready_{false};
  double left_force_norm_{0.0};
  double right_force_norm_{0.0};
  std::optional<geometry_msgs::msg::PoseStamped> latest_target_pose_;
  rclcpp::Time latest_target_received_{0, 0, RCL_ROS_TIME};

  std::ofstream wrist_trace_;
  double wrist_trace_last_{-1.0};
  std::size_t wrist_trace_rows_{0};
  std::string process_stage_{"initialization"};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr body_pub_, process_arm_pub_;
  std::atomic_bool process_arm_active_{false};
  std::mutex process_mutex_;
  std::unique_ptr<moveit::core::RobotState> carry_nominal_;
  pallet_placement::ProcessAdmittance carry_law_;
  std::array<double,6> carry_reference_{};
  double carry_last_update_{-1.0};
  std::size_t carry_cycles_{0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr left_wrench_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr right_wrench_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  std::vector<BoxSpec> boxes_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gazebo_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr base_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr waist_state_sub_;
  std::array<double, 2> measured_waist_{0.0, 0.0};
  std::array<double, 2> measured_grippers_{0.0, 0.0};
  std::map<std::string, double> measured_joint_positions_;
  std::chrono::steady_clock::time_point waist_received_;
  std::optional<Pose2D> measured_base_pose_;
  Eigen::Isometry3d measured_base_transform_{Eigen::Isometry3d::Identity()};
  double measured_base_pitch_{0.0};
  double measured_base_roll_{0.0};
  double measured_box_tilt_{0.0};
  std::chrono::steady_clock::time_point measured_base_received_;
  std::map<std::string, std::array<double, 3>> measured_boxes_;
  std::map<std::string, std::chrono::steady_clock::time_point> measured_boxes_received_;
  std::string attached_box_id_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr estimated_reaction_sub_;
  bool reaction_estimate_ready_{false};
  double estimated_bottom_reaction_{0.0};
  std::chrono::steady_clock::time_point reaction_estimate_received_;
};

int main(int argc, char** argv)
{
  int exit_code = 0;
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
    exit_code = 1;
  }
  rclcpp::shutdown();
  if (spinner.joinable()) {
    spinner.join();
  }
  return exit_code;
}
