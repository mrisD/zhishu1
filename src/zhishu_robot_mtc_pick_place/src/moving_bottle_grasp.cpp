#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

namespace
{
struct Observation
{
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::TwistStamped twist;
  bool have_pose{false};
  bool have_twist{false};
};

geometry_msgs::msg::Quaternion quaternionFromRpy(double roll, double pitch, double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  quaternion.normalize();
  geometry_msgs::msg::Quaternion result;
  result.x = quaternion.x();
  result.y = quaternion.y();
  result.z = quaternion.z();
  result.w = quaternion.w();
  return result;
}

bool setBoundedStartState(MoveGroupInterface& group)
{
  auto current_state = group.getCurrentState(1.0);
  if (!current_state) {
    return false;
  }
  // Gazebo can report joints a few picometres / radians outside a hard zero
  // bound.  MoveIt rejects that numerically noisy state before planning, so
  // clamp it to the model bounds before every request.
  current_state->enforceBounds();
  group.setStartState(*current_state);
  return true;
}

moveit_msgs::msg::CollisionObject box(
  const std::string& id, const std::string& frame, const std::vector<double>& dimensions,
  const std::vector<double>& xyz)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = frame;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = dimensions.at(0);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = dimensions.at(1);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = dimensions.at(2);
  geometry_msgs::msg::Pose pose;
  pose.position.x = xyz.at(0);
  pose.position.y = xyz.at(1);
  pose.position.z = xyz.at(2);
  pose.orientation.w = 1.0;
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

bool executeNamed(MoveGroupInterface& group, const std::string& target)
{
  if (!setBoundedStartState(group)) {
    return false;
  }
  if (!group.setNamedTarget(target)) {
    return false;
  }
  MoveGroupInterface::Plan plan;
  const auto planned = group.plan(plan);
  return static_cast<bool>(planned) && static_cast<bool>(group.execute(plan));
}

bool planPoseWithOrientationSearch(
  MoveGroupInterface& arm, geometry_msgs::msg::PoseStamped target,
  double roll, double pitch, double yaw_offset, const rclcpp::Logger& logger)
{
  constexpr double kPi = 3.14159265358979323846;
  const std::vector<double> pitch_offsets{0.0, kPi / 2.0, -kPi / 2.0};
  for (double pitch_offset : pitch_offsets) {
    for (int index = 0; index < 8; ++index) {
      target.pose.orientation = quaternionFromRpy(
        roll, pitch + pitch_offset, yaw_offset + index * kPi / 4.0);
      if (!setBoundedStartState(arm)) {
        return false;
      }
      arm.setPoseTarget(target);
      MoveGroupInterface::Plan plan;
      const auto result = arm.plan(plan);
      if (static_cast<bool>(result)) {
        RCLCPP_INFO(logger, "Found interception IK orientation candidate %d", index);
        return static_cast<bool>(arm.execute(plan));
      }
      arm.clearPoseTargets();
    }
  }
  return false;
}

template<typename T>
T parameterOr(rclcpp::Node& node, const std::string& name, const T& fallback)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, fallback);
  }
  T value{};
  if (!node.get_parameter(name, value)) {
    return fallback;
  }
  return value;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("moving_bottle_grasp", options);

  const auto world_frame = parameterOr(*node, "world_frame", std::string("base_link"));
  const auto arm_group = parameterOr(*node, "arm_group", std::string("right_arm"));
  const auto gripper_group = parameterOr(*node, "gripper_group", std::string("right_gripper"));
  const auto tcp_link = parameterOr(*node, "tcp_link", std::string("right_gripper_tcp"));
  const auto open_target = parameterOr(*node, "open_target", std::string("right_gripper_opened"));
  const auto closed_target = parameterOr(*node, "closed_target", std::string("right_gripper_closed"));
  const double intercept_x = parameterOr(*node, "intercept_x", 0.62);
  const double intercept_y = parameterOr(*node, "intercept_y", 0.0);
  const double grasp_z_offset = parameterOr(*node, "grasp_z_offset", 0.025);
  // The TCP is at y=0, but the open fingers contact the bottle centre before
  // it reaches the TCP.  Keep arm placement and contact triggering separate.
  const double trigger_y = parameterOr(*node, "trigger_y", -0.19);
  const double trigger_lead = parameterOr(*node, "trigger_lead", 0.035);
  const double trigger_tolerance = parameterOr(*node, "trigger_tolerance", 0.035);
  const double minimum_forward_velocity =
    parameterOr(*node, "minimum_forward_velocity", -0.005);
  const double lift_distance = parameterOr(*node, "lift_distance", 0.14);
  const double pose_timeout = parameterOr(*node, "pose_timeout", 20.0);
  const double intercept_timeout = parameterOr(*node, "intercept_timeout", 60.0);
  const double orientation_roll = parameterOr(*node, "orientation_roll", 3.14159265358979323846);
  const double orientation_pitch = parameterOr(*node, "orientation_pitch", 0.0);
  const double orientation_yaw = parameterOr(*node, "orientation_yaw", 0.0);

  std::mutex observation_mutex;
  std::condition_variable observation_changed;
  Observation observation;
  auto pose_subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/bottle_grasp/grasp_pose", 10,
    [&](const geometry_msgs::msg::PoseStamped& message) {
      std::lock_guard<std::mutex> lock(observation_mutex);
      observation.pose = message;
      observation.have_pose = true;
      observation_changed.notify_all();
    });
  auto twist_subscription = node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/bottle_grasp/tracked_twist", 10,
    [&](const geometry_msgs::msg::TwistStamped& message) {
      std::lock_guard<std::mutex> lock(observation_mutex);
      observation.twist = message;
      observation.have_twist = true;
      observation_changed.notify_all();
    });
  (void)pose_subscription;
  (void)twist_subscription;

  auto attach_publisher = node->create_publisher<std_msgs::msg::Empty>("/bottle_grasp/attach", 10);
  auto status_publisher = node->create_publisher<std_msgs::msg::String>("/bottle_grasp/status", 10);
  auto motion_client = node->create_client<std_srvs::srv::SetBool>("/moving_bottle/set_motion");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  MoveGroupInterface arm(node, arm_group);
  MoveGroupInterface gripper(node, gripper_group);
  arm.setEndEffectorLink(tcp_link);
  arm.setPoseReferenceFrame(world_frame);
  arm.setPlanningTime(4.0);
  arm.setNumPlanningAttempts(8);
  arm.setMaxVelocityScalingFactor(0.35);
  arm.setMaxAccelerationScalingFactor(0.30);
  gripper.setPlanningTime(2.0);
  gripper.setMaxVelocityScalingFactor(0.65);
  gripper.setMaxAccelerationScalingFactor(0.50);

  moveit::planning_interface::PlanningSceneInterface planning_scene;
  planning_scene.applyCollisionObjects({
    box("conveyor_table", world_frame, {0.78, 0.86, 0.10}, {0.66, 0.0, 1.05}),
    box("obstacle_box", world_frame, {0.12, 0.16, 0.30}, {0.48, 0.24, 1.25}),
  });

  std_msgs::msg::String status;
  status.data = "waiting_for_rgbd_track";
  status_publisher->publish(status);
  geometry_msgs::msg::PoseStamped tracked_pose;
  {
    std::unique_lock<std::mutex> lock(observation_mutex);
    const bool ready = observation_changed.wait_for(
      lock, std::chrono::duration<double>(pose_timeout),
      [&observation]() { return observation.have_pose && observation.have_twist; });
    if (!ready) {
      RCLCPP_ERROR(node->get_logger(), "Timed out waiting for RGB-D bottle track");
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 2;
    }
    tracked_pose = observation.pose;
  }

  if (!executeNamed(gripper, open_target)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to open gripper");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 3;
  }

  geometry_msgs::msg::PoseStamped interception;
  interception.header.frame_id = world_frame;
  interception.header.stamp = node->now();
  interception.pose.position.x = intercept_x;
  interception.pose.position.y = intercept_y;
  // The perception pose describes the desired bottle contact band.  The TCP
  // sits above the finger pads, so compensate the calibrated pad-to-TCP
  // vertical offset while retaining depth-derived bottle height.
  interception.pose.position.z = tracked_pose.pose.position.z - grasp_z_offset;
  status.data = "planning_interception_pose";
  status_publisher->publish(status);
  if (!planPoseWithOrientationSearch(
      arm, interception, orientation_roll, orientation_pitch, orientation_yaw,
      node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "No executable IK solution for interception pose");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 4;
  }

  status.data = "armed_waiting_for_moving_bottle";
  status_publisher->publish(status);
  bool crossed_interception_window = false;
  {
    std::unique_lock<std::mutex> lock(observation_mutex);
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(intercept_timeout);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      observation_changed.wait_for(lock, 50ms);
      if (!observation.have_pose || !observation.have_twist) {
        continue;
      }
      const double y = observation.pose.pose.position.y;
      const double velocity_y = observation.twist.twist.linear.y;
      if (velocity_y > minimum_forward_velocity && y >= trigger_y - trigger_lead &&
          y <= trigger_y + trigger_tolerance) {
        tracked_pose = observation.pose;
        crossed_interception_window = true;
        break;
      }
    }
    if (!crossed_interception_window) {
      RCLCPP_ERROR(node->get_logger(), "Moving bottle did not cross interception window");
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 5;
    }
  }

  status.data = "closing_on_moving_bottle";
  status_publisher->publish(status);
  // Freeze the bottle at first finger contact, then close synchronously.  A
  // previous async gripper move overlapped the following arm request and could
  // race MoveIt's trajectory manager during cancellation.
  if (motion_client->wait_for_service(1s)) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    motion_client->async_send_request(request);
  } else {
    RCLCPP_WARN(node->get_logger(), "Moving bottle stop service unavailable");
  }
  std::this_thread::sleep_for(150ms);
  if (!executeNamed(gripper, closed_target)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to close gripper on bottle");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 6;
  }
  attach_publisher->publish(std_msgs::msg::Empty());
  std::this_thread::sleep_for(300ms);

  auto lift = arm.getCurrentPose(tcp_link);
  lift.pose.position.z += lift_distance;
  if (!setBoundedStartState(arm)) {
    RCLCPP_ERROR(node->get_logger(), "Unable to read bounded arm start state");
    status.data = "failed_lift_start_state";
    status_publisher->publish(status);
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 6;
  }
  arm.setPoseTarget(lift);
  MoveGroupInterface::Plan lift_plan;
  if (!static_cast<bool>(arm.plan(lift_plan)) || !static_cast<bool>(arm.execute(lift_plan))) {
    RCLCPP_ERROR(node->get_logger(), "Bottle closure occurred but lift failed");
    status.data = "failed_lift";
    status_publisher->publish(status);
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 6;
  }

  status.data = "grasp_complete";
  status_publisher->publish(status);
  RCLCPP_INFO(node->get_logger(), "Moving bottle grasp and lift completed");
  std::this_thread::sleep_for(1s);
  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
