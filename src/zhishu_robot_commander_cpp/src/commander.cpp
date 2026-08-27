#include "zhishu_robot_commander_cpp/commander.hpp"

#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>

namespace zhishu_robot_commander_cpp
{
namespace
{
constexpr std::size_t kSingleArmJointCount = 7;
constexpr std::size_t kDualArmJointCount = 14;
constexpr double kGripperMinimum = 0.0;
// Prismatic master-finger limits from zhishu_robot_description/urdf/gripper.xacro (m).
constexpr double kGripperMaximum = 0.06;

constexpr char kLeftGripperJoint[] = "left_gripper_left_finger_joint";
constexpr char kRightGripperJoint[] = "gripper_left_finger_joint";

bool succeeded(const moveit::core::MoveItErrorCode & code)
{
  return code == moveit::core::MoveItErrorCode::SUCCESS;
}
}  // namespace

Commander::Commander(const rclcpp::Node::SharedPtr & node)
: Commander(node, Options{})
{
}

Commander::Commander(const rclcpp::Node::SharedPtr & node, const Options & options)
: node_(node), options_(options)
{
  if (!node_) {
    throw std::invalid_argument("Commander requires a valid rclcpp node");
  }

  left_arm_ = std::make_unique<MoveGroupInterface>(node_, options_.left_arm_group);
  right_arm_ = std::make_unique<MoveGroupInterface>(node_, options_.right_arm_group);
  both_arms_ = std::make_unique<MoveGroupInterface>(node_, options_.both_arms_group);
  left_gripper_ = std::make_unique<MoveGroupInterface>(node_, options_.left_gripper_group);
  right_gripper_ = std::make_unique<MoveGroupInterface>(node_, options_.right_gripper_group);
  both_grippers_ = std::make_unique<MoveGroupInterface>(node_, options_.both_grippers_group);

  configureMoveGroup(*left_arm_);
  configureMoveGroup(*right_arm_);
  configureMoveGroup(*both_arms_);
  configureMoveGroup(*left_gripper_);
  configureMoveGroup(*right_gripper_);
  configureMoveGroup(*both_grippers_);

  left_arm_->startStateMonitor();
  right_arm_->startStateMonitor();
  both_arms_->startStateMonitor();
  left_gripper_->startStateMonitor();
  right_gripper_->startStateMonitor();
  both_grippers_->startStateMonitor();

  RCLCPP_INFO(node_->get_logger(), "Zhishu robot commander SDK initialized");
}

void Commander::configureMoveGroup(MoveGroupInterface & group)
{
  group.setMaxVelocityScalingFactor(options_.velocity_scaling);
  group.setMaxAccelerationScalingFactor(options_.acceleration_scaling);
  group.setPlanningTime(options_.planning_time);
  group.setNumPlanningAttempts(options_.planning_attempts);
}

bool Commander::moveLeftArmNamed(const std::string & target_name)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return moveNamed(*left_arm_, target_name);
}

bool Commander::moveRightArmNamed(const std::string & target_name)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return moveNamed(*right_arm_, target_name);
}

bool Commander::moveBothArmsNamed(const std::string & target_name)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return moveNamed(*both_arms_, target_name);
}

bool Commander::moveLeftArmJoints(const std::vector<double> & joints)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return moveJoints(*left_arm_, joints, kSingleArmJointCount, "left arm");
}

bool Commander::moveRightArmJoints(const std::vector<double> & joints)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return moveJoints(*right_arm_, joints, kSingleArmJointCount, "right arm");
}

bool Commander::moveBothArmsJoints(const std::vector<double> & joints)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return moveJoints(*both_arms_, joints, kDualArmJointCount, "both arms");
}

bool Commander::moveLeftArmPose(
  const geometry_msgs::msg::PoseStamped & target,
  bool cartesian,
  const std::string & end_effector_link)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  const auto & link = end_effector_link.empty() ? options_.left_end_effector_link : end_effector_link;
  return movePose(*left_arm_, target, cartesian, link, "left arm");
}

bool Commander::moveRightArmPose(
  const geometry_msgs::msg::PoseStamped & target,
  bool cartesian,
  const std::string & end_effector_link)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  const auto & link = end_effector_link.empty() ? options_.right_end_effector_link : end_effector_link;
  return movePose(*right_arm_, target, cartesian, link, "right arm");
}

bool Commander::moveBothArmsPoses(
  const geometry_msgs::msg::PoseStamped & left_target,
  const geometry_msgs::msg::PoseStamped & right_target,
  const std::string & left_end_effector_link,
  const std::string & right_end_effector_link)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  const auto left_link = left_end_effector_link.empty() ?
    options_.left_end_effector_link : left_end_effector_link;
  const auto right_link = right_end_effector_link.empty() ?
    options_.right_end_effector_link : right_end_effector_link;

  both_arms_->setStartStateToCurrentState();
  both_arms_->clearPoseTargets();
  if (!both_arms_->setPoseTarget(normalizedPose(left_target), left_link) ||
    !both_arms_->setPoseTarget(normalizedPose(right_target), right_link))
  {
    RCLCPP_ERROR(node_->get_logger(), "Rejected one or both dual-arm pose targets");
    both_arms_->clearPoseTargets();
    return false;
  }
  const bool result = planAndExecute(*both_arms_, "both-arm pose target");
  both_arms_->clearPoseTargets();
  return result;
}

bool Commander::moveLeftGripper(double position)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!validGripperPosition(position, "left gripper")) {
    return false;
  }
  left_gripper_->setStartStateToCurrentState();
  if (!left_gripper_->setJointValueTarget(
      std::map<std::string, double>{{kLeftGripperJoint, position}}))
  {
    RCLCPP_ERROR(node_->get_logger(), "Rejected left gripper target %.4f", position);
    return false;
  }
  return planAndExecute(*left_gripper_, "left gripper");
}

bool Commander::moveRightGripper(double position)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!validGripperPosition(position, "right gripper")) {
    return false;
  }
  right_gripper_->setStartStateToCurrentState();
  if (!right_gripper_->setJointValueTarget(
      std::map<std::string, double>{{kRightGripperJoint, position}}))
  {
    RCLCPP_ERROR(node_->get_logger(), "Rejected right gripper target %.4f", position);
    return false;
  }
  return planAndExecute(*right_gripper_, "right gripper");
}

bool Commander::moveBothGrippers(double left_position, double right_position)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!validGripperPosition(left_position, "left gripper") ||
    !validGripperPosition(right_position, "right gripper"))
  {
    return false;
  }

  both_grippers_->setStartStateToCurrentState();
  const std::map<std::string, double> targets{
    {kLeftGripperJoint, left_position},
    {kRightGripperJoint, right_position}
  };
  if (!both_grippers_->setJointValueTarget(targets)) {
    RCLCPP_ERROR(node_->get_logger(), "Rejected dual-gripper target");
    return false;
  }
  return planAndExecute(*both_grippers_, "both grippers");
}

bool Commander::openLeftGripper()
{
  return moveLeftGripper(kGripperMinimum);
}

bool Commander::closeLeftGripper()
{
  return moveLeftGripper(kGripperMaximum);
}

bool Commander::openRightGripper()
{
  return moveRightGripper(kGripperMinimum);
}

bool Commander::closeRightGripper()
{
  return moveRightGripper(kGripperMaximum);
}

bool Commander::openBothGrippers()
{
  return moveBothGrippers(kGripperMinimum, kGripperMinimum);
}

bool Commander::closeBothGrippers()
{
  return moveBothGrippers(kGripperMaximum, kGripperMaximum);
}

std::vector<double> Commander::getLeftArmJoints() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return left_arm_->getCurrentJointValues();
}

std::vector<double> Commander::getRightArmJoints() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return right_arm_->getCurrentJointValues();
}

std::vector<double> Commander::getBothArmsJoints() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return both_arms_->getCurrentJointValues();
}

geometry_msgs::msg::PoseStamped Commander::getLeftArmPose(const std::string & link) const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return left_arm_->getCurrentPose(link.empty() ? options_.left_end_effector_link : link);
}

geometry_msgs::msg::PoseStamped Commander::getRightArmPose(const std::string & link) const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return right_arm_->getCurrentPose(link.empty() ? options_.right_end_effector_link : link);
}

void Commander::stop()
{
  left_arm_->stop();
  right_arm_->stop();
  both_arms_->stop();
  left_gripper_->stop();
  right_gripper_->stop();
  both_grippers_->stop();
  RCLCPP_WARN(node_->get_logger(), "Stop requested for all MoveIt groups");
}

void Commander::clearTargets()
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  left_arm_->clearPoseTargets();
  right_arm_->clearPoseTargets();
  both_arms_->clearPoseTargets();
}

bool Commander::moveNamed(MoveGroupInterface & group, const std::string & target_name)
{
  if (target_name.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Named target must not be empty");
    return false;
  }
  group.setStartStateToCurrentState();
  if (!group.setNamedTarget(target_name)) {
    RCLCPP_ERROR(node_->get_logger(), "Unknown named target: %s", target_name.c_str());
    return false;
  }
  return planAndExecute(group, target_name);
}

bool Commander::moveJoints(
  MoveGroupInterface & group,
  const std::vector<double> & joints,
  std::size_t expected_size,
  const std::string & label)
{
  if (joints.size() != expected_size) {
    RCLCPP_ERROR(
      node_->get_logger(), "%s requires %zu joints, received %zu",
      label.c_str(), expected_size, joints.size());
    return false;
  }
  static const std::vector<std::string> left_joint_names{
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint"
  };
  static const std::vector<std::string> right_joint_names{
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_pitch_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint"
  };

  std::map<std::string, double> targets;
  if (expected_size == kSingleArmJointCount) {
    const auto & names = label == "left arm" ? left_joint_names : right_joint_names;
    for (std::size_t index = 0; index < names.size(); ++index) {
      targets[names[index]] = joints[index];
    }
  } else {
    for (std::size_t index = 0; index < left_joint_names.size(); ++index) {
      targets[left_joint_names[index]] = joints[index];
      targets[right_joint_names[index]] = joints[index + left_joint_names.size()];
    }
  }

  group.setStartStateToCurrentState();
  if (!group.setJointValueTarget(targets)) {
    RCLCPP_ERROR(node_->get_logger(), "Rejected %s joint target", label.c_str());
    return false;
  }
  return planAndExecute(group, label + " joint target");
}

bool Commander::movePose(
  MoveGroupInterface & group,
  const geometry_msgs::msg::PoseStamped & target,
  bool cartesian,
  const std::string & end_effector_link,
  const std::string & label)
{
  group.setStartStateToCurrentState();
  group.clearPoseTargets();
  const auto normalized_target = normalizedPose(target);

  if (!cartesian) {
    if (!group.setPoseTarget(normalized_target, end_effector_link)) {
      RCLCPP_ERROR(
        node_->get_logger(), "Rejected pose target for %s using link '%s'",
        label.c_str(), end_effector_link.c_str());
      return false;
    }
    const bool result = planAndExecute(group, label + " pose target");
    group.clearPoseTargets();
    return result;
  }

  if (!group.setEndEffectorLink(end_effector_link)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Invalid end-effector link '%s' for %s",
      end_effector_link.c_str(), label.c_str());
    return false;
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  const std::vector<geometry_msgs::msg::Pose> waypoints{normalized_target.pose};
  const double fraction = group.computeCartesianPath(
    waypoints,
    options_.cartesian_eef_step,
    trajectory);

  if (fraction < options_.minimum_cartesian_fraction) {
    RCLCPP_ERROR(
      node_->get_logger(), "%s Cartesian path fraction %.3f is below required %.3f",
      label.c_str(), fraction, options_.minimum_cartesian_fraction);
    return false;
  }

  const auto result = group.execute(trajectory);
  if (!succeeded(result)) {
    RCLCPP_ERROR(node_->get_logger(), "%s Cartesian trajectory execution failed", label.c_str());
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(), "%s Cartesian trajectory executed, fraction %.3f",
    label.c_str(), fraction);
  return true;
}

bool Commander::planAndExecute(MoveGroupInterface & group, const std::string & label)
{
  MoveGroupInterface::Plan plan;
  const auto planning_result = group.plan(plan);
  if (!succeeded(planning_result)) {
    RCLCPP_ERROR(node_->get_logger(), "Planning failed for %s", label.c_str());
    return false;
  }

  const auto execution_result = group.execute(plan);
  if (!succeeded(execution_result)) {
    RCLCPP_ERROR(node_->get_logger(), "Execution failed for %s", label.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Completed %s", label.c_str());
  return true;
}

bool Commander::validGripperPosition(double position, const std::string & label) const
{
  if (!std::isfinite(position) || position < kGripperMinimum || position > kGripperMaximum) {
    RCLCPP_ERROR(
      node_->get_logger(), "%s position must be in [%.3f, %.3f] m, received %.4f",
      label.c_str(), kGripperMinimum, kGripperMaximum, position);
    return false;
  }
  return true;
}

geometry_msgs::msg::PoseStamped Commander::normalizedPose(
  const geometry_msgs::msg::PoseStamped & pose)
{
  auto result = pose;
  auto & q = result.pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm < 1e-9) {
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
  } else {
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    q.w /= norm;
  }
  return result;
}

}  // namespace zhishu_robot_commander_cpp
