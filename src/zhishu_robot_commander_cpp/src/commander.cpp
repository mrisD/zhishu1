#include "zhishu_robot_commander_cpp/commander.hpp"

#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
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
constexpr std::size_t kTorsoHeadJointCount = 3;
constexpr std::array<const char *, kTorsoHeadJointCount> kTorsoHeadJointNames{
  "waist_pitch_joint", "head_yaw_joint", "head_pitch_joint"
};
constexpr double kGripperMinimum = 0.0;
// Prismatic master-finger limits from zhishu_robot_description/urdf/gripper.xacro (m).
constexpr double kGripperMaximum = 0.06;

constexpr char kLeftGripperJoint[] = "left_gripper_left_finger_joint";
constexpr char kRightGripperJoint[] = "gripper_left_finger_joint";

bool succeeded(const moveit::core::MoveItErrorCode & code)
{
  return code == moveit::core::MoveItErrorCode::SUCCESS;
}

Result moveItFailure(
  const moveit::core::MoveItErrorCode & code,
  ErrorCode fallback,
  const std::string & message)
{
  auto sdk_code = fallback;
  if (code.val == moveit::core::MoveItErrorCode::TIMED_OUT) {
    sdk_code = ErrorCode::kTimeout;
  } else if (code.val == moveit::core::MoveItErrorCode::PREEMPTED) {
    sdk_code = ErrorCode::kCancelled;
  }
  auto detail = message + " (MoveIt code " + std::to_string(code.val) + ")";
  if (!code.message.empty()) {
    detail += ": " + code.message;
  }
  return Result::failure(sdk_code, std::move(detail), code.val);
}

Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond orientation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  orientation.normalize();

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = orientation.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();

  Eigen::Quaterniond orientation(transform.rotation());
  orientation.normalize();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

void validateOptions(const Commander::Options & options)
{
  const std::vector<std::string> required_names{
    options.left_arm_group,
    options.right_arm_group,
    options.both_arms_group,
    options.torso_head_group,
    options.left_gripper_group,
    options.right_gripper_group,
    options.both_grippers_group,
    options.left_end_effector_link,
    options.right_end_effector_link,
    options.left_tcp_link,
    options.right_tcp_link,
  };
  if (std::any_of(required_names.begin(), required_names.end(), [](const auto & value) {
      return value.empty();
    }))
  {
    throw std::invalid_argument("Commander group and end-effector names must not be empty");
  }

  const std::vector<double> numeric_options{
    options.velocity_scaling,
    options.acceleration_scaling,
    options.planning_time,
    options.cartesian_eef_step,
    options.minimum_cartesian_fraction,
  };
  if (!std::all_of(numeric_options.begin(), numeric_options.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    throw std::invalid_argument("Commander numeric options must be finite");
  }
  if (options.velocity_scaling <= 0.0 || options.velocity_scaling > 1.0 ||
    options.acceleration_scaling <= 0.0 || options.acceleration_scaling > 1.0)
  {
    throw std::invalid_argument("Commander velocity and acceleration scaling must be in (0, 1]");
  }
  if (options.planning_time <= 0.0 || options.planning_attempts < 1 ||
    options.cartesian_eef_step <= 0.0 || options.minimum_cartesian_fraction <= 0.0 ||
    options.minimum_cartesian_fraction > 1.0)
  {
    throw std::invalid_argument("Commander planning options are outside their valid ranges");
  }
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
  validateOptions(options_);

  left_arm_ = std::make_unique<MoveGroupInterface>(node_, options_.left_arm_group);
  right_arm_ = std::make_unique<MoveGroupInterface>(node_, options_.right_arm_group);
  both_arms_ = std::make_unique<MoveGroupInterface>(node_, options_.both_arms_group);
  torso_head_ = std::make_unique<MoveGroupInterface>(node_, options_.torso_head_group);
  left_gripper_ = std::make_unique<MoveGroupInterface>(node_, options_.left_gripper_group);
  right_gripper_ = std::make_unique<MoveGroupInterface>(node_, options_.right_gripper_group);
  both_grippers_ = std::make_unique<MoveGroupInterface>(node_, options_.both_grippers_group);

  configureMoveGroup(*left_arm_);
  configureMoveGroup(*right_arm_);
  configureMoveGroup(*both_arms_);
  configureMoveGroup(*torso_head_);
  configureMoveGroup(*left_gripper_);
  configureMoveGroup(*right_gripper_);
  configureMoveGroup(*both_grippers_);

  left_arm_->startStateMonitor();
  right_arm_->startStateMonitor();
  both_arms_->startStateMonitor();
  torso_head_->startStateMonitor();
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
  return moveLeftArmNamedResult(target_name).ok();
}

bool Commander::moveRightArmNamed(const std::string & target_name)
{
  return moveRightArmNamedResult(target_name).ok();
}

bool Commander::moveBothArmsNamed(const std::string & target_name)
{
  return moveBothArmsNamedResult(target_name).ok();
}

bool Commander::moveTorsoHeadNamed(const std::string & target_name)
{
  return moveTorsoHeadNamedResult(target_name).ok();
}

bool Commander::moveLeftArmJoints(const std::vector<double> & joints)
{
  return moveLeftArmJointsResult(joints).ok();
}

bool Commander::moveRightArmJoints(const std::vector<double> & joints)
{
  return moveRightArmJointsResult(joints).ok();
}

bool Commander::moveBothArmsJoints(const std::vector<double> & joints)
{
  return moveBothArmsJointsResult(joints).ok();
}

bool Commander::moveTorsoHeadJoints(const std::vector<double> & joints)
{
  return moveTorsoHeadJointsResult(joints).ok();
}

bool Commander::moveLeftArmPose(
  const geometry_msgs::msg::PoseStamped & target,
  bool cartesian,
  const std::string & end_effector_link)
{
  return moveLeftArmPoseResult(target, cartesian, end_effector_link).ok();
}

bool Commander::moveRightArmPose(
  const geometry_msgs::msg::PoseStamped & target,
  bool cartesian,
  const std::string & end_effector_link)
{
  return moveRightArmPoseResult(target, cartesian, end_effector_link).ok();
}

bool Commander::moveBothArmsPoses(
  const geometry_msgs::msg::PoseStamped & left_target,
  const geometry_msgs::msg::PoseStamped & right_target,
  const std::string & left_end_effector_link,
  const std::string & right_end_effector_link)
{
  return moveBothArmsPosesResult(
    left_target, right_target, left_end_effector_link, right_end_effector_link).ok();
}

bool Commander::moveLeftTcpPose(
  const geometry_msgs::msg::PoseStamped & target, bool cartesian)
{
  return moveLeftTcpPoseResult(target, cartesian).ok();
}

bool Commander::moveRightTcpPose(
  const geometry_msgs::msg::PoseStamped & target, bool cartesian)
{
  return moveRightTcpPoseResult(target, cartesian).ok();
}

bool Commander::moveBothTcpPoses(
  const geometry_msgs::msg::PoseStamped & left_target,
  const geometry_msgs::msg::PoseStamped & right_target)
{
  return moveBothTcpPosesResult(left_target, right_target).ok();
}

bool Commander::moveLeftGripper(double position)
{
  return moveLeftGripperResult(position).ok();
}

bool Commander::moveRightGripper(double position)
{
  return moveRightGripperResult(position).ok();
}

bool Commander::moveBothGrippers(double left_position, double right_position)
{
  return moveBothGrippersResult(left_position, right_position).ok();
}

bool Commander::openLeftGripper()
{
  return openLeftGripperResult().ok();
}

bool Commander::closeLeftGripper()
{
  return closeLeftGripperResult().ok();
}

bool Commander::openRightGripper()
{
  return openRightGripperResult().ok();
}

bool Commander::closeRightGripper()
{
  return closeRightGripperResult().ok();
}

bool Commander::openBothGrippers()
{
  return openBothGrippersResult().ok();
}

bool Commander::closeBothGrippers()
{
  return closeBothGrippersResult().ok();
}

Result Commander::moveLeftArmNamedResult(const std::string & target_name)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return moveNamed(*left_arm_, target_name);
}

Result Commander::moveRightArmNamedResult(const std::string & target_name)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return moveNamed(*right_arm_, target_name);
}

Result Commander::moveBothArmsNamedResult(const std::string & target_name)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return moveNamed(*both_arms_, target_name);
}

Result Commander::moveTorsoHeadNamedResult(const std::string & target_name)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return moveNamed(*torso_head_, target_name);
}

Result Commander::moveLeftArmJointsResult(const std::vector<double> & joints)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return moveJoints(*left_arm_, joints, kSingleArmJointCount, "left arm");
}

Result Commander::moveRightArmJointsResult(const std::vector<double> & joints)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return moveJoints(*right_arm_, joints, kSingleArmJointCount, "right arm");
}

Result Commander::moveBothArmsJointsResult(const std::vector<double> & joints)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return moveJoints(*both_arms_, joints, kDualArmJointCount, "both arms");
}

Result Commander::moveTorsoHeadJointsResult(const std::vector<double> & joints)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return moveJoints(*torso_head_, joints, kTorsoHeadJointCount, "torso/head");
}

Result Commander::moveLeftArmPoseResult(
  const geometry_msgs::msg::PoseStamped & target,
  bool cartesian,
  const std::string & end_effector_link)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  const auto & link = end_effector_link.empty() ?
    options_.left_end_effector_link : end_effector_link;
  return movePose(*left_arm_, target, cartesian, link, "left arm");
}

Result Commander::moveRightArmPoseResult(
  const geometry_msgs::msg::PoseStamped & target,
  bool cartesian,
  const std::string & end_effector_link)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  const auto & link = end_effector_link.empty() ?
    options_.right_end_effector_link : end_effector_link;
  return movePose(*right_arm_, target, cartesian, link, "right arm");
}

Result Commander::moveBothArmsPosesResult(
  const geometry_msgs::msg::PoseStamped & left_target,
  const geometry_msgs::msg::PoseStamped & right_target,
  const std::string & left_end_effector_link,
  const std::string & right_end_effector_link)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!isFinitePose(left_target) || !isFinitePose(right_target)) {
    RCLCPP_ERROR(node_->get_logger(), "Dual-arm pose target contains NaN or infinity");
    return Result::failure(
      ErrorCode::kInvalidArgument, "dual-arm pose target contains NaN or infinity");
  }
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
    return Result::failure(
      ErrorCode::kTargetRejected, "one or both dual-arm pose targets were rejected");
  }
  const auto result = planAndExecute(*both_arms_, "both-arm pose target");
  both_arms_->clearPoseTargets();
  return result;
}

Result Commander::moveLeftTcpPoseResult(
  const geometry_msgs::msg::PoseStamped & target, bool cartesian)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  geometry_msgs::msg::PoseStamped planning_target;
  const auto conversion = tcpTargetToPlanningTarget(
    *left_arm_, target, options_.left_end_effector_link, options_.left_tcp_link,
    "left gripper TCP", planning_target);
  if (!conversion) {
    return conversion;
  }
  return movePose(
    *left_arm_, planning_target, cartesian, options_.left_end_effector_link,
    "left gripper TCP");
}

Result Commander::moveRightTcpPoseResult(
  const geometry_msgs::msg::PoseStamped & target, bool cartesian)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  geometry_msgs::msg::PoseStamped planning_target;
  const auto conversion = tcpTargetToPlanningTarget(
    *right_arm_, target, options_.right_end_effector_link, options_.right_tcp_link,
    "right gripper TCP", planning_target);
  if (!conversion) {
    return conversion;
  }
  return movePose(
    *right_arm_, planning_target, cartesian, options_.right_end_effector_link,
    "right gripper TCP");
}

Result Commander::moveBothTcpPosesResult(
  const geometry_msgs::msg::PoseStamped & left_target,
  const geometry_msgs::msg::PoseStamped & right_target)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!isFinitePose(left_target) || !isFinitePose(right_target)) {
    return Result::failure(
      ErrorCode::kInvalidArgument, "dual-arm TCP pose target contains NaN or infinity");
  }

  geometry_msgs::msg::PoseStamped left_planning_target;
  geometry_msgs::msg::PoseStamped right_planning_target;
  auto conversion = tcpTargetToPlanningTarget(
    *both_arms_, left_target, options_.left_end_effector_link, options_.left_tcp_link,
    "left gripper TCP", left_planning_target);
  if (!conversion) {
    return conversion;
  }
  conversion = tcpTargetToPlanningTarget(
    *both_arms_, right_target, options_.right_end_effector_link, options_.right_tcp_link,
    "right gripper TCP", right_planning_target);
  if (!conversion) {
    return conversion;
  }

  both_arms_->setStartStateToCurrentState();
  both_arms_->clearPoseTargets();
  if (!both_arms_->setPoseTarget(left_planning_target, options_.left_end_effector_link) ||
    !both_arms_->setPoseTarget(right_planning_target, options_.right_end_effector_link))
  {
    both_arms_->clearPoseTargets();
    return Result::failure(
      ErrorCode::kTargetRejected, "one or both dual-arm TCP targets were rejected");
  }
  const auto result = planAndExecute(*both_arms_, "both-arm TCP pose target");
  both_arms_->clearPoseTargets();
  return result;
}

Result Commander::moveLeftGripperResult(double position)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  const auto validation = validateGripperPosition(position, "left gripper");
  if (!validation) {
    return validation;
  }
  left_gripper_->setStartStateToCurrentState();
  if (!left_gripper_->setJointValueTarget(
      std::map<std::string, double>{{kLeftGripperJoint, position}}))
  {
    RCLCPP_ERROR(node_->get_logger(), "Rejected left gripper target %.4f", position);
    return Result::failure(ErrorCode::kTargetRejected, "left gripper target was rejected");
  }
  return planAndExecute(*left_gripper_, "left gripper");
}

Result Commander::moveRightGripperResult(double position)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  const auto validation = validateGripperPosition(position, "right gripper");
  if (!validation) {
    return validation;
  }
  right_gripper_->setStartStateToCurrentState();
  if (!right_gripper_->setJointValueTarget(
      std::map<std::string, double>{{kRightGripperJoint, position}}))
  {
    RCLCPP_ERROR(node_->get_logger(), "Rejected right gripper target %.4f", position);
    return Result::failure(ErrorCode::kTargetRejected, "right gripper target was rejected");
  }
  return planAndExecute(*right_gripper_, "right gripper");
}

Result Commander::moveBothGrippersResult(double left_position, double right_position)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  const auto left_validation = validateGripperPosition(left_position, "left gripper");
  if (!left_validation) {
    return left_validation;
  }
  const auto right_validation = validateGripperPosition(right_position, "right gripper");
  if (!right_validation) {
    return right_validation;
  }

  both_grippers_->setStartStateToCurrentState();
  const std::map<std::string, double> targets{
    {kLeftGripperJoint, left_position},
    {kRightGripperJoint, right_position}
  };
  if (!both_grippers_->setJointValueTarget(targets)) {
    RCLCPP_ERROR(node_->get_logger(), "Rejected dual-gripper target");
    return Result::failure(ErrorCode::kTargetRejected, "dual-gripper target was rejected");
  }
  return planAndExecute(*both_grippers_, "both grippers");
}

Result Commander::openLeftGripperResult()
{
  return moveLeftGripperResult(kGripperMinimum);
}

Result Commander::closeLeftGripperResult()
{
  return moveLeftGripperResult(kGripperMaximum);
}

Result Commander::openRightGripperResult()
{
  return moveRightGripperResult(kGripperMinimum);
}

Result Commander::closeRightGripperResult()
{
  return moveRightGripperResult(kGripperMaximum);
}

Result Commander::openBothGrippersResult()
{
  return moveBothGrippersResult(kGripperMinimum, kGripperMinimum);
}

Result Commander::closeBothGrippersResult()
{
  return moveBothGrippersResult(kGripperMaximum, kGripperMaximum);
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

std::vector<double> Commander::getTorsoHeadJoints() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  const auto state = torso_head_->getCurrentState();
  if (!state) {
    throw std::runtime_error("current torso/head joint state is unavailable");
  }
  std::vector<double> joints;
  joints.reserve(kTorsoHeadJointCount);
  for (const auto * name : kTorsoHeadJointNames) {
    joints.push_back(state->getVariablePosition(name));
  }
  return joints;
}

std::vector<double> Commander::getLeftGripperJoints() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return left_gripper_->getCurrentJointValues();
}

std::vector<double> Commander::getRightGripperJoints() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return right_gripper_->getCurrentJointValues();
}

std::vector<double> Commander::getBothGrippersJoints() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return both_grippers_->getCurrentJointValues();
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

geometry_msgs::msg::PoseStamped Commander::getLeftTcpPose() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return left_arm_->getCurrentPose(options_.left_tcp_link);
}

geometry_msgs::msg::PoseStamped Commander::getRightTcpPose() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return right_arm_->getCurrentPose(options_.right_tcp_link);
}

void Commander::stop()
{
  left_arm_->stop();
  right_arm_->stop();
  both_arms_->stop();
  torso_head_->stop();
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
  torso_head_->clearPoseTargets();
}

Result Commander::moveNamed(MoveGroupInterface & group, const std::string & target_name)
{
  if (target_name.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Named target must not be empty");
    return Result::failure(ErrorCode::kInvalidArgument, "named target must not be empty");
  }
  group.setStartStateToCurrentState();
  if (!group.setNamedTarget(target_name)) {
    RCLCPP_ERROR(node_->get_logger(), "Unknown named target: %s", target_name.c_str());
    return Result::failure(
      ErrorCode::kTargetRejected, "unknown named target: " + target_name);
  }
  return planAndExecute(group, target_name);
}

Result Commander::moveJoints(
  MoveGroupInterface & group,
  const std::vector<double> & joints,
  std::size_t expected_size,
  const std::string & label)
{
  if (joints.size() != expected_size) {
    RCLCPP_ERROR(
      node_->get_logger(), "%s requires %zu joints, received %zu",
      label.c_str(), expected_size, joints.size());
    return Result::failure(
      ErrorCode::kInvalidArgument,
      label + " requires " + std::to_string(expected_size) + " joints, received " +
      std::to_string(joints.size()));
  }
  if (!std::all_of(joints.begin(), joints.end(), [](double value) {return std::isfinite(value);})) {
    RCLCPP_ERROR(node_->get_logger(), "%s joint target contains NaN or infinity", label.c_str());
    return Result::failure(
      ErrorCode::kInvalidArgument, label + " joint target contains NaN or infinity");
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
  if (expected_size == kTorsoHeadJointCount) {
    for (std::size_t index = 0; index < kTorsoHeadJointNames.size(); ++index) {
      targets[kTorsoHeadJointNames[index]] = joints[index];
    }
  } else if (expected_size == kSingleArmJointCount) {
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
    return Result::failure(
      ErrorCode::kTargetRejected, label + " joint target was rejected");
  }
  return planAndExecute(group, label + " joint target");
}

Result Commander::movePose(
  MoveGroupInterface & group,
  const geometry_msgs::msg::PoseStamped & target,
  bool cartesian,
  const std::string & end_effector_link,
  const std::string & label)
{
  if (!isFinitePose(target)) {
    RCLCPP_ERROR(node_->get_logger(), "%s pose target contains NaN or infinity", label.c_str());
    return Result::failure(
      ErrorCode::kInvalidArgument, label + " pose target contains NaN or infinity");
  }
  group.setStartStateToCurrentState();
  group.clearPoseTargets();
  const auto normalized_target = normalizedPose(target);

  if (!cartesian) {
    if (!group.setPoseTarget(normalized_target, end_effector_link)) {
      RCLCPP_ERROR(
        node_->get_logger(), "Rejected pose target for %s using link '%s'",
        label.c_str(), end_effector_link.c_str());
      group.clearPoseTargets();
      return Result::failure(
        ErrorCode::kTargetRejected, label + " pose target was rejected");
    }
    const auto result = planAndExecute(group, label + " pose target");
    group.clearPoseTargets();
    return result;
  }

  if (!group.setEndEffectorLink(end_effector_link)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Invalid end-effector link '%s' for %s",
      end_effector_link.c_str(), label.c_str());
    return Result::failure(
      ErrorCode::kInvalidArgument, "invalid end-effector link: " + end_effector_link);
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
    return Result::failure(
      ErrorCode::kPlanningFailed,
      label + " Cartesian path fraction " + std::to_string(fraction) +
      " is below required " + std::to_string(options_.minimum_cartesian_fraction));
  }

  const auto result = group.execute(trajectory);
  if (!succeeded(result)) {
    RCLCPP_ERROR(node_->get_logger(), "%s Cartesian trajectory execution failed", label.c_str());
    return moveItFailure(
      result, ErrorCode::kExecutionFailed,
      label + " Cartesian trajectory execution failed");
  }

  RCLCPP_INFO(
    node_->get_logger(), "%s Cartesian trajectory executed, fraction %.3f",
    label.c_str(), fraction);
  return Result::success(label + " Cartesian trajectory completed");
}

Result Commander::planAndExecute(MoveGroupInterface & group, const std::string & label)
{
  MoveGroupInterface::Plan plan;
  const auto planning_result = group.plan(plan);
  if (!succeeded(planning_result)) {
    RCLCPP_ERROR(node_->get_logger(), "Planning failed for %s", label.c_str());
    return moveItFailure(
      planning_result, ErrorCode::kPlanningFailed, "planning failed for " + label);
  }

  const auto execution_result = group.execute(plan);
  if (!succeeded(execution_result)) {
    RCLCPP_ERROR(node_->get_logger(), "Execution failed for %s", label.c_str());
    return moveItFailure(
      execution_result, ErrorCode::kExecutionFailed, "execution failed for " + label);
  }

  RCLCPP_INFO(node_->get_logger(), "Completed %s", label.c_str());
  return Result::success("completed " + label);
}

Result Commander::tcpTargetToPlanningTarget(
  MoveGroupInterface & group,
  const geometry_msgs::msg::PoseStamped & tcp_target,
  const std::string & planning_link,
  const std::string & tcp_link,
  const std::string & label,
  geometry_msgs::msg::PoseStamped & planning_target)
{
  if (!isFinitePose(tcp_target)) {
    return Result::failure(
      ErrorCode::kInvalidArgument, label + " target contains NaN or infinity");
  }

  const auto state = group.getCurrentState(2.0);
  if (!state) {
    return Result::failure(
      ErrorCode::kNotReady, "cannot read current robot state for " + label);
  }
  const auto * planning_link_model = state->getLinkModel(planning_link);
  const auto * tcp_link_model = state->getLinkModel(tcp_link);
  if (planning_link_model == nullptr || tcp_link_model == nullptr) {
    return Result::failure(
      ErrorCode::kInvalidArgument,
      "invalid planning/TCP link for " + label + ": " + planning_link + " -> " + tcp_link);
  }

  const Eigen::Isometry3d model_to_planning =
    state->getGlobalLinkTransform(planning_link_model);
  const Eigen::Isometry3d model_to_tcp =
    state->getGlobalLinkTransform(tcp_link_model);
  const Eigen::Isometry3d planning_to_tcp =
    model_to_planning.inverse() * model_to_tcp;

  const auto normalized_tcp_target = normalizedPose(tcp_target);
  const Eigen::Isometry3d frame_to_tcp = poseToEigen(normalized_tcp_target.pose);
  const Eigen::Isometry3d frame_to_planning = frame_to_tcp * planning_to_tcp.inverse();

  planning_target = normalized_tcp_target;
  planning_target.pose = eigenToPose(frame_to_planning);
  return Result::success(
    label + " converted from " + tcp_link + " to IK tip " + planning_link);
}

Result Commander::validateGripperPosition(double position, const std::string & label) const
{
  if (!std::isfinite(position) || position < kGripperMinimum || position > kGripperMaximum) {
    RCLCPP_ERROR(
      node_->get_logger(), "%s position must be in [%.3f, %.3f] m, received %.4f",
      label.c_str(), kGripperMinimum, kGripperMaximum, position);
    return Result::failure(
      ErrorCode::kInvalidArgument,
      label + " position must be in [" + std::to_string(kGripperMinimum) + ", " +
      std::to_string(kGripperMaximum) + "] m");
  }
  return Result::success();
}

bool Commander::isFinitePose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & position = pose.pose.position;
  const auto & orientation = pose.pose.orientation;
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z) && std::isfinite(orientation.x) &&
         std::isfinite(orientation.y) && std::isfinite(orientation.z) &&
         std::isfinite(orientation.w);
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
