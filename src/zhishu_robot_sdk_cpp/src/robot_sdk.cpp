#include "zhishu_robot_sdk/robot_sdk.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>

namespace zhishu_robot_sdk
{
namespace
{
constexpr double kOpenGripperPosition = 0.0;
constexpr double kClosedGripperPosition = 0.06;
constexpr std::size_t kSingleArmJointCount = 7;
constexpr std::size_t kDualArmJointCount = 14;
constexpr std::size_t kTorsoHeadJointCount = 3;

bool isValidArmGroup(ArmGroup group)
{
  return group == ArmGroup::kLeft || group == ArmGroup::kRight || group == ArmGroup::kBoth;
}

bool isValidGripperGroup(GripperGroup group)
{
  return group == GripperGroup::kLeft || group == GripperGroup::kRight ||
         group == GripperGroup::kBoth;
}

bool allFinite(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);});
}

bool validPoseFeedback(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w) && std::isfinite(norm) && norm > 1e-9;
}

bool validJointFeedback(const std::vector<double> & joints, std::size_t expected_count)
{
  return joints.size() == expected_count && allFinite(joints);
}

bool validOptionalJointFeedback(const std::vector<double> & joints)
{
  return !joints.empty() && allFinite(joints);
}

Result validateJointTarget(ArmGroup group, const std::vector<double> & joints)
{
  if (!isValidArmGroup(group)) {
    return Result::failure(ErrorCode::kInvalidArgument, "invalid arm group");
  }
  const std::size_t expected = group == ArmGroup::kBoth ? kDualArmJointCount : kSingleArmJointCount;
  if (joints.size() != expected) {
    return Result::failure(
      ErrorCode::kInvalidArgument,
      "joint target requires " + std::to_string(expected) + " values, received " +
      std::to_string(joints.size()));
  }
  if (!allFinite(joints)) {
    return Result::failure(ErrorCode::kInvalidArgument, "joint target contains NaN or infinity");
  }
  return Result::success();
}

Result validateTorsoHeadTarget(const std::vector<double> & joints)
{
  if (joints.size() != kTorsoHeadJointCount) {
    return Result::failure(
      ErrorCode::kInvalidArgument,
      "torso/head target requires 3 values, received " + std::to_string(joints.size()));
  }
  if (!allFinite(joints)) {
    return Result::failure(ErrorCode::kInvalidArgument, "torso/head target contains NaN or infinity");
  }
  return Result::success();
}

Result validatePoseTarget(const geometry_msgs::msg::PoseStamped & target, const char * label)
{
  const auto & p = target.pose.position;
  const auto & q = target.pose.orientation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
    !std::isfinite(norm) || norm < 1e-9)
  {
    return Result::failure(
      ErrorCode::kInvalidArgument,
      std::string(label) + " must contain finite XYZ and a nonzero quaternion");
  }
  return Result::success();
}

Result validateNonEmptyName(const std::string & name, const char * label)
{
  if (name.empty()) {
    return Result::failure(ErrorCode::kInvalidArgument, std::string(label) + " must not be empty");
  }
  return Result::success();
}
}

struct RobotSdk::Impl
{
  // Reserved for future trajectory recording/playback, scan, and IO state.
};

RobotSdk::RobotSdk(const rclcpp::Node::SharedPtr & node)
: RobotSdk(node, RobotSdkOptions{})
{
}

RobotSdk::RobotSdk(
  const rclcpp::Node::SharedPtr & node,
  const Commander::Options & options)
: RobotSdk(node, RobotSdkOptions{options, {}, {}, {}, {}})
{
}

RobotSdk::RobotSdk(
  const rclcpp::Node::SharedPtr & node,
  const RobotSdkOptions & options)
: node_(node), options_(options), impl_(std::make_unique<Impl>()),
  commander_(std::make_shared<Commander>(node, options.commander))
{
}

RobotSdk::~RobotSdk()
{
  cancelActiveMotion();
  if (worker_.joinable()) {
    worker_.join();
  }
}

Result RobotSdk::moveNamed(ArmGroup group, const std::string & target_name)
{
  const auto name_check = validateNonEmptyName(target_name, "named target");
  if (!name_check.ok()) {return name_check;}
  if (!isValidArmGroup(group)) {
    return Result::failure(ErrorCode::kInvalidArgument, "invalid arm group");
  }

  return runSynchronous(
    armOperationName(group, "move_named"),
    [group, target_name](Commander & commander) {
      switch (group) {
        case ArmGroup::kLeft:
          return commander.moveLeftArmNamedResult(target_name);
        case ArmGroup::kRight:
          return commander.moveRightArmNamedResult(target_name);
        case ArmGroup::kBoth:
          return commander.moveBothArmsNamedResult(target_name);
      }
      return Result::failure(ErrorCode::kInvalidArgument, "invalid arm group");
    });
}

Result RobotSdk::moveJoints(ArmGroup group, const std::vector<double> & joints)
{
  const auto validation = validateJointTarget(group, joints);
  if (!validation.ok()) {return validation;}

  return runSynchronous(
    armOperationName(group, "move_joints"),
    [group, joints](Commander & commander) {
      switch (group) {
        case ArmGroup::kLeft:
          return commander.moveLeftArmJointsResult(joints);
        case ArmGroup::kRight:
          return commander.moveRightArmJointsResult(joints);
        case ArmGroup::kBoth:
          return commander.moveBothArmsJointsResult(joints);
      }
      return Result::failure(ErrorCode::kInvalidArgument, "invalid arm group");
    });
}

Result RobotSdk::moveTorsoHeadNamed(const std::string & target_name)
{
  const auto name_check = validateNonEmptyName(target_name, "torso/head named target");
  if (!name_check.ok()) {return name_check;}

  return runSynchronous(
    "torso_head.move_named",
    [target_name](Commander & commander) {
      return commander.moveTorsoHeadNamedResult(target_name);
    });
}

Result RobotSdk::moveTorsoHeadJoints(const std::vector<double> & joints)
{
  const auto validation = validateTorsoHeadTarget(joints);
  if (!validation.ok()) {return validation;}

  return runSynchronous(
    "torso_head.move_joints",
    [joints](Commander & commander) {
      return commander.moveTorsoHeadJointsResult(joints);
    });
}

Result RobotSdk::movePose(
  ArmGroup group,
  const geometry_msgs::msg::PoseStamped & target,
  bool cartesian,
  const std::string & end_effector_link, const MotionFrames & frames)
{
  const auto pose_validation = validatePoseTarget(target, "pose target");
  if (!pose_validation.ok()) {return pose_validation;}

  if (group == ArmGroup::kBoth) {
    return Result::failure(
      ErrorCode::kInvalidArgument, "movePose requires left or right; use moveBothPoses");
  }
  if (group != ArmGroup::kLeft && group != ArmGroup::kRight) {
    return Result::failure(ErrorCode::kInvalidArgument, "invalid arm group");
  }
  return runSynchronous(
    armOperationName(group, cartesian ? "move_cartesian_pose" : "move_pose"),
    [group, target, cartesian, end_effector_link, frames](Commander & commander) {
      if (group == ArmGroup::kLeft) {
        return commander.moveLeftArmPoseResult(target, cartesian, end_effector_link, frames);
      }
      return commander.moveRightArmPoseResult(target, cartesian, end_effector_link, frames);
    });
}

Result RobotSdk::moveBothPoses(
  const geometry_msgs::msg::PoseStamped & left_target,
  const geometry_msgs::msg::PoseStamped & right_target,
  const std::string & left_end_effector_link,
  const std::string & right_end_effector_link,
  const MotionFrames & left_frames, const MotionFrames & right_frames)
{
  const auto left_validation = validatePoseTarget(left_target, "left pose target");
  if (!left_validation.ok()) {return left_validation;}
  const auto right_validation = validatePoseTarget(right_target, "right pose target");
  if (!right_validation.ok()) {return right_validation;}

  return runSynchronous(
    "both_arms.move_poses",
    [left_target, right_target, left_end_effector_link, right_end_effector_link, left_frames, right_frames](
      Commander & commander) {
      return commander.moveBothArmsPosesResult(
        left_target, right_target, left_end_effector_link, right_end_effector_link, left_frames, right_frames);
    });
}

Result RobotSdk::moveTcpPose(
  ArmGroup group,
  const geometry_msgs::msg::PoseStamped & target,
  bool cartesian, const MotionFrames & frames)
{
  const auto pose_validation = validatePoseTarget(target, "TCP pose target");
  if (!pose_validation.ok()) {return pose_validation;}

  if (group == ArmGroup::kBoth) {
    return Result::failure(
      ErrorCode::kInvalidArgument, "moveTcpPose requires left or right; use moveBothTcpPoses");
  }
  if (group != ArmGroup::kLeft && group != ArmGroup::kRight) {
    return Result::failure(ErrorCode::kInvalidArgument, "invalid arm group");
  }
  return runSynchronous(
    armOperationName(group, cartesian ? "move_cartesian_tcp_pose" : "move_tcp_pose"),
    [group, target, cartesian, frames](Commander & commander) {
      return group == ArmGroup::kLeft ?
             commander.moveLeftTcpPoseResult(target, cartesian, frames) :
             commander.moveRightTcpPoseResult(target, cartesian, frames);
    });
}

Result RobotSdk::moveBothTcpPoses(
  const geometry_msgs::msg::PoseStamped & left_target,
  const geometry_msgs::msg::PoseStamped & right_target,
  const MotionFrames & left_frames, const MotionFrames & right_frames)
{
  const auto left_validation = validatePoseTarget(left_target, "left TCP pose target");
  if (!left_validation.ok()) {return left_validation;}
  const auto right_validation = validatePoseTarget(right_target, "right TCP pose target");
  if (!right_validation.ok()) {return right_validation;}

  return runSynchronous(
    "both_arms.move_tcp_poses",
    [left_target, right_target, left_frames, right_frames](Commander & commander) {
      return commander.moveBothTcpPosesResult(left_target, right_target, left_frames, right_frames);
    });
}

Result RobotSdk::moveGripper(GripperGroup group, double position)
{
  if (!isValidGripperGroup(group)) {
    return Result::failure(ErrorCode::kInvalidArgument, "invalid gripper group");
  }
  if (!std::isfinite(position)) {
    return Result::failure(ErrorCode::kInvalidArgument, "gripper position must be finite");
  }

  if (group == GripperGroup::kBoth) {
    return moveBothGrippers(position, position);
  }
  if (group != GripperGroup::kLeft && group != GripperGroup::kRight) {
    return Result::failure(ErrorCode::kInvalidArgument, "invalid gripper group");
  }
  return runSynchronous(
    gripperOperationName(group, "move"),
    [group, position](Commander & commander) {
      return group == GripperGroup::kLeft ?
             commander.moveLeftGripperResult(position) :
             commander.moveRightGripperResult(position);
    });
}

Result RobotSdk::coordinateFramesCommand(
  const std::string & operation, const std::string & path, std::string & yaml)
{
  return runSynchronous("coordinate_frames." + operation,
    [operation, path, &yaml](Commander & commander) {
      return commander.coordinateFramesCommand(operation, path, yaml);
    });
}

Result RobotSdk::MoveJ(ArmGroup arm, const geometry_msgs::msg::PoseStamped & target,
  const std::string & tool, const std::string & wobj)
{
  return movePose(arm, target, false, "", MotionFrames{tool, wobj});
}

Result RobotSdk::MoveL(ArmGroup arm, const geometry_msgs::msg::PoseStamped & target,
  const std::string & tool, const std::string & wobj)
{
  return movePose(arm, target, true, "", MotionFrames{tool, wobj});
}

Result RobotSdk::MoveAbsJ(ArmGroup arm, const std::vector<double> & joints_rad,
  const std::string & tool, const std::string & wobj)
{
  const auto validation = validateJointTarget(arm, joints_rad);
  if (!validation.ok()) {return validation;}

  const std::string group = arm == ArmGroup::kLeft ? "left" : arm == ArmGroup::kRight ? "right" : "both";
  return runSynchronous(group + ".MoveAbsJ",
    [group, joints_rad, tool, wobj](Commander & commander) {
      return commander.moveAbsJResult(group, joints_rad, MotionFrames{tool, wobj});
    });
}

Result RobotSdk::moveBothGrippers(double left_position, double right_position)
{
  if (!std::isfinite(left_position) || !std::isfinite(right_position)) {
    return Result::failure(ErrorCode::kInvalidArgument, "gripper positions must be finite");
  }

  return runSynchronous(
    "both_grippers.move",
    [left_position, right_position](Commander & commander) {
      return commander.moveBothGrippersResult(left_position, right_position);
    });
}

Result RobotSdk::openGripper(GripperGroup group)
{
  return moveGripper(group, kOpenGripperPosition);
}

Result RobotSdk::closeGripper(GripperGroup group)
{
  return moveGripper(group, kClosedGripperPosition);
}

MotionHandle RobotSdk::moveNamedAsync(ArmGroup group, std::string target_name)
{
  const auto name_check = validateNonEmptyName(target_name, "named target");
  if (!name_check.ok()) {return rejectedHandle("move_named", name_check);}
  if (!isValidArmGroup(group)) {
    return rejectedHandle("move_named", Result::failure(ErrorCode::kInvalidArgument, "invalid arm group"));
  }

  const auto operation_name = armOperationName(group, "move_named");
  return submit(
    operation_name,
    [group, target_name = std::move(target_name)](Commander & commander) {
      switch (group) {
        case ArmGroup::kLeft:
          return commander.moveLeftArmNamedResult(target_name);
        case ArmGroup::kRight:
          return commander.moveRightArmNamedResult(target_name);
        case ArmGroup::kBoth:
          return commander.moveBothArmsNamedResult(target_name);
      }
      return Result::failure(ErrorCode::kInvalidArgument, "invalid arm group");
    });
}

MotionHandle RobotSdk::moveJointsAsync(ArmGroup group, std::vector<double> joints)
{
  const auto validation = validateJointTarget(group, joints);
  if (!validation.ok()) {return rejectedHandle("move_joints", validation);}

  const auto operation_name = armOperationName(group, "move_joints");
  return submit(
    operation_name,
    [group, joints = std::move(joints)](Commander & commander) {
      switch (group) {
        case ArmGroup::kLeft:
          return commander.moveLeftArmJointsResult(joints);
        case ArmGroup::kRight:
          return commander.moveRightArmJointsResult(joints);
        case ArmGroup::kBoth:
          return commander.moveBothArmsJointsResult(joints);
      }
      return Result::failure(ErrorCode::kInvalidArgument, "invalid arm group");
    });
}

MotionHandle RobotSdk::moveTorsoHeadNamedAsync(std::string target_name)
{
  const auto name_check = validateNonEmptyName(target_name, "torso/head named target");
  if (!name_check.ok()) {return rejectedHandle("torso_head.move_named", name_check);}

  return submit(
    "torso_head.move_named",
    [target_name = std::move(target_name)](Commander & commander) {
      return commander.moveTorsoHeadNamedResult(target_name);
    });
}

MotionHandle RobotSdk::moveTorsoHeadJointsAsync(std::vector<double> joints)
{
  const auto validation = validateTorsoHeadTarget(joints);
  if (!validation.ok()) {return rejectedHandle("torso_head.move_joints", validation);}

  return submit(
    "torso_head.move_joints",
    [joints = std::move(joints)](Commander & commander) {
      return commander.moveTorsoHeadJointsResult(joints);
    });
}

MotionHandle RobotSdk::movePoseAsync(
  ArmGroup group,
  geometry_msgs::msg::PoseStamped target,
  bool cartesian,
  std::string end_effector_link, MotionFrames frames)
{
  const auto pose_validation = validatePoseTarget(target, "pose target");
  if (!pose_validation.ok()) {return rejectedHandle("move_pose", pose_validation);}

  if (group == ArmGroup::kBoth) {
    return rejectedHandle(
      "both_arms.move_pose",
      Result::failure(
        ErrorCode::kInvalidArgument,
        "movePoseAsync requires left or right; use moveBothPosesAsync"));
  }
  if (group != ArmGroup::kLeft && group != ArmGroup::kRight) {
    return rejectedHandle(
      "invalid_arm.move_pose",
      Result::failure(ErrorCode::kInvalidArgument, "invalid arm group"));
  }
  const auto operation_name = armOperationName(
    group, cartesian ? "move_cartesian_pose" : "move_pose");
  return submit(
    operation_name,
    [group, target = std::move(target), cartesian,
      end_effector_link = std::move(end_effector_link), frames = std::move(frames)](Commander & commander) {
      if (group == ArmGroup::kLeft) {
        return commander.moveLeftArmPoseResult(target, cartesian, end_effector_link, frames);
      }
      return commander.moveRightArmPoseResult(target, cartesian, end_effector_link, frames);
    });
}

MotionHandle RobotSdk::moveBothPosesAsync(
  geometry_msgs::msg::PoseStamped left_target,
  geometry_msgs::msg::PoseStamped right_target,
  std::string left_end_effector_link,
  std::string right_end_effector_link, MotionFrames left_frames, MotionFrames right_frames)
{
  const auto left_validation = validatePoseTarget(left_target, "left pose target");
  if (!left_validation.ok()) {return rejectedHandle("both_arms.move_poses", left_validation);}
  const auto right_validation = validatePoseTarget(right_target, "right pose target");
  if (!right_validation.ok()) {return rejectedHandle("both_arms.move_poses", right_validation);}

  return submit(
    "both_arms.move_poses",
    [left_target = std::move(left_target), right_target = std::move(right_target),
      left_end_effector_link = std::move(left_end_effector_link),
      right_end_effector_link = std::move(right_end_effector_link),
      left_frames = std::move(left_frames), right_frames = std::move(right_frames)](Commander & commander) {
      return commander.moveBothArmsPosesResult(
        left_target, right_target, left_end_effector_link, right_end_effector_link, left_frames, right_frames);
    });
}

MotionHandle RobotSdk::moveTcpPoseAsync(
  ArmGroup group,
  geometry_msgs::msg::PoseStamped target,
  bool cartesian, MotionFrames frames)
{
  const auto pose_validation = validatePoseTarget(target, "TCP pose target");
  if (!pose_validation.ok()) {return rejectedHandle("move_tcp_pose", pose_validation);}

  if (group == ArmGroup::kBoth) {
    return rejectedHandle(
      "both_arms.move_tcp_pose",
      Result::failure(
        ErrorCode::kInvalidArgument,
        "moveTcpPoseAsync requires left or right; use moveBothTcpPosesAsync"));
  }
  if (group != ArmGroup::kLeft && group != ArmGroup::kRight) {
    return rejectedHandle(
      "invalid_arm.move_tcp_pose",
      Result::failure(ErrorCode::kInvalidArgument, "invalid arm group"));
  }
  const auto operation_name = armOperationName(
    group, cartesian ? "move_cartesian_tcp_pose" : "move_tcp_pose");
  return submit(
    operation_name,
    [group, target = std::move(target), cartesian, frames = std::move(frames)](Commander & commander) {
      return group == ArmGroup::kLeft ?
             commander.moveLeftTcpPoseResult(target, cartesian, frames) :
             commander.moveRightTcpPoseResult(target, cartesian, frames);
    });
}

MotionHandle RobotSdk::moveBothTcpPosesAsync(
  geometry_msgs::msg::PoseStamped left_target,
  geometry_msgs::msg::PoseStamped right_target, MotionFrames left_frames, MotionFrames right_frames)
{
  const auto left_validation = validatePoseTarget(left_target, "left TCP pose target");
  if (!left_validation.ok()) {return rejectedHandle("both_arms.move_tcp_poses", left_validation);}
  const auto right_validation = validatePoseTarget(right_target, "right TCP pose target");
  if (!right_validation.ok()) {return rejectedHandle("both_arms.move_tcp_poses", right_validation);}

  return submit(
    "both_arms.move_tcp_poses",
    [left_target = std::move(left_target),
      right_target = std::move(right_target), left_frames = std::move(left_frames),
      right_frames = std::move(right_frames)](Commander & commander) {
      return commander.moveBothTcpPosesResult(left_target, right_target, left_frames, right_frames);
    });
}

MotionHandle RobotSdk::moveGripperAsync(GripperGroup group, double position)
{
  if (!isValidGripperGroup(group)) {
    return rejectedHandle("gripper.move", Result::failure(ErrorCode::kInvalidArgument, "invalid gripper group"));
  }
  if (!std::isfinite(position)) {
    return rejectedHandle("gripper.move", Result::failure(ErrorCode::kInvalidArgument, "gripper position must be finite"));
  }

  if (group == GripperGroup::kBoth) {
    return moveBothGrippersAsync(position, position);
  }
  if (group != GripperGroup::kLeft && group != GripperGroup::kRight) {
    return rejectedHandle(
      "invalid_gripper.move",
      Result::failure(ErrorCode::kInvalidArgument, "invalid gripper group"));
  }
  return submit(
    gripperOperationName(group, "move"),
    [group, position](Commander & commander) {
      return group == GripperGroup::kLeft ?
             commander.moveLeftGripperResult(position) :
             commander.moveRightGripperResult(position);
    });
}

MotionHandle RobotSdk::moveBothGrippersAsync(double left_position, double right_position)
{
  if (!std::isfinite(left_position) || !std::isfinite(right_position)) {
    return rejectedHandle("both_grippers.move", Result::failure(
      ErrorCode::kInvalidArgument, "gripper positions must be finite"));
  }

  return submit(
    "both_grippers.move",
    [left_position, right_position](Commander & commander) {
      return commander.moveBothGrippersResult(left_position, right_position);
    });
}

MotionHandle RobotSdk::openGripperAsync(GripperGroup group)
{
  return moveGripperAsync(group, kOpenGripperPosition);
}

MotionHandle RobotSdk::closeGripperAsync(GripperGroup group)
{
  return moveGripperAsync(group, kClosedGripperPosition);
}

bool RobotSdk::cancelActiveMotion()
{
  MotionHandle handle;
  bool stop_synchronous = false;
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    handle = active_motion_;
    stop_synchronous = synchronous_busy_;
  }
  if (handle.valid() && !handle.done()) {
    return handle.cancel();
  }
  if (stop_synchronous) {
    commander_->stop();
    return true;
  }
  return false;
}

SdkStatus RobotSdk::status() const
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  SdkStatus result;
  result.sdk_initialized = static_cast<bool>(node_) && static_cast<bool>(commander_);
  result.ros_context_available = static_cast<bool>(node_) && rclcpp::ok();
  // A real MoveIt action/controller readiness probe has not been implemented yet. Keep this
  // false rather than reporting ready merely because the Commander object was constructed.
  result.motion_interface_ready = false;
  result.ready = result.sdk_initialized && result.ros_context_available &&
    result.motion_interface_ready;
  result.busy = synchronous_busy_ || (active_motion_.valid() && !active_motion_.done());
  result.motion = active_motion_.snapshot();
  return result;
}

std::vector<double> RobotSdk::getTorsoHeadJoints() const
{
  return commander_->getTorsoHeadJoints();
}

RobotState RobotSdk::readState() const
{
  RobotState state;
  try {
    state.left_arm_joints = commander_->getLeftArmJoints();
    state.right_arm_joints = commander_->getRightArmJoints();
    state.torso_head_joints = commander_->getTorsoHeadJoints();
    state.left_gripper_joints = commander_->getLeftGripperJoints();
    state.right_gripper_joints = commander_->getRightGripperJoints();
    state.left_arm_pose = commander_->getLeftArmPose();
    state.right_arm_pose = commander_->getRightArmPose();
    state.left_tcp_pose = commander_->getLeftTcpPose();
    state.right_tcp_pose = commander_->getRightTcpPose();

    state.left_arm_joints_valid = validJointFeedback(
      state.left_arm_joints, kSingleArmJointCount);
    state.right_arm_joints_valid = validJointFeedback(
      state.right_arm_joints, kSingleArmJointCount);
    state.torso_head_joints_valid = validJointFeedback(
      state.torso_head_joints, kTorsoHeadJointCount);
    state.left_gripper_joints_valid = validOptionalJointFeedback(state.left_gripper_joints);
    state.right_gripper_joints_valid = validOptionalJointFeedback(state.right_gripper_joints);
    state.left_arm_pose_valid = validPoseFeedback(state.left_arm_pose);
    state.right_arm_pose_valid = validPoseFeedback(state.right_arm_pose);
    state.left_tcp_pose_valid = validPoseFeedback(state.left_tcp_pose);
    state.right_tcp_pose_valid = validPoseFeedback(state.right_tcp_pose);

    if (!state.left_arm_joints_valid || !state.right_arm_joints_valid ||
      !state.torso_head_joints_valid)
    {
      state.result = Result::failure(
        ErrorCode::kNotReady,
        "required joint feedback is incomplete or invalid: expected 7 joints per arm and "
        "3 torso/head joints, all finite");
      return state;
    }
    state.result = Result::success(
      "required arm and torso/head joint feedback is valid; optional feedback validity is "
      "reported in RobotState flags");
  } catch (const std::exception & error) {
    state.result = Result::failure(ErrorCode::kNotReady, error.what());
  }
  return state;
}

Result RobotSdk::runSynchronous(const std::string & operation_name, Operation operation)
{
  reapFinishedWorker();
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    if (synchronous_busy_ || (active_motion_.valid() && !active_motion_.done())) {
      return Result::failure(
        ErrorCode::kBusy, "cannot start " + operation_name + "; another motion is active");
    }
    synchronous_busy_ = true;
  }

  Result result;
  try {
    result = operation(*commander_);
  } catch (const std::exception & error) {
    result = Result::failure(ErrorCode::kInternalError, error.what());
  } catch (...) {
    result = Result::failure(ErrorCode::kInternalError, "unknown SDK exception");
  }

  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    synchronous_busy_ = false;
  }
  return result;
}

MotionHandle RobotSdk::submit(std::string operation_name, Operation operation)
{
  std::thread finished;
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    if (synchronous_busy_ || (active_motion_.valid() && !active_motion_.done())) {
      return rejectedHandle(
        std::move(operation_name),
        Result::failure(ErrorCode::kBusy, "another motion is active"));
    }
    if (worker_.joinable()) {
      finished = std::move(worker_);
    }
  }
  if (finished.joinable()) {
    finished.join();
  }

  std::lock_guard<std::mutex> lock(task_mutex_);
  if (synchronous_busy_ || (active_motion_.valid() && !active_motion_.done())) {
    return rejectedHandle(
      std::move(operation_name),
      Result::failure(ErrorCode::kBusy, "another motion is active"));
  }

  auto state = std::make_shared<detail::MotionState>();
  state->id = next_motion_id_.fetch_add(1);
  state->operation = std::move(operation_name);
  std::weak_ptr<Commander> weak_commander = commander_;
  state->cancel_callback = [weak_commander]() {
      if (const auto commander = weak_commander.lock()) {
        commander->stop();
      }
    };
  active_motion_ = MotionHandle(state);

  try {
    worker_ = std::thread(
      [commander = commander_, state, operation = std::move(operation)]() mutable {
        {
          std::lock_guard<std::mutex> state_lock(state->mutex);
          if (state->cancel_requested) {
            state->status = MotionStatus::kCancelled;
            state->result = Result::failure(
              ErrorCode::kCancelled, "motion cancelled before execution");
            state->condition.notify_all();
            return;
          }
          state->status = MotionStatus::kRunning;
        }
        state->condition.notify_all();

        Result operation_result;
        try {
          operation_result = operation(*commander);
        } catch (const std::exception & error) {
          operation_result = Result::failure(ErrorCode::kInternalError, error.what());
        } catch (...) {
          operation_result = Result::failure(
            ErrorCode::kInternalError, "unknown SDK exception");
        }

        {
          std::lock_guard<std::mutex> state_lock(state->mutex);
          if (state->cancel_requested) {
            state->status = MotionStatus::kCancelled;
            state->result = Result::failure(ErrorCode::kCancelled, "motion cancelled");
          } else {
            state->status = operation_result.ok() ?
              MotionStatus::kSucceeded : MotionStatus::kFailed;
            state->result = std::move(operation_result);
          }
        }
        state->condition.notify_all();
      });
  } catch (const std::exception & error) {
    std::lock_guard<std::mutex> state_lock(state->mutex);
    state->status = MotionStatus::kFailed;
    state->result = Result::failure(ErrorCode::kInternalError, error.what());
    state->condition.notify_all();
  }
  return active_motion_;
}

MotionHandle RobotSdk::rejectedHandle(std::string operation_name, Result result)
{
  auto state = std::make_shared<detail::MotionState>();
  state->operation = std::move(operation_name);
  state->status = MotionStatus::kFailed;
  state->result = std::move(result);
  return MotionHandle(std::move(state));
}

void RobotSdk::reapFinishedWorker()
{
  std::thread finished;
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    if (worker_.joinable() && active_motion_.done()) {
      finished = std::move(worker_);
    }
  }
  if (finished.joinable()) {
    finished.join();
  }
}

std::string RobotSdk::armOperationName(ArmGroup group, const std::string & operation)
{
  switch (group) {
    case ArmGroup::kLeft:
      return "left_arm." + operation;
    case ArmGroup::kRight:
      return "right_arm." + operation;
    case ArmGroup::kBoth:
      return "both_arms." + operation;
  }
  return "invalid_arm." + operation;
}

std::string RobotSdk::gripperOperationName(
  GripperGroup group, const std::string & operation)
{
  switch (group) {
    case GripperGroup::kLeft:
      return "left_gripper." + operation;
    case GripperGroup::kRight:
      return "right_gripper." + operation;
    case GripperGroup::kBoth:
      return "both_grippers." + operation;
  }
  return "invalid_gripper." + operation;
}

}  // namespace zhishu_robot_sdk
