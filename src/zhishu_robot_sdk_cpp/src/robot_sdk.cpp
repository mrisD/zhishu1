#include "zhishu_robot_sdk/robot_sdk.hpp"

#include <exception>
#include <utility>

namespace zhishu_robot_sdk
{
namespace
{
constexpr double kOpenGripperPosition = 0.0;
constexpr double kClosedGripperPosition = 0.06;
}

RobotSdk::RobotSdk(const rclcpp::Node::SharedPtr & node)
: RobotSdk(node, Commander::Options{})
{
}

RobotSdk::RobotSdk(
  const rclcpp::Node::SharedPtr & node,
  const Commander::Options & options)
: commander_(std::make_shared<Commander>(node, options))
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
  return runSynchronous(
    "torso_head.move_named",
    [target_name](Commander & commander) {
      return commander.moveTorsoHeadNamedResult(target_name);
    });
}

Result RobotSdk::moveTorsoHeadJoints(const std::vector<double> & joints)
{
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
  return runSynchronous(
    "both_arms.move_tcp_poses",
    [left_target, right_target, left_frames, right_frames](Commander & commander) {
      return commander.moveBothTcpPosesResult(left_target, right_target, left_frames, right_frames);
    });
}

Result RobotSdk::moveGripper(GripperGroup group, double position)
{
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
  const std::string group = arm == ArmGroup::kLeft ? "left" : arm == ArmGroup::kRight ? "right" :
    arm == ArmGroup::kBoth ? "both" : "invalid";
  return runSynchronous(group + ".MoveAbsJ",
    [group, joints_rad, tool, wobj](Commander & commander) {
      return commander.moveAbsJResult(group, joints_rad, MotionFrames{tool, wobj});
    });
}

Result RobotSdk::moveBothGrippers(double left_position, double right_position)
{
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
  return submit(
    "torso_head.move_named",
    [target_name = std::move(target_name)](Commander & commander) {
      return commander.moveTorsoHeadNamedResult(target_name);
    });
}

MotionHandle RobotSdk::moveTorsoHeadJointsAsync(std::vector<double> joints)
{
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
  result.ready = static_cast<bool>(commander_);
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
    state.result = Result::success("robot state read successfully");
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
