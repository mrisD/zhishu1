#ifndef ZHISHU_ROBOT_SDK__COMMANDER_HPP_
#define ZHISHU_ROBOT_SDK__COMMANDER_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "zhishu_robot_sdk/result.hpp"
#include "zhishu_robot_sdk/visibility_control.h"
#include "zhishu_robot_sdk/coordinate_frames.hpp"

namespace zhishu_robot_sdk
{

class ZHISHU_ROBOT_SDK_PUBLIC Commander
{
public:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

  struct Options
  {
    std::string left_arm_group{"left_arm"};
    std::string right_arm_group{"right_arm"};
    std::string both_arms_group{"both_arms"};
    std::string torso_head_group{"torso_head"};
    std::string left_gripper_group{"left_gripper"};
    std::string right_gripper_group{"right_gripper"};
    std::string both_grippers_group{"both_grippers"};
    std::string left_end_effector_link{"L_hand_link"};
    std::string right_end_effector_link{"R_hand_link"};
    std::string left_tcp_link{"left_gripper_tcp"};
    std::string right_tcp_link{"right_gripper_tcp"};
    std::string coordinate_frames_file;
    double velocity_scaling{0.30};
    double acceleration_scaling{0.30};
    double planning_time{5.0};
    int planning_attempts{5};
    double cartesian_eef_step{0.005};
    double minimum_cartesian_fraction{0.95};
  };

  explicit Commander(const rclcpp::Node::SharedPtr & node);
  Commander(
    const rclcpp::Node::SharedPtr & node,
    const Options & options);
  ~Commander() = default;

  Commander(const Commander &) = delete;
  Commander & operator=(const Commander &) = delete;

  bool moveLeftArmNamed(const std::string & target_name);
  bool moveRightArmNamed(const std::string & target_name);
  bool moveBothArmsNamed(const std::string & target_name);
  bool moveTorsoHeadNamed(const std::string & target_name);

  bool moveLeftArmJoints(const std::vector<double> & joints);
  bool moveRightArmJoints(const std::vector<double> & joints);
  bool moveBothArmsJoints(const std::vector<double> & joints);
  // Joint order: waist_pitch, head_yaw, head_pitch (radians).
  bool moveTorsoHeadJoints(const std::vector<double> & joints);

  bool moveLeftArmPose(
    const geometry_msgs::msg::PoseStamped & target,
    bool cartesian = false,
    const std::string & end_effector_link = "", const MotionFrames & frames = {});
  bool moveRightArmPose(
    const geometry_msgs::msg::PoseStamped & target,
    bool cartesian = false,
    const std::string & end_effector_link = "", const MotionFrames & frames = {});
  bool moveBothArmsPoses(
    const geometry_msgs::msg::PoseStamped & left_target,
    const geometry_msgs::msg::PoseStamped & right_target,
    const std::string & left_end_effector_link = "",
    const std::string & right_end_effector_link = "",
    const MotionFrames & left_frames = {}, const MotionFrames & right_frames = {});

  // True gripper-TCP targets. These are converted to the configured arm IK
  // tip (L_hand_link/R_hand_link) through the rigid transform in RobotModel.
  bool moveLeftTcpPose(
    const geometry_msgs::msg::PoseStamped & target, bool cartesian = false,
    const MotionFrames & frames = {});
  bool moveRightTcpPose(
    const geometry_msgs::msg::PoseStamped & target, bool cartesian = false,
    const MotionFrames & frames = {});
  bool moveBothTcpPoses(
    const geometry_msgs::msg::PoseStamped & left_target,
    const geometry_msgs::msg::PoseStamped & right_target,
    const MotionFrames & left_frames = {}, const MotionFrames & right_frames = {});

  bool moveLeftGripper(double position);
  bool moveRightGripper(double position);
  bool moveBothGrippers(double left_position, double right_position);
  bool openLeftGripper();
  bool closeLeftGripper();
  bool openRightGripper();
  bool closeRightGripper();
  bool openBothGrippers();
  bool closeBothGrippers();

  // Structured-result variants for SDK users. The bool methods above remain
  // source-compatible wrappers around these methods.
  Result moveLeftArmNamedResult(const std::string & target_name);
  Result moveRightArmNamedResult(const std::string & target_name);
  Result moveBothArmsNamedResult(const std::string & target_name);
  Result moveTorsoHeadNamedResult(const std::string & target_name);

  Result moveLeftArmJointsResult(const std::vector<double> & joints);
  Result moveRightArmJointsResult(const std::vector<double> & joints);
  Result moveBothArmsJointsResult(const std::vector<double> & joints);
  Result moveTorsoHeadJointsResult(const std::vector<double> & joints);

  Result moveLeftArmPoseResult(
    const geometry_msgs::msg::PoseStamped & target,
    bool cartesian = false,
    const std::string & end_effector_link = "", const MotionFrames & frames = {});
  Result moveRightArmPoseResult(
    const geometry_msgs::msg::PoseStamped & target,
    bool cartesian = false,
    const std::string & end_effector_link = "", const MotionFrames & frames = {});
  Result moveBothArmsPosesResult(
    const geometry_msgs::msg::PoseStamped & left_target,
    const geometry_msgs::msg::PoseStamped & right_target,
    const std::string & left_end_effector_link = "",
    const std::string & right_end_effector_link = "",
    const MotionFrames & left_frames = {}, const MotionFrames & right_frames = {});

  Result moveLeftTcpPoseResult(
    const geometry_msgs::msg::PoseStamped & target, bool cartesian = false,
    const MotionFrames & frames = {});
  Result moveRightTcpPoseResult(
    const geometry_msgs::msg::PoseStamped & target, bool cartesian = false,
    const MotionFrames & frames = {});
  Result moveBothTcpPosesResult(
    const geometry_msgs::msg::PoseStamped & left_target,
    const geometry_msgs::msg::PoseStamped & right_target,
    const MotionFrames & left_frames = {}, const MotionFrames & right_frames = {});

  // Management never changes a global active selection; frames are per motion.
  Result coordinateFramesCommand(
    const std::string & operation, const std::string & file_path, std::string & yaml);
  Result moveAbsJResult(
    const std::string & arm, const std::vector<double> & joints_rad,
    const MotionFrames & frames = {"tool0", "wobj0"});

  Result moveLeftGripperResult(double position);
  Result moveRightGripperResult(double position);
  Result moveBothGrippersResult(double left_position, double right_position);
  Result openLeftGripperResult();
  Result closeLeftGripperResult();
  Result openRightGripperResult();
  Result closeRightGripperResult();
  Result openBothGrippersResult();
  Result closeBothGrippersResult();

  std::vector<double> getLeftArmJoints() const;
  std::vector<double> getRightArmJoints() const;
  std::vector<double> getBothArmsJoints() const;
  std::vector<double> getTorsoHeadJoints() const;
  std::vector<double> getLeftGripperJoints() const;
  std::vector<double> getRightGripperJoints() const;
  std::vector<double> getBothGrippersJoints() const;
  geometry_msgs::msg::PoseStamped getLeftArmPose(const std::string & link = "") const;
  geometry_msgs::msg::PoseStamped getRightArmPose(const std::string & link = "") const;
  geometry_msgs::msg::PoseStamped getLeftTcpPose() const;
  geometry_msgs::msg::PoseStamped getRightTcpPose() const;

  void stop();
  void clearTargets();

private:
  void configureMoveGroup(MoveGroupInterface & group);
  Result moveNamed(MoveGroupInterface & group, const std::string & target_name);
  Result moveJoints(
    MoveGroupInterface & group,
    const std::vector<double> & joints,
    std::size_t expected_size,
    const std::string & label);
  Result movePose(
    MoveGroupInterface & group,
    const geometry_msgs::msg::PoseStamped & target,
    bool cartesian,
    const std::string & end_effector_link,
    const std::string & label,
    const Eigen::Isometry3d & flange_T_tool = Eigen::Isometry3d::Identity());
  Result planAndExecute(MoveGroupInterface & group, const std::string & label);
  Result coordinateTargetToPlanningTarget(
    MoveGroupInterface & group,
    const geometry_msgs::msg::PoseStamped & tcp_target,
    const std::string & planning_link,
    const std::string & arm, const MotionFrames & frames, bool tcp_default,
    const std::string & label,
    geometry_msgs::msg::PoseStamped & planning_target, Eigen::Isometry3d & flange_T_tool);
  Result validateGripperPosition(double position, const std::string & label) const;
  static bool isFinitePose(const geometry_msgs::msg::PoseStamped & pose);
  static geometry_msgs::msg::PoseStamped normalizedPose(
    const geometry_msgs::msg::PoseStamped & pose);

  rclcpp::Node::SharedPtr node_;
  Options options_;
  CoordinateFrames coordinate_frames_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<MoveGroupInterface> left_arm_;
  std::unique_ptr<MoveGroupInterface> right_arm_;
  std::unique_ptr<MoveGroupInterface> both_arms_;
  std::unique_ptr<MoveGroupInterface> torso_head_;
  std::unique_ptr<MoveGroupInterface> left_gripper_;
  std::unique_ptr<MoveGroupInterface> right_gripper_;
  std::unique_ptr<MoveGroupInterface> both_grippers_;

  mutable std::mutex command_mutex_;
};

}  // namespace zhishu_robot_sdk

#endif  // ZHISHU_ROBOT_SDK__COMMANDER_HPP_
