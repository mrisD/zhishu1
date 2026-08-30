#ifndef ZHISHU_ROBOT_COMMANDER_CPP__COMMANDER_HPP_
#define ZHISHU_ROBOT_COMMANDER_CPP__COMMANDER_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "zhishu_robot_commander_cpp/result.hpp"

namespace zhishu_robot_commander_cpp
{

class Commander
{
public:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

  struct Options
  {
    std::string left_arm_group{"left_arm"};
    std::string right_arm_group{"right_arm"};
    std::string both_arms_group{"both_arms"};
    std::string left_gripper_group{"left_gripper"};
    std::string right_gripper_group{"right_gripper"};
    std::string both_grippers_group{"both_grippers"};
    std::string left_end_effector_link{"L_hand_link"};
    std::string right_end_effector_link{"R_hand_link"};
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

  bool moveLeftArmJoints(const std::vector<double> & joints);
  bool moveRightArmJoints(const std::vector<double> & joints);
  bool moveBothArmsJoints(const std::vector<double> & joints);

  bool moveLeftArmPose(
    const geometry_msgs::msg::PoseStamped & target,
    bool cartesian = false,
    const std::string & end_effector_link = "");
  bool moveRightArmPose(
    const geometry_msgs::msg::PoseStamped & target,
    bool cartesian = false,
    const std::string & end_effector_link = "");
  bool moveBothArmsPoses(
    const geometry_msgs::msg::PoseStamped & left_target,
    const geometry_msgs::msg::PoseStamped & right_target,
    const std::string & left_end_effector_link = "",
    const std::string & right_end_effector_link = "");

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

  Result moveLeftArmJointsResult(const std::vector<double> & joints);
  Result moveRightArmJointsResult(const std::vector<double> & joints);
  Result moveBothArmsJointsResult(const std::vector<double> & joints);

  Result moveLeftArmPoseResult(
    const geometry_msgs::msg::PoseStamped & target,
    bool cartesian = false,
    const std::string & end_effector_link = "");
  Result moveRightArmPoseResult(
    const geometry_msgs::msg::PoseStamped & target,
    bool cartesian = false,
    const std::string & end_effector_link = "");
  Result moveBothArmsPosesResult(
    const geometry_msgs::msg::PoseStamped & left_target,
    const geometry_msgs::msg::PoseStamped & right_target,
    const std::string & left_end_effector_link = "",
    const std::string & right_end_effector_link = "");

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
  std::vector<double> getLeftGripperJoints() const;
  std::vector<double> getRightGripperJoints() const;
  std::vector<double> getBothGrippersJoints() const;
  geometry_msgs::msg::PoseStamped getLeftArmPose(const std::string & link = "") const;
  geometry_msgs::msg::PoseStamped getRightArmPose(const std::string & link = "") const;

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
    const std::string & label);
  Result planAndExecute(MoveGroupInterface & group, const std::string & label);
  Result validateGripperPosition(double position, const std::string & label) const;
  static bool isFinitePose(const geometry_msgs::msg::PoseStamped & pose);
  static geometry_msgs::msg::PoseStamped normalizedPose(
    const geometry_msgs::msg::PoseStamped & pose);

  rclcpp::Node::SharedPtr node_;
  Options options_;

  std::unique_ptr<MoveGroupInterface> left_arm_;
  std::unique_ptr<MoveGroupInterface> right_arm_;
  std::unique_ptr<MoveGroupInterface> both_arms_;
  std::unique_ptr<MoveGroupInterface> left_gripper_;
  std::unique_ptr<MoveGroupInterface> right_gripper_;
  std::unique_ptr<MoveGroupInterface> both_grippers_;

  mutable std::mutex command_mutex_;
};

}  // namespace zhishu_robot_commander_cpp

#endif  // ZHISHU_ROBOT_COMMANDER_CPP__COMMANDER_HPP_
