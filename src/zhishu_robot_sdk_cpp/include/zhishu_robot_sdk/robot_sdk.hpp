#ifndef ZHISHU_ROBOT_SDK__ROBOT_SDK_HPP_
#define ZHISHU_ROBOT_SDK__ROBOT_SDK_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "zhishu_robot_sdk/commander.hpp"
#include "zhishu_robot_sdk/motion_handle.hpp"
#include "zhishu_robot_sdk/pose_math.hpp"

namespace zhishu_robot_sdk
{

enum class ArmGroup
{
  kLeft,
  kRight,
  kBoth,
};

enum class GripperGroup
{
  kLeft,
  kRight,
  kBoth,
};

struct RobotState
{
  Result result{Result::failure(ErrorCode::kNotReady, "robot state is unavailable")};
  std::vector<double> left_arm_joints;
  std::vector<double> right_arm_joints;
  std::vector<double> torso_head_joints;
  std::vector<double> left_gripper_joints;
  std::vector<double> right_gripper_joints;
  geometry_msgs::msg::PoseStamped left_arm_pose;
  geometry_msgs::msg::PoseStamped right_arm_pose;
  geometry_msgs::msg::PoseStamped left_tcp_pose;
  geometry_msgs::msg::PoseStamped right_tcp_pose;
};

struct SdkStatus
{
  bool ready{false};
  bool busy{false};
  MotionSnapshot motion;
};

class RobotSdk
{
public:
  explicit RobotSdk(const rclcpp::Node::SharedPtr & node);
  RobotSdk(const rclcpp::Node::SharedPtr & node, const Commander::Options & options);
  ~RobotSdk();

  RobotSdk(const RobotSdk &) = delete;
  RobotSdk & operator=(const RobotSdk &) = delete;

  Result moveNamed(ArmGroup group, const std::string & target_name);
  Result moveJoints(ArmGroup group, const std::vector<double> & joints);
  Result moveTorsoHeadNamed(const std::string & target_name);
  Result moveTorsoHeadJoints(const std::vector<double> & joints);
  Result movePose(
    ArmGroup group,
    const geometry_msgs::msg::PoseStamped & target,
    bool cartesian = false,
    const std::string & end_effector_link = "", const MotionFrames & frames = {});
  Result moveBothPoses(
    const geometry_msgs::msg::PoseStamped & left_target,
    const geometry_msgs::msg::PoseStamped & right_target,
    const std::string & left_end_effector_link = "",
    const std::string & right_end_effector_link = "",
    const MotionFrames & left_frames = {}, const MotionFrames & right_frames = {});
  Result moveTcpPose(
    ArmGroup group,
    const geometry_msgs::msg::PoseStamped & target,
    bool cartesian = false, const MotionFrames & frames = {});
  Result moveBothTcpPoses(
    const geometry_msgs::msg::PoseStamped & left_target,
    const geometry_msgs::msg::PoseStamped & right_target,
    const MotionFrames & left_frames = {}, const MotionFrames & right_frames = {});
  Result coordinateFramesCommand(
    const std::string & operation, const std::string & path, std::string & yaml);
  // ABB-style operation names; ROS/SI units, not RAPID units or syntax.
  Result MoveJ(ArmGroup arm, const geometry_msgs::msg::PoseStamped & target,
    const std::string & tool = "tool0", const std::string & wobj = "wobj0");
  Result MoveL(ArmGroup arm, const geometry_msgs::msg::PoseStamped & target,
    const std::string & tool = "tool0", const std::string & wobj = "wobj0");
  Result MoveAbsJ(ArmGroup arm, const std::vector<double> & joints_rad,
    const std::string & tool = "tool0", const std::string & wobj = "wobj0");
  static geometry_msgs::msg::PoseStamped Offs(const geometry_msgs::msg::PoseStamped & point,
    double dx_m, double dy_m, double dz_m) {
    return zhishu_robot_sdk::Offs(point, dx_m, dy_m, dz_m);
  }
  static geometry_msgs::msg::PoseStamped RelTool(const geometry_msgs::msg::PoseStamped & point,
    double dx_m, double dy_m, double dz_m, double rx_rad = 0., double ry_rad = 0., double rz_rad = 0.) {
    return zhishu_robot_sdk::RelTool(point, dx_m, dy_m, dz_m, rx_rad, ry_rad, rz_rad);
  }
  Result moveGripper(GripperGroup group, double position);
  Result moveBothGrippers(double left_position, double right_position);
  Result openGripper(GripperGroup group);
  Result closeGripper(GripperGroup group);

  MotionHandle moveNamedAsync(ArmGroup group, std::string target_name);
  MotionHandle moveJointsAsync(ArmGroup group, std::vector<double> joints);
  MotionHandle moveTorsoHeadNamedAsync(std::string target_name);
  MotionHandle moveTorsoHeadJointsAsync(std::vector<double> joints);
  MotionHandle movePoseAsync(
    ArmGroup group,
    geometry_msgs::msg::PoseStamped target,
    bool cartesian = false,
    std::string end_effector_link = "", MotionFrames frames = {});
  MotionHandle moveBothPosesAsync(
    geometry_msgs::msg::PoseStamped left_target,
    geometry_msgs::msg::PoseStamped right_target,
    std::string left_end_effector_link = "",
    std::string right_end_effector_link = "",
    MotionFrames left_frames = {}, MotionFrames right_frames = {});
  MotionHandle moveTcpPoseAsync(
    ArmGroup group,
    geometry_msgs::msg::PoseStamped target,
    bool cartesian = false, MotionFrames frames = {});
  MotionHandle moveBothTcpPosesAsync(
    geometry_msgs::msg::PoseStamped left_target,
    geometry_msgs::msg::PoseStamped right_target,
    MotionFrames left_frames = {}, MotionFrames right_frames = {});
  MotionHandle moveGripperAsync(GripperGroup group, double position);
  MotionHandle moveBothGrippersAsync(double left_position, double right_position);
  MotionHandle openGripperAsync(GripperGroup group);
  MotionHandle closeGripperAsync(GripperGroup group);

  bool cancelActiveMotion();
  SdkStatus status() const;
  // Returns [waist_pitch, head_yaw, head_pitch] in radians; throws if unavailable.
  std::vector<double> getTorsoHeadJoints() const;
  RobotState readState() const;

private:
  using Operation = std::function<Result(Commander &)>;

  Result runSynchronous(const std::string & operation_name, Operation operation);
  MotionHandle submit(std::string operation_name, Operation operation);
  MotionHandle rejectedHandle(std::string operation_name, Result result);
  void reapFinishedWorker();
  static std::string armOperationName(ArmGroup group, const std::string & operation);
  static std::string gripperOperationName(
    GripperGroup group, const std::string & operation);

  std::shared_ptr<Commander> commander_;
  mutable std::mutex task_mutex_;
  MotionHandle active_motion_;
  std::thread worker_;
  bool synchronous_busy_{false};
  std::atomic<std::uint64_t> next_motion_id_{1};
};

}  // namespace zhishu_robot_sdk

#endif  // ZHISHU_ROBOT_SDK__ROBOT_SDK_HPP_
