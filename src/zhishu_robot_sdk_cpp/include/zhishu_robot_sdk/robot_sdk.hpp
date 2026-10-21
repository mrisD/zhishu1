#ifndef ZHISHU_ROBOT_SDK__ROBOT_SDK_HPP_
#define ZHISHU_ROBOT_SDK__ROBOT_SDK_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
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
#include "zhishu_robot_sdk/robot_io.hpp"
#include "zhishu_robot_sdk/robot_status.hpp"
#include "zhishu_robot_sdk/visibility_control.h"

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

  // Optional feedback validity. These flags avoid treating a default/empty ROS message as
  // a valid pose or gripper state. Arm and torso/head joint feedback remains required for
  // RobotState::result to be successful; optional feedback is reported independently.
  bool left_arm_joints_valid{false};
  bool right_arm_joints_valid{false};
  bool torso_head_joints_valid{false};
  bool left_gripper_joints_valid{false};
  bool right_gripper_joints_valid{false};
  bool left_arm_pose_valid{false};
  bool right_arm_pose_valid{false};
  bool left_tcp_pose_valid{false};
  bool right_tcp_pose_valid{false};
};

struct SdkStatus
{
  // `ready` is true only when the SDK can verify the motion backend is ready. It must not
  // be inferred merely from a non-null Commander pointer.
  bool ready{false};
  bool busy{false};
  MotionSnapshot motion;
  bool sdk_initialized{false};
  bool ros_context_available{false};
  bool motion_interface_ready{false};
};

/// Teaching input mode. Physical end-effector-button drag is reserved for a later phase.
enum class DragTeachMode
{
  /// Software teaching through bounded SDK jog commands; usable in simulation/fake hardware.
  kSoftwareJog,
  /// Reserved for future hardware-specific end-effector button integration.
  kPhysicalButton,
};

/// Transport used to receive a scan result while a trajectory is playing.
enum class ScanTransport
{
  kTopic,
  kService,
};

enum class ScanOutcome
{
  kNotRequested,
  kDetected,
  kNotDetected,
  kTimeout,
  kError,
};

struct RobotSdkOptions
{
  Commander::Options commander{};

  /// Empty uses $HOME/.ros/zhishu_robot_sdk_cpp/trajectories.
  std::string trajectory_directory;
  /// Empty uses $HOME/.ros/zhishu_robot_sdk_cpp/home_pose.json.
  std::string home_pose_file;

  /// Configurable default scan endpoint names. Empty means the caller must provide a name
  /// in ScanOptions; names are never hard-coded by the implementation.
  std::string scan_topic_name;
  std::string scan_service_name;
};

struct ConnectionStatus
{
  // `connected` means a future backend connect/readiness probe completed successfully; it is
  // deliberately false while that probe is not implemented.
  bool connected{false};
  // This reports only the ROS context, not MoveIt action-server/controller readiness.
  bool ros_context_available{false};
  bool motion_interface_ready{false};
  std::string detail;
};

/// Relative Cartesian offset in the robot base frame. XYZ use millimetres; ABC use degrees.
struct StepOffset
{
  double x_mm{0.0};
  double y_mm{0.0};
  double z_mm{0.0};
  double a_deg{0.0};
  double b_deg{0.0};
  double c_deg{0.0};
};

struct RecordingOptions
{
  /// One arm or both arms. For kBoth, left/right JSON files share a common time origin.
  ArmGroup arms{ArmGroup::kBoth};
  /// AR3 sampling frequency; defaults to 30 Hz.
  double sample_rate_hz{30.0};
  /// Append a sample when max absolute joint delta exceeds this threshold, in radians.
  double joint_change_threshold_rad{0.002};
  bool capture_joint_positions{true};
  /// Optional SDK extension metadata; the required AR3 fields remain unchanged.
  bool capture_tcp_poses{false};
};

struct RecordingStatus
{
  bool active{false};
  bool segment_active{false};
  ArmGroup arms{ArmGroup::kBoth};
  std::size_t segment_count{0};
  std::size_t waypoint_count{0};
  std::string detail;
};

/// One logical named trajectory. AR3-compatible files are stored per arm:
/// traj_L_<name>.json and/or traj_R_<name>.json. For a paired dual-arm trajectory both paths
/// are populated and each file keeps the legacy dof/hz/arm/name/created/points_data keys.
struct TrajectoryInfo
{
  std::string name;
  ArmGroup arms{ArmGroup::kBoth};
  std::size_t segment_count{0};
  std::size_t waypoint_count{0};
  std::string created_at;
  std::string left_file_path;
  std::string right_file_path;
};

struct TrajectoryPlaybackOptions
{
  /// Speed scaling percentage in [0, 100]. Zero is not a valid active playback speed;
  /// callers should use stopTrajectoryPlayback() to stop motion.
  double speed_percent{50.0};
  /// kLeft/kRight plays one arm; kBoth pairs AR3 traj_L_<name>.json and traj_R_<name>.json.
  ArmGroup arms{ArmGroup::kBoth};
  std::chrono::milliseconds motion_timeout{std::chrono::minutes(5)};
};

struct ScanOptions
{
  ScanTransport transport{ScanTransport::kTopic};
  /// Optional per-call endpoint override. If empty, use topic_name/service_name below, then
  /// RobotSdkOptions::scan_topic_name/scan_service_name. No endpoint is hard-coded.
  std::string endpoint_name;
  std::string topic_name;
  std::string service_name;
  /// For std_msgs/msg/String, empty accepts any non-empty code. If non-empty, the code must match.
  /// For std_srvs/srv/Trigger, response.success indicates a scan hit and response.message may
  /// carry the code used for success_code comparison.
  std::string success_code;
  bool stop_on_success{true};
  std::chrono::milliseconds poll_period{std::chrono::milliseconds(200)};
};

struct TrajectoryPlaybackStatus
{
  bool active{false};
  bool completed{false};
  bool stopped_by_scan{false};
  double progress_percent{0.0};
  ScanOutcome scan_outcome{ScanOutcome::kNotRequested};
  std::string scanned_code;
  Result result{Result::failure(ErrorCode::kNotReady, "playback status is unavailable")};
};

struct VacuumOptions
{
  /// Target vacuum percentage. AR3 example accepts values strictly between 0 and 100.
  double vacuum_percent{60.0};
  /// Maximum time to wait for an acquired/secured workpiece.
  std::chrono::milliseconds acquisition_timeout{std::chrono::seconds(10)};
};

struct VacuumStatus
{
  bool supported{false};
  bool active{false};
  bool workpiece_secured{false};
  bool leak_detected{false};
  double measured_vacuum_percent{0.0};
  int fault_code{0};
  std::string message;
};

/// Unified customer-facing C++ API combining the original motion SDK with Excel workflow APIs.
///
/// Contract for the newly added workflow APIs:
/// - connect()/disconnect() will check/release SDK communication only; they never switch motor power.
/// - Software Jog/teaching uses bounded MoveIt increments; physical-button drag is reserved.
/// - setHomePose()/moveHome() manage one unified dual-arm joint pose.
/// - Trajectory JSON preserves AR3 fields dof, hz, arm, name, created, points_data; each point
///   contains t (seconds) and q (joint positions in radians). Left/right use separate files.
/// - speed_percent is a percentage in [0, 100], not the original AR3 raw speed number.
/// - Scan Topic uses std_msgs/msg/String and Scan Service uses std_srvs/srv/Trigger; names are configurable.
/// - Newly added workflow and vendor-inspired APIs remain placeholders returning kNotReady/defaults.
///   The pre-existing motion APIs retain their existing implementations.

class ZHISHU_ROBOT_SDK_PUBLIC RobotSdk
{
public:
  explicit RobotSdk(const rclcpp::Node::SharedPtr & node);
  RobotSdk(const rclcpp::Node::SharedPtr & node, const Commander::Options & options);
  RobotSdk(const rclcpp::Node::SharedPtr & node, const RobotSdkOptions & options);
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

  // Unified customer-facing workflow APIs.
  // 1-2. Connection management. These APIs do not enable/disable servos or motor power.
  Result connect();
  Result disconnect();
  bool isConnected() const noexcept;
  ConnectionStatus getConnectionStatus() const;

  // 3. Software teaching and jog. arm must be kLeft or kRight for jog commands.
  Result startDragTeach(
    ArmGroup arms = ArmGroup::kBoth,
    DragTeachMode mode = DragTeachMode::kSoftwareJog);
  Result stopDragTeach(ArmGroup arms = ArmGroup::kBoth);
  Result jogCartesian(ArmGroup arm, const StepOffset & offset);
  /// Joint deltas are radians; expected vector size is seven for a single arm.
  Result jogJoints(ArmGroup arm, const std::vector<double> & delta_rad);
  Result stopJog();

  // 4-5. Unified dual-arm home pose.
  Result setHomePose();
  Result moveHome();

  // 6. Software sample recording. Pausing/finishing a segment does not clear accumulated points.
  Result startTrajectoryRecording(const RecordingOptions & options = RecordingOptions{});
  Result finishTrajectorySegment(bool continue_recording);
  Result pauseTrajectoryRecording();
  Result resumeTrajectoryRecording();
  Result saveRecordedTrajectory(const std::string & name, bool overwrite = false);
  Result cancelTrajectoryRecording();
  RecordingStatus getRecordingStatus() const;

  // 7 and 9. Start trajectory playback; monitor it through getTrajectoryPlaybackStatus().
  Result playTrajectory(
    const std::string & name,
    const TrajectoryPlaybackOptions & playback_options = TrajectoryPlaybackOptions{});
  Result playTrajectoryWithScan(
    const std::string & name,
    const TrajectoryPlaybackOptions & playback_options,
    const ScanOptions & scan_options = ScanOptions{});
  Result stopTrajectoryPlayback();
  TrajectoryPlaybackStatus getTrajectoryPlaybackStatus() const;

  // 8. Clear one logical trajectory or all trajectories. All-delete requires confirmed=true.
  // Prefer the Result overload so an empty directory can be distinguished from an unavailable
  // backend. The vector-only overload is retained for source compatibility and returns empty
  // when the listing backend is unavailable.
  Result listTrajectories(std::vector<TrajectoryInfo> & trajectories) const;
  std::vector<TrajectoryInfo> listTrajectories() const;
  Result deleteTrajectory(const std::string & name, ArmGroup arms = ArmGroup::kBoth);
  Result clearAllTrajectories(bool confirmed);

  // 10. Bounded XYZABC increment in the base frame; do not pass kBoth for this single-arm API.
  Result moveStep(ArmGroup arm, const StepOffset & offset);

  // 11-12. Abstract vacuum interface only; hardware support is not integrated in this skeleton.
  Result vacuumOn(const VacuumOptions & options, VacuumStatus & status);
  Result vacuumOff();
  VacuumStatus getVacuumStatus() const;

  // Common vendor-SDK capabilities exposed as normalized APIs. These declarations are
  // intentionally stubs in the first phase; no vendor protocol is copied into this package.
  // Speed is a percentage in [0, 100], matching the new SDK contract rather than raw vendor units.
  Result setSpeedScale(double percent);
  Result getSpeedScale(double & percent) const;

  // Tool transform is flange_T_tool (metres + quaternion). Work frame is base_T_workobject.
  Result setToolFrame(
    ArmGroup arm, const std::string & name, const geometry_msgs::msg::Pose & flange_to_tool);
  Result getToolFrame(
    ArmGroup arm, const std::string & name, geometry_msgs::msg::Pose & flange_to_tool) const;
  Result listToolFrames(ArmGroup arm, std::vector<std::string> & names) const;
  Result deleteToolFrame(ArmGroup arm, const std::string & name);
  Result setWorkFrame(
    const std::string & name, const geometry_msgs::msg::PoseStamped & base_to_workobject);
  Result getWorkFrame(
    const std::string & name, geometry_msgs::msg::PoseStamped & base_to_workobject) const;
  Result listWorkFrames(std::vector<std::string> & names) const;
  Result deleteWorkFrame(const std::string & name);

  // Normalized state APIs. Backend-specific controller/error protocol mapping is deferred.
  Result getRobotVersion(RobotVersionInfo & version) const;
  Result getErrorCode(RobotErrorInfo & error) const;
  Result readIO(const IOAddress & address, IOValue & value) const;
  Result writeIO(const IOAddress & address, double value);

private:
  using Operation = std::function<Result(Commander &)>;

  Result runSynchronous(const std::string & operation_name, Operation operation);
  MotionHandle submit(std::string operation_name, Operation operation);
  MotionHandle rejectedHandle(std::string operation_name, Result result);
  void reapFinishedWorker();
  static std::string armOperationName(ArmGroup group, const std::string & operation);
  static std::string gripperOperationName(
    GripperGroup group, const std::string & operation);

  struct Impl;
  Result notImplemented(const char * api_name) const;

  rclcpp::Node::SharedPtr node_;
  RobotSdkOptions options_;
  std::unique_ptr<Impl> impl_;
  std::shared_ptr<Commander> commander_;
  mutable std::mutex task_mutex_;
  MotionHandle active_motion_;
  std::thread worker_;
  bool synchronous_busy_{false};
  std::atomic<std::uint64_t> next_motion_id_{1};
};

}  // namespace zhishu_robot_sdk

#endif  // ZHISHU_ROBOT_SDK__ROBOT_SDK_HPP_
