#include "zhishu_robot_sdk/robot_sdk.hpp"

#include <string>

namespace zhishu_robot_sdk
{

Result RobotSdk::notImplemented(const char * api_name) const
{
  return Result::failure(
    ErrorCode::kNotReady,
    std::string(api_name) + " is declared but its behavior is not implemented yet");
}

Result RobotSdk::connect()
{
  return notImplemented("connect");
}

Result RobotSdk::disconnect()
{
  return notImplemented("disconnect");
}

bool RobotSdk::isConnected() const noexcept
{
  return false;
}

ConnectionStatus RobotSdk::getConnectionStatus() const
{
  ConnectionStatus status;
  status.ros_context_available = static_cast<bool>(node_) && rclcpp::ok();
  // Do not infer a connection or motion readiness from the local Commander object. The
  // MoveIt action-server/controller readiness probe is still pending.
  status.connected = false;
  status.motion_interface_ready = false;
  if (!node_) {
    status.detail = "SDK has no ROS node; connection readiness is unavailable";
  } else if (!rclcpp::ok()) {
    status.detail = "ROS context is not initialized or has been shut down";
  } else {
    status.detail = "ROS context is available; MoveIt action/controller readiness probing is not implemented";
  }
  return status;
}

Result RobotSdk::startDragTeach(ArmGroup, DragTeachMode)
{
  return notImplemented("startDragTeach");
}

Result RobotSdk::stopDragTeach(ArmGroup)
{
  return notImplemented("stopDragTeach");
}

Result RobotSdk::jogCartesian(ArmGroup, const StepOffset &)
{
  return notImplemented("jogCartesian");
}

Result RobotSdk::jogJoints(ArmGroup, const std::vector<double> &)
{
  return notImplemented("jogJoints");
}

Result RobotSdk::stopJog()
{
  return notImplemented("stopJog");
}

Result RobotSdk::setHomePose()
{
  return notImplemented("setHomePose");
}

Result RobotSdk::moveHome()
{
  return notImplemented("moveHome");
}

Result RobotSdk::startTrajectoryRecording(const RecordingOptions &)
{
  return notImplemented("startTrajectoryRecording");
}

Result RobotSdk::finishTrajectorySegment(bool)
{
  return notImplemented("finishTrajectorySegment");
}

Result RobotSdk::pauseTrajectoryRecording()
{
  return notImplemented("pauseTrajectoryRecording");
}

Result RobotSdk::resumeTrajectoryRecording()
{
  return notImplemented("resumeTrajectoryRecording");
}

Result RobotSdk::saveRecordedTrajectory(const std::string &, bool)
{
  return notImplemented("saveRecordedTrajectory");
}

Result RobotSdk::cancelTrajectoryRecording()
{
  return notImplemented("cancelTrajectoryRecording");
}

RecordingStatus RobotSdk::getRecordingStatus() const
{
  RecordingStatus status;
  status.detail = "API skeleton: trajectory recording status is not implemented";
  return status;
}

Result RobotSdk::playTrajectory(const std::string &, const TrajectoryPlaybackOptions &)
{
  return notImplemented("playTrajectory");
}

Result RobotSdk::playTrajectoryWithScan(
  const std::string &, const TrajectoryPlaybackOptions &, const ScanOptions &)
{
  return notImplemented("playTrajectoryWithScan");
}

Result RobotSdk::stopTrajectoryPlayback()
{
  return notImplemented("stopTrajectoryPlayback");
}

TrajectoryPlaybackStatus RobotSdk::getTrajectoryPlaybackStatus() const
{
  TrajectoryPlaybackStatus status;
  status.result = notImplemented("getTrajectoryPlaybackStatus");
  return status;
}

Result RobotSdk::listTrajectories(std::vector<TrajectoryInfo> & trajectories) const
{
  trajectories.clear();
  return notImplemented("listTrajectories");
}

std::vector<TrajectoryInfo> RobotSdk::listTrajectories() const
{
  std::vector<TrajectoryInfo> trajectories;
  (void)listTrajectories(trajectories);
  return trajectories;
}

Result RobotSdk::deleteTrajectory(const std::string &, ArmGroup)
{
  return notImplemented("deleteTrajectory");
}

Result RobotSdk::clearAllTrajectories(bool)
{
  return notImplemented("clearAllTrajectories");
}

Result RobotSdk::moveStep(ArmGroup, const StepOffset &)
{
  return notImplemented("moveStep");
}

Result RobotSdk::vacuumOn(const VacuumOptions &, VacuumStatus & status)
{
  status = VacuumStatus{};
  status.message = "API skeleton: vacuum hardware interface is not implemented";
  return notImplemented("vacuumOn");
}

Result RobotSdk::vacuumOff()
{
  return notImplemented("vacuumOff");
}

VacuumStatus RobotSdk::getVacuumStatus() const
{
  VacuumStatus status;
  status.message = "API skeleton: vacuum hardware interface is not implemented";
  return status;
}


Result RobotSdk::setSpeedScale(double)
{
  return notImplemented("setSpeedScale");
}

Result RobotSdk::getSpeedScale(double & percent) const
{
  percent = 0.0;
  return notImplemented("getSpeedScale");
}

Result RobotSdk::setToolFrame(ArmGroup, const std::string &, const geometry_msgs::msg::Pose &)
{
  return notImplemented("setToolFrame");
}

Result RobotSdk::getToolFrame(
  ArmGroup, const std::string &, geometry_msgs::msg::Pose & flange_to_tool) const
{
  flange_to_tool = geometry_msgs::msg::Pose{};
  return notImplemented("getToolFrame");
}

Result RobotSdk::listToolFrames(ArmGroup, std::vector<std::string> & names) const
{
  names.clear();
  return notImplemented("listToolFrames");
}

Result RobotSdk::deleteToolFrame(ArmGroup, const std::string &)
{
  return notImplemented("deleteToolFrame");
}

Result RobotSdk::setWorkFrame(const std::string &, const geometry_msgs::msg::PoseStamped &)
{
  return notImplemented("setWorkFrame");
}

Result RobotSdk::getWorkFrame(
  const std::string &, geometry_msgs::msg::PoseStamped & base_to_workobject) const
{
  base_to_workobject = geometry_msgs::msg::PoseStamped{};
  return notImplemented("getWorkFrame");
}

Result RobotSdk::listWorkFrames(std::vector<std::string> & names) const
{
  names.clear();
  return notImplemented("listWorkFrames");
}

Result RobotSdk::deleteWorkFrame(const std::string &)
{
  return notImplemented("deleteWorkFrame");
}

Result RobotSdk::getRobotVersion(RobotVersionInfo & version) const
{
  version = RobotVersionInfo{};
  return notImplemented("getRobotVersion");
}

Result RobotSdk::getErrorCode(RobotErrorInfo & error) const
{
  error = RobotErrorInfo{};
  return notImplemented("getErrorCode");
}

Result RobotSdk::readIO(const IOAddress & address, IOValue & value) const
{
  value = IOValue{};
  value.address = address;
  return notImplemented("readIO");
}

Result RobotSdk::writeIO(const IOAddress &, double)
{
  return notImplemented("writeIO");
}


}  // namespace zhishu_robot_sdk
