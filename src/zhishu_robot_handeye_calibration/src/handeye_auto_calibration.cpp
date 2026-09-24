#include "zhishu_robot_commander_cpp/sdk.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <moveit/handeye_calibration_solver/handeye_solver_opencv.h>
#include <moveit/handeye_calibration_target/handeye_target_charuco.h>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Geometry>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using namespace std::chrono_literals;
using Pose = std::array<double, 7>;

const std::array<Pose, 12> kCalibrationPoses{{
  {{-1.8010, -0.0919, -1.4576, 0.2305, 1.1331, 0.7153, -0.4490}},
  {{-1.8010, -0.0919, -1.4576, 0.3000, 1.0000, 0.6000, -0.3000}},
  {{-2.0822, -0.0729, -1.5280, 0.2000, 0.4000, 0.0500, -0.5000}},
  {{-2.0822, -0.0729, -1.5280, 0.1200, 0.2256, -0.0796, -0.6282}},
  {{-1.7652, 0.0691, -0.8699, 0.2000, 1.1000, 0.9000, -1.0500}},
  {{-1.7652, 0.0691, -0.8699, 0.1200, 0.9859, 1.0126, -1.2000}},
  {{-1.7240, -0.0294, -1.3465, 0.0935, 1.8382, 0.0301, -1.0577}},
  {{-1.6961, -0.3052, -0.1272, 0.2631, 1.1635, 0.0212, -0.6832}},
  {{-1.5452, -0.1728, -0.0730, 0.8000, 1.5500, 0.5000, -0.1000}},
  {{-1.5452, -0.1728, -0.0730, 0.7150, 1.6793, 0.6521, -0.2634}},
  {{-1.3670, 0.0620, -0.8758, 1.0500, 1.3500, -0.0500, 0.2000}},
  {{-1.3670, 0.0620, -0.8758, 0.9184, 1.4938, -0.1578, 0.3599}},
}};

void importMoveItDescriptions(const rclcpp::Node::SharedPtr & node)
{
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(20s)) {
    throw std::runtime_error("/move_group parameter service is unavailable");
  }
  const std::vector<std::string> names{"robot_description", "robot_description_semantic"};
  auto future = client->get_parameters(names);
  if (future.wait_for(10s) != std::future_status::ready) {
    throw std::runtime_error("timed out reading MoveIt robot descriptions");
  }
  const auto parameters = future.get();
  if (parameters.size() != names.size()) {
    throw std::runtime_error("failed to read MoveIt robot descriptions");
  }
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (parameters[index].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      parameters[index].as_string().empty())
    {
      throw std::runtime_error("MoveIt parameter is empty: " + names[index]);
    }
    if (node->has_parameter(names[index])) {
      node->set_parameter(parameters[index]);
    } else {
      node->declare_parameter<std::string>(names[index], parameters[index].as_string());
    }
  }
}

double rotationError(const Eigen::Isometry3d & estimate, const Eigen::Isometry3d & truth)
{
  return Eigen::AngleAxisd(estimate.rotation().transpose() * truth.rotation()).angle();
}

bool isValidTransform(const Eigen::Isometry3d & transform)
{
  const auto & matrix = transform.matrix();
  const Eigen::Matrix3d rotation = transform.rotation();
  return matrix.allFinite() &&
         (matrix.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() < 1e-9 &&
         (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
         std::abs(rotation.determinant() - 1.0) < 1e-6;
}

std::string yamlQuote(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      escaped.push_back('\\');
    }
    escaped.push_back(character == '\n' ? ' ' : character);
  }
  return '"' + escaped + '"';
}

void writeFailureFile(const std::string & path, const std::string & reason, std::size_t samples)
{
  const auto parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("cannot open failure result file: " + path);
  }
  output << "status: FAIL\nreason: " << yamlQuote(reason) <<
    "\nvalid_samples: " << samples << "\n";
}

std::string matrixYaml(const Eigen::Isometry3d & transform, const std::string & indent)
{
  std::ostringstream stream;
  stream << std::setprecision(12);
  for (int row = 0; row < 4; ++row) {
    stream << indent << "- [";
    for (int col = 0; col < 4; ++col) {
      if (col != 0) {
        stream << ", ";
      }
      stream << transform.matrix()(row, col);
    }
    stream << "]\n";
  }
  return stream.str();
}

struct SolverResult
{
  std::string name;
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
  double reprojection_translation{std::numeric_limits<double>::quiet_NaN()};
  double reprojection_rotation{std::numeric_limits<double>::quiet_NaN()};
  double truth_translation{std::numeric_limits<double>::quiet_NaN()};
  double truth_rotation{std::numeric_limits<double>::quiet_NaN()};
};
}  // namespace

class AutomaticHandEyeCalibration
{
public:
  explicit AutomaticHandEyeCalibration(const rclcpp::Node::SharedPtr & node)
  : node_(node), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_)
  {
    image_topic_ = node_->declare_parameter<std::string>(
      "image_topic", "/camera_head/color/image_raw");
    camera_info_topic_ = node_->declare_parameter<std::string>(
      "camera_info_topic", "/camera_head/color/camera_info");
    output_file_ = node_->declare_parameter<std::string>(
      "output_file", "/tmp/handeye_calibration_result.yaml");
    image_directory_ = node_->declare_parameter<std::string>(
      "image_directory", "/tmp/handeye_calibration_samples");
    moveit_samples_file_ = node_->declare_parameter<std::string>(
      "moveit_samples_file", "/tmp/handeye_moveit_samples.yaml");
    truth_sensor_frame_ = node_->declare_parameter<std::string>(
      "truth_sensor_frame", "head_camera_optical_truth");
    minimum_markers_ = node_->declare_parameter<int>("minimum_markers", 3);
    minimum_samples_ = node_->declare_parameter<int>("minimum_samples", 5);
    sample_attempts_per_pose_ = node_->declare_parameter<int>("sample_attempts_per_pose", 3);
    settle_seconds_ = node_->declare_parameter<double>("settle_seconds", 1.0);
    head_yaw_target_ = node_->declare_parameter<double>("head_yaw_target", 0.5);
    head_pitch_target_ = node_->declare_parameter<double>("head_pitch_target", -0.15);
    head_tolerance_ = node_->declare_parameter<double>("head_tolerance", 0.002);
    translation_tolerance_ = node_->declare_parameter<double>(
      "translation_tolerance", 0.03);
    rotation_tolerance_ = node_->declare_parameter<double>(
      "rotation_tolerance", 0.05);
    reprojection_translation_tolerance_ = node_->declare_parameter<double>(
      "reprojection_translation_tolerance", 0.02);
    reprojection_rotation_tolerance_ = node_->declare_parameter<double>(
      "reprojection_rotation_tolerance", 0.03);

    detector_.setParameter("squares, X", 7);
    detector_.setParameter("squares, Y", 5);
    detector_.setParameter("marker size (px)", 220);
    detector_.setParameter("square size (px)", 300);
    detector_.setParameter("margin size (px)", 0);
    detector_.setParameter("marker border (bits)", 1);
    detector_.setParameter("ArUco dictionary", std::string("DICT_4X4_50"));
    detector_.setParameter("longest board side (m)", 0.21);
    detector_.setParameter("measured marker size (m)", 0.022);
    if (!detector_.initialize()) {
      throw std::runtime_error("failed to initialize the ChArUco target detector");
    }

    sample_markers_publisher_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/handeye_calibration/sample_markers", rclcpp::QoS(1).transient_local().reliable());
    annotated_image_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(
      "/handeye_calibration/annotated_image", rclcpp::SensorDataQoS());
    auto_load_samples_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      "/handeye_calibration/auto_load_samples", rclcpp::QoS(1).transient_local().reliable());

    image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_image_ = std::move(message);
        ++image_sequence_;
        data_condition_.notify_all();
      });
    camera_info_subscription_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_camera_info_ = std::move(message);
        data_condition_.notify_all();
      });
    joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (std::size_t index = 0; index < message->name.size() &&
          index < message->position.size(); ++index)
        {
          if (message->name[index] == "head_yaw_joint") {
            head_yaw_ = message->position[index];
          } else if (message->name[index] == "head_pitch_joint") {
            head_pitch_ = message->position[index];
          }
        }
        data_condition_.notify_all();
      });
  }

  bool run()
  {
    importMoveItDescriptions(node_);
    waitForCamera();
    requireHeadLocked("before calibration");
    std::filesystem::create_directories(image_directory_);

    zhishu_robot_commander_cpp::Commander::Options options;
    options.left_end_effector_link = "L_hand_link";
    options.velocity_scaling = 0.15;
    options.acceleration_scaling = 0.10;
    options.planning_time = 10.0;
    options.planning_attempts = 10;
    zhishu_robot_commander_cpp::RobotSdk robot(node_, options);

    RCLCPP_INFO(
      node_->get_logger(), "Starting automatic Eye-to-Hand calibration with %zu poses",
      kCalibrationPoses.size());
    publishSampleVisualization("Collecting samples: 0 / 12");

    for (std::size_t index = 0; index < kCalibrationPoses.size(); ++index) {
      const auto sequence_before_move = currentImageSequence();
      const auto & pose = kCalibrationPoses[index];
      const std::vector<double> joints(pose.begin(), pose.end());
      const auto result = robot.moveJoints(zhishu_robot_commander_cpp::ArmGroup::kLeft, joints);
      if (!result) {
        RCLCPP_WARN(
          node_->get_logger(), "Pose %zu motion failed [%s]: %s; skipping", index + 1,
          zhishu_robot_commander_cpp::toString(result.code), result.message.c_str());
        continue;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(settle_seconds_));
      requireHeadLocked("after moving to pose " + std::to_string(index + 1));

      auto previous_sequence = sequence_before_move;
      bool accepted = false;
      for (int attempt = 1; attempt <= std::max(1, sample_attempts_per_pose_); ++attempt) {
        sensor_msgs::msg::Image::ConstSharedPtr image_message;
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;
        if (!waitForFreshFrame(previous_sequence, image_message, camera_info)) {
          RCLCPP_WARN(
            node_->get_logger(), "Pose %zu attempt %d has no fresh camera frame",
            index + 1, attempt);
          continue;
        }
        previous_sequence = currentImageSequence();
        requireHeadLocked("while sampling pose " + std::to_string(index + 1));
        if (takeSample(index, image_message, camera_info)) {
          accepted = true;
          break;
        }
      }
      if (!accepted) {
        RCLCPP_WARN(
          node_->get_logger(), "Pose %zu was rejected after %d sample attempts",
          index + 1, std::max(1, sample_attempts_per_pose_));
      }
    }

    if (effector_wrt_world_.size() < static_cast<std::size_t>(minimum_samples_)) {
      RCLCPP_ERROR(
        node_->get_logger(), "Only %zu valid samples were collected; at least %d are required",
        effector_wrt_world_.size(), minimum_samples_);
      writeFailure("insufficient valid samples");
      return false;
    }

    // Do not use head_camera_optical_frame as simulation truth here.  The
    // MoveIt HandEye RViz context publishes its current pose guess on that
    // frame (identity before solving), which would make the validation compare
    // against the GUI guess instead of the robot model.  The coincident *_truth
    // frame is owned only by robot_state_publisher.
    const Eigen::Isometry3d truth = lookupTransform("base_link", truth_sensor_frame_);
    const auto results = solveAll(truth);
    if (results.empty()) {
      writeFailure("all solvers failed");
      return false;
    }
    const auto best = std::min_element(
      results.begin(), results.end(), [this](const auto & left, const auto & right) {
        const double left_score =
          left.reprojection_translation / reprojection_translation_tolerance_ +
          left.reprojection_rotation / reprojection_rotation_tolerance_;
        const double right_score =
          right.reprojection_translation / reprojection_translation_tolerance_ +
          right.reprojection_rotation / reprojection_rotation_tolerance_;
        return left_score < right_score;
      });
    const bool passed = best->truth_translation <= translation_tolerance_ &&
      best->truth_rotation <= rotation_tolerance_ &&
      best->reprojection_translation <= reprojection_translation_tolerance_ &&
      best->reprojection_rotation <= reprojection_rotation_tolerance_;
    writeResults(results, *best, truth, passed);
    writeMoveItSamples();
    publishMoveItAutoLoadRequest(best->name, true);

    RCLCPP_INFO_STREAM(
      node_->get_logger(), "Calibration result (base_link -> head_camera_optical_frame):\n" <<
        best->transform.matrix());
    RCLCPP_INFO(
      node_->get_logger(),
      "Best solver=%s, samples=%zu, reprojection=(%.6f m, %.6f rad), truth error=(%.6f m, %.6f rad), result=%s, output=%s",
      best->name.c_str(), effector_wrt_world_.size(), best->reprojection_translation,
      best->reprojection_rotation, best->truth_translation, best->truth_rotation,
      passed ? "PASS" : "FAIL", output_file_.c_str());
    publishSampleVisualization(
      std::string(passed ? "PASS" : "FAIL") + " | " + best->name + " | samples " +
      std::to_string(effector_wrt_world_.size()) + " / " +
      std::to_string(kCalibrationPoses.size()));
    return passed;
  }

private:
  void waitForCamera()
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    if (!data_condition_.wait_for(
        lock, 20s, [this] {return latest_image_ && latest_camera_info_;}))
    {
      throw std::runtime_error("camera image or CameraInfo was not received within 20 seconds");
    }
  }

  void requireHeadLocked(const std::string & phase)
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    if (!data_condition_.wait_for(
        lock, 10s, [this] {return head_yaw_.has_value() && head_pitch_.has_value();}))
    {
      throw std::runtime_error("head joint state was not received " + phase);
    }
    const double yaw_error = std::abs(*head_yaw_ - head_yaw_target_);
    const double pitch_error = std::abs(*head_pitch_ - head_pitch_target_);
    if (yaw_error > head_tolerance_ || pitch_error > head_tolerance_) {
      std::ostringstream message;
      message << "head lock violated " << phase << ": yaw=" << *head_yaw_ <<
        " (target " << head_yaw_target_ << "), pitch=" << *head_pitch_ <<
        " (target " << head_pitch_target_ << "), tolerance=" << head_tolerance_;
      throw std::runtime_error(message.str());
    }
  }

  std::uint64_t currentImageSequence()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return image_sequence_;
  }

  static void setMarkerPose(
    visualization_msgs::msg::Marker & marker, const Eigen::Isometry3d & transform)
  {
    marker.pose.position.x = transform.translation().x();
    marker.pose.position.y = transform.translation().y();
    marker.pose.position.z = transform.translation().z();
    Eigen::Quaterniond quaternion(transform.rotation());
    quaternion.normalize();
    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();
    marker.pose.orientation.w = quaternion.w();
  }

  void publishAnnotatedImage(
    const sensor_msgs::msg::Image::ConstSharedPtr & source, const cv::Mat & gray,
    const std::vector<std::vector<cv::Point2f>> & corners, const std::vector<int> & ids)
  {
    cv::Mat annotated;
    if (gray.channels() == 1) {
      cv::cvtColor(gray, annotated, cv::COLOR_GRAY2BGR);
    } else {
      annotated = gray.clone();
    }
    cv::aruco::drawDetectedMarkers(annotated, corners, ids);
    auto message = cv_bridge::CvImage(source->header, "bgr8", annotated).toImageMsg();
    annotated_image_publisher_->publish(*message);
  }

  void publishSampleVisualization(const std::string & status)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    const auto stamp = node_->now();
    const std::array<Eigen::Matrix3d, 3> axis_rotations{
      Eigen::Matrix3d::Identity(),
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
      Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix()};

    for (std::size_t sample = 0; sample < detected_targets_in_camera_.size(); ++sample) {
      for (std::size_t axis = 0; axis < axis_rotations.size(); ++axis) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "head_camera_optical_frame";
        marker.header.stamp = stamp;
        marker.ns = "handeye_sample_axes";
        marker.id = static_cast<int>(sample * 3 + axis);
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        Eigen::Isometry3d axis_pose = detected_targets_in_camera_[sample];
        axis_pose.linear() = axis_pose.rotation() * axis_rotations[axis];
        setMarkerPose(marker, axis_pose);
        marker.scale.x = 0.08;
        marker.scale.y = 0.012;
        marker.scale.z = 0.012;
        marker.color.a = 0.95F;
        marker.color.r = axis == 0 ? 1.0F : 0.0F;
        marker.color.g = axis == 1 ? 1.0F : 0.0F;
        marker.color.b = axis == 2 ? 1.0F : 0.0F;
        marker.frame_locked = true;
        markers.markers.push_back(marker);
      }

      visualization_msgs::msg::Marker label;
      label.header.frame_id = "head_camera_optical_frame";
      label.header.stamp = stamp;
      label.ns = "handeye_sample_labels";
      label.id = static_cast<int>(sample);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      Eigen::Isometry3d label_pose = detected_targets_in_camera_[sample];
      label_pose.translate(Eigen::Vector3d(0.0, 0.0, 0.07));
      setMarkerPose(label, label_pose);
      label.scale.z = 0.035;
      label.color.r = 1.0F;
      label.color.g = 1.0F;
      label.color.b = 1.0F;
      label.color.a = 1.0F;
      label.text = "S" + std::to_string(sample + 1) + " (" +
        std::to_string(marker_counts_[sample]) + " markers)";
      label.frame_locked = true;
      markers.markers.push_back(label);
    }

    visualization_msgs::msg::Marker progress;
    progress.header.frame_id = "base_link";
    progress.header.stamp = stamp;
    progress.ns = "handeye_progress";
    progress.id = 0;
    progress.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    progress.action = visualization_msgs::msg::Marker::ADD;
    progress.pose.position.x = -0.35;
    progress.pose.position.z = 1.65;
    progress.pose.orientation.w = 1.0;
    progress.scale.z = 0.07;
    progress.color.r = 1.0F;
    progress.color.g = 0.85F;
    progress.color.b = 0.1F;
    progress.color.a = 1.0F;
    progress.text = status;
    progress.frame_locked = true;
    markers.markers.push_back(progress);

    sample_markers_publisher_->publish(markers);
  }

  bool waitForFreshFrame(
    std::uint64_t old_sequence, sensor_msgs::msg::Image::ConstSharedPtr & image,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    const bool ready = data_condition_.wait_for(
      lock, 10s, [this, old_sequence] {
        return image_sequence_ > old_sequence && latest_image_ && latest_camera_info_;
      });
    if (ready) {
      image = latest_image_;
      info = latest_camera_info_;
    }
    return ready;
  }

  Eigen::Isometry3d lookupTransform(const std::string & target, const std::string & source)
  {
    const auto transform = tf_buffer_.lookupTransform(target, source, tf2::TimePointZero, 5s);
    return tf2::transformToEigen(transform);
  }

  bool takeSample(
    std::size_t pose_index, const sensor_msgs::msg::Image::ConstSharedPtr & image_message,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info)
  {
    cv::Mat gray;
    try {
      const auto converted = cv_bridge::toCvCopy(image_message, "mono8");
      gray = converted->image.clone();
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_WARN(node_->get_logger(), "Pose %zu image conversion failed: %s", pose_index + 1, error.what());
      return false;
    }

    const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray, dictionary, corners, ids);
    if (ids.size() < static_cast<std::size_t>(minimum_markers_)) {
      RCLCPP_WARN(
        node_->get_logger(), "Pose %zu sees %zu markers (minimum %d); skipping",
        pose_index + 1, ids.size(), minimum_markers_);
      return false;
    }

    if (!detector_.setCameraIntrinsicParams(camera_info) || !detector_.detectTargetPose(gray)) {
      RCLCPP_WARN(node_->get_logger(), "Pose %zu target pose estimation failed; skipping", pose_index + 1);
      return false;
    }

    try {
      const auto base_to_eef_message = tf_buffer_.lookupTransform(
        "base_link", "L_hand_link", rclcpp::Time(image_message->header.stamp), 5s);
      const Eigen::Isometry3d base_to_eef = tf2::transformToEigen(base_to_eef_message);
      const auto detected = detector_.getTransformStamped("head_camera_optical_frame");
      const Eigen::Isometry3d camera_to_object = tf2::transformToEigen(detected);

      effector_wrt_world_.push_back(base_to_eef);
      object_wrt_sensor_.push_back(camera_to_object);
      detected_targets_in_camera_.push_back(camera_to_object);
      marker_counts_.push_back(ids.size());
      const std::string image_path = image_directory_ + "/sample_" +
        (pose_index + 1 < 10 ? "0" : "") + std::to_string(pose_index + 1) + ".png";
      if (!cv::imwrite(image_path, gray)) {
        effector_wrt_world_.pop_back();
        object_wrt_sensor_.pop_back();
        detected_targets_in_camera_.pop_back();
        marker_counts_.pop_back();
        RCLCPP_WARN(node_->get_logger(), "Could not write sample image: %s", image_path.c_str());
        return false;
      }
      publishAnnotatedImage(image_message, gray, corners, ids);
      writeMoveItSamples();
      publishMoveItAutoLoadRequest("", false);
      publishSampleVisualization(
        "Collecting samples: " + std::to_string(effector_wrt_world_.size()) + " / " +
        std::to_string(kCalibrationPoses.size()) + " | pose " +
        std::to_string(pose_index + 1) + " | " + std::to_string(ids.size()) + " markers");
      RCLCPP_INFO(
        node_->get_logger(), "Accepted sample %zu from pose %zu with %zu visible markers",
        effector_wrt_world_.size(), pose_index + 1, ids.size());
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(node_->get_logger(), "Pose %zu TF lookup failed: %s", pose_index + 1, error.what());
      return false;
    }
  }

  std::vector<SolverResult> solveAll(const Eigen::Isometry3d & truth)
  {
    moveit_handeye_calibration::HandEyeSolverDefault solver;
    solver.initialize();
    std::vector<SolverResult> results;
    for (const auto & solver_name : solver.getSolverNames()) {
      std::string error;
      if (!solver.solve(
          effector_wrt_world_, object_wrt_sensor_, moveit_handeye_calibration::EYE_TO_HAND,
          solver_name, &error))
      {
        RCLCPP_WARN(node_->get_logger(), "Solver %s failed: %s", solver_name.c_str(), error.c_str());
        continue;
      }
      SolverResult result;
      result.name = solver_name;
      result.transform = solver.getCameraRobotPose();
      // The upstream MoveIt solver builds its OpenCV homogeneous matrix from
      // zeros and leaves (3, 3) unset.  Restore a valid SE(3) bottom row before
      // serializing or comparing the result.
      result.transform.matrix().row(3) << 0.0, 0.0, 0.0, 1.0;
      const auto reprojection = solver.getReprojectionError(
        effector_wrt_world_, object_wrt_sensor_, result.transform,
        moveit_handeye_calibration::EYE_TO_HAND);
      result.reprojection_translation = reprojection.first;
      result.reprojection_rotation = reprojection.second;
      result.truth_translation = (result.transform.translation() - truth.translation()).norm();
      result.truth_rotation = rotationError(result.transform, truth);
      if (std::isfinite(result.reprojection_translation) &&
        std::isfinite(result.reprojection_rotation) && std::isfinite(result.truth_translation) &&
        std::isfinite(result.truth_rotation) && isValidTransform(result.transform))
      {
        results.push_back(result);
      }
    }
    return results;
  }

  void writeFailure(const std::string & reason)
  {
    writeFailureFile(output_file_, reason, effector_wrt_world_.size());
  }

  void writeMoveItSamples()
  {
    const auto parent = std::filesystem::path(moveit_samples_file_).parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }
    std::ofstream output(moveit_samples_file_);
    if (!output) {
      throw std::runtime_error("cannot open MoveIt samples file: " + moveit_samples_file_);
    }
    output << std::setprecision(12);
    for (std::size_t index = 0; index < effector_wrt_world_.size(); ++index) {
      output << "- effector_wrt_world: [";
      for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
          output << (row == 0 && col == 0 ? "" : ", ") <<
            effector_wrt_world_[index].matrix()(row, col);
        }
      }
      output << "]\n  object_wrt_sensor: [";
      for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
          output << (row == 0 && col == 0 ? "" : ", ") <<
            object_wrt_sensor_[index].matrix()(row, col);
        }
      }
      output << "]\n";
    }
  }

  void publishMoveItAutoLoadRequest(const std::string & solver_name, bool auto_solve)
  {
    std_msgs::msg::String request;
    request.data = "request_id: " + std::to_string(node_->now().nanoseconds()) +
      "\nsample_file: " + yamlQuote(moveit_samples_file_) +
      "\nsolver: " + yamlQuote(solver_name) + "\nauto_solve: " +
      (auto_solve ? "true\n" : "false\n");
    // Publish more than once so discovery races at the end of a short-lived
    // calibration process cannot leave the RViz panel empty. The request id
    // lets the plugin de-duplicate these copies.
    const int publish_attempts = auto_solve ? 3 : 1;
    for (int attempt = 0; attempt < publish_attempts; ++attempt) {
      auto_load_samples_publisher_->publish(request);
      if (attempt + 1 < publish_attempts) {
        std::this_thread::sleep_for(200ms);
      }
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "Published native MoveIt sample import request for %zu samples (solve=%s): %s",
      effector_wrt_world_.size(), auto_solve ? "true" : "false", moveit_samples_file_.c_str());
  }

  void writeResults(
    const std::vector<SolverResult> & results, const SolverResult & best,
    const Eigen::Isometry3d & truth, bool passed)
  {
    const auto parent = std::filesystem::path(output_file_).parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }
    std::ofstream output(output_file_);
    if (!output) {
      throw std::runtime_error("cannot open result file: " + output_file_);
    }
    output << std::setprecision(12);
    output << "status: " << (passed ? "PASS" : "FAIL") << "\n";
    output << "calibration_type: eye_to_hand\n";
    output << "parent_frame: base_link\nchild_frame: head_camera_optical_frame\n";
    output << "valid_samples: " << effector_wrt_world_.size() << "\n";
    output << "marker_counts: [";
    for (std::size_t index = 0; index < marker_counts_.size(); ++index) {
      output << (index == 0 ? "" : ", ") << marker_counts_[index];
    }
    output << "]\nselected_solver: " << best.name << "\n";
    output << "thresholds:\n  translation_m: " << translation_tolerance_ <<
      "\n  rotation_rad: " << rotation_tolerance_ <<
      "\n  reprojection_translation_m: " << reprojection_translation_tolerance_ <<
      "\n  reprojection_rotation_rad: " << reprojection_rotation_tolerance_ << "\n";
    output << "selected_result:\n  transform_matrix:\n" << matrixYaml(best.transform, "    ");
    const Eigen::Quaterniond quaternion(best.transform.rotation());
    output << "  translation_xyz_m: [" << best.transform.translation().x() << ", " <<
      best.transform.translation().y() << ", " << best.transform.translation().z() << "]\n";
    output << "  quaternion_xyzw: [" << quaternion.x() << ", " << quaternion.y() << ", " <<
      quaternion.z() << ", " << quaternion.w() << "]\n";
    output << "  reprojection_translation_rmse_m: " << best.reprojection_translation << "\n";
    output << "  reprojection_rotation_rmse_rad: " << best.reprojection_rotation << "\n";
    output << "  truth_translation_error_m: " << best.truth_translation << "\n";
    output << "  truth_rotation_error_rad: " << best.truth_rotation << "\n";
    output << "simulation_truth:\n  parent_frame: base_link\n  child_frame: " <<
      truth_sensor_frame_ << "\n  transform_matrix:\n" << matrixYaml(truth, "    ");
    output << "all_solvers:\n";
    for (const auto & result : results) {
      output << "  - name: " << result.name << "\n";
      output << "    reprojection_translation_rmse_m: " << result.reprojection_translation << "\n";
      output << "    reprojection_rotation_rmse_rad: " << result.reprojection_rotation << "\n";
      output << "    truth_translation_error_m: " << result.truth_translation << "\n";
      output << "    truth_rotation_error_rad: " << result.truth_rotation << "\n";
    }
    output << "samples:\n";
    for (std::size_t index = 0; index < effector_wrt_world_.size(); ++index) {
      output << "  - marker_count: " << marker_counts_[index] << "\n";
      output << "    base_to_effector:\n" << matrixYaml(effector_wrt_world_[index], "      ");
      output << "    camera_to_target:\n" << matrixYaml(object_wrt_sensor_[index], "      ");
    }
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  moveit_handeye_calibration::HandEyeCharucoTarget detector_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sample_markers_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr auto_load_samples_publisher_;
  std::mutex data_mutex_;
  std::condition_variable data_condition_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;
  std::uint64_t image_sequence_{0};
  std::optional<double> head_yaw_;
  std::optional<double> head_pitch_;
  std::vector<Eigen::Isometry3d> effector_wrt_world_;
  std::vector<Eigen::Isometry3d> object_wrt_sensor_;
  std::vector<Eigen::Isometry3d> detected_targets_in_camera_;
  std::vector<std::size_t> marker_counts_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string output_file_;
  std::string image_directory_;
  std::string moveit_samples_file_;
  std::string truth_sensor_frame_;
  int minimum_markers_;
  int minimum_samples_;
  int sample_attempts_per_pose_;
  double settle_seconds_;
  double head_yaw_target_;
  double head_pitch_target_;
  double head_tolerance_;
  double translation_tolerance_;
  double rotation_tolerance_;
  double reprojection_translation_tolerance_;
  double reprojection_rotation_tolerance_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("handeye_auto_calibration");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  int exit_code = 1;
  try {
    AutomaticHandEyeCalibration calibration(node);
    exit_code = calibration.run() ? 0 : 2;
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "Automatic calibration failed: %s", error.what());
    try {
      std::string output_file = "/tmp/handeye_calibration_result.yaml";
      node->get_parameter("output_file", output_file);
      writeFailureFile(output_file, error.what(), 0);
    } catch (const std::exception & write_error) {
      RCLCPP_FATAL(node->get_logger(), "Could not write FAIL result: %s", write_error.what());
    }
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return exit_code;
}
