#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{
constexpr std::uint32_t kMagic = 0x31525651U;  // ASCII "QVR1" in little-endian
constexpr std::uint32_t kVersion = 1U;
constexpr std::size_t kPacketSize = 104U;

struct HandPacket
{
  Eigen::Vector3d position_unity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_unity{Eigen::Quaterniond::Identity()};
  double grip{0.0};
  double trigger{0.0};
  bool valid{false};
};

struct QuestPacket
{
  std::uint32_t sequence{0U};
  double device_time_s{0.0};
  HandPacket left;
  HandPacket right;
};

template<typename T>
bool readLittleEndian(
  const std::array<std::uint8_t, kPacketSize> & buffer,
  std::size_t & offset,
  T & value)
{
  if (offset + sizeof(T) > buffer.size()) {
    return false;
  }
  std::memcpy(&value, buffer.data() + offset, sizeof(T));
  offset += sizeof(T);
  return true;
}

bool readHand(
  const std::array<std::uint8_t, kPacketSize> & buffer,
  std::size_t & offset,
  HandPacket & hand)
{
  float px = 0.0F;
  float py = 0.0F;
  float pz = 0.0F;
  float qx = 0.0F;
  float qy = 0.0F;
  float qz = 0.0F;
  float qw = 1.0F;
  float grip = 0.0F;
  float trigger = 0.0F;
  std::uint32_t valid = 0U;

  if (!readLittleEndian(buffer, offset, px) ||
      !readLittleEndian(buffer, offset, py) ||
      !readLittleEndian(buffer, offset, pz) ||
      !readLittleEndian(buffer, offset, qx) ||
      !readLittleEndian(buffer, offset, qy) ||
      !readLittleEndian(buffer, offset, qz) ||
      !readLittleEndian(buffer, offset, qw) ||
      !readLittleEndian(buffer, offset, grip) ||
      !readLittleEndian(buffer, offset, trigger) ||
      !readLittleEndian(buffer, offset, valid))
  {
    return false;
  }

  hand.position_unity = Eigen::Vector3d(px, py, pz);
  hand.orientation_unity = Eigen::Quaterniond(qw, qx, qy, qz);
  hand.grip = static_cast<double>(grip);
  hand.trigger = static_cast<double>(trigger);
  hand.valid = valid != 0U;
  return true;
}

bool parsePacket(
  const std::array<std::uint8_t, kPacketSize> & buffer,
  QuestPacket & packet)
{
  std::size_t offset = 0U;
  std::uint32_t magic = 0U;
  std::uint32_t version = 0U;
  std::uint32_t reserved = 0U;

  if (!readLittleEndian(buffer, offset, magic) ||
      !readLittleEndian(buffer, offset, version) ||
      !readLittleEndian(buffer, offset, packet.sequence) ||
      !readLittleEndian(buffer, offset, reserved) ||
      !readLittleEndian(buffer, offset, packet.device_time_s))
  {
    return false;
  }

  if (magic != kMagic || version != kVersion) {
    return false;
  }

  return readHand(buffer, offset, packet.left) &&
         readHand(buffer, offset, packet.right) &&
         offset == kPacketSize;
}

Eigen::Matrix3d unityToRosBasis()
{
  // Unity: +X right, +Y up, +Z forward (left-handed convention)
  // ROS base_link: +X forward, +Y left, +Z up (right-handed convention)
  Eigen::Matrix3d basis;
  basis <<
    0.0, 0.0, 1.0,
   -1.0, 0.0, 0.0,
    0.0, 1.0, 0.0;
  return basis;
}

bool handRotationInRos(
  const HandPacket & hand,
  const Eigen::Matrix3d & unity_to_ros,
  Eigen::Matrix3d & rotation_ros)
{
  Eigen::Quaterniond q = hand.orientation_unity;
  if (!std::isfinite(q.norm()) || q.norm() < 1e-6) {
    return false;
  }
  q.normalize();
  rotation_ros = unity_to_ros * q.toRotationMatrix() * unity_to_ros.transpose();
  return rotation_ros.allFinite();
}

Eigen::Vector3d clampNorm(const Eigen::Vector3d & value, const double max_norm)
{
  const double norm = value.norm();
  if (max_norm > 0.0 && norm > max_norm && norm > 1e-9) {
    return value * (max_norm / norm);
  }
  return value;
}
}  // namespace

class QuestDualArmTeleopNode : public rclcpp::Node
{
public:
  QuestDualArmTeleopNode()
  : Node("quest_dual_arm_teleop"), unity_to_ros_(unityToRosBasis())
  {
    const std::uint16_t endian_test = 1U;
    if (*reinterpret_cast<const std::uint8_t *>(&endian_test) != 1U) {
      throw std::runtime_error("This protocol implementation requires a little-endian CPU");
    }

    bind_address_ = declare_parameter<std::string>("bind_address", "0.0.0.0");
    tcp_port_ = declare_parameter<int>("tcp_port", 5005);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    quest_frame_ = declare_parameter<std::string>("quest_frame", "quest_tracking");
    left_twist_topic_ = declare_parameter<std::string>(
      "left_twist_topic", "/left_servo/delta_twist_cmds");
    right_twist_topic_ = declare_parameter<std::string>(
      "right_twist_topic", "/right_servo/delta_twist_cmds");
    left_gripper_topic_ = declare_parameter<std::string>(
      "left_gripper_topic", "/left_gripper_controller/joint_trajectory");
    right_gripper_topic_ = declare_parameter<std::string>(
      "right_gripper_topic", "/right_gripper_controller/joint_trajectory");
    left_gripper_joint_ = declare_parameter<std::string>(
      "left_gripper_joint", "left_gripper_left_finger_joint");
    right_gripper_joint_ = declare_parameter<std::string>(
      "right_gripper_joint", "gripper_left_finger_joint");
    invert_vertical_command_ = declare_parameter<bool>("invert_vertical_command", true);

    translation_scale_ = declare_parameter<double>("translation_scale", 0.70);
    rotation_scale_ = declare_parameter<double>("rotation_scale", 0.75);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.15);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.60);
    grip_threshold_ = declare_parameter<double>("grip_threshold", 0.65);
    trigger_press_threshold_ = declare_parameter<double>("trigger_press_threshold", 0.65);
    trigger_release_threshold_ = declare_parameter<double>("trigger_release_threshold", 0.35);
    gripper_open_position_ = declare_parameter<double>("gripper_open_position", 0.0);
    gripper_closed_position_ = declare_parameter<double>("gripper_closed_position", 0.06);
    gripper_motion_time_s_ = declare_parameter<double>("gripper_motion_time", 0.50);
    input_timeout_s_ = declare_parameter<double>("input_timeout", 0.15);
    filter_alpha_ = declare_parameter<double>("filter_alpha", 0.35);
    max_packet_translation_jump_ = declare_parameter<double>(
      "max_packet_translation_jump", 0.12);
    max_packet_rotation_jump_ = declare_parameter<double>(
      "max_packet_rotation_jump", 0.70);
    control_rate_hz_ = declare_parameter<double>("control_rate", 100.0);

    filter_alpha_ = std::clamp(filter_alpha_, 0.0, 1.0);
    if (tcp_port_ <= 0 || tcp_port_ > 65535) {
      throw std::runtime_error("tcp_port must be in [1, 65535]");
    }
    if (control_rate_hz_ <= 0.0) {
      throw std::runtime_error("control_rate must be positive");
    }
    if (trigger_release_threshold_ < 0.0 || trigger_press_threshold_ > 1.0 ||
      trigger_release_threshold_ >= trigger_press_threshold_)
    {
      throw std::runtime_error(
              "trigger thresholds must satisfy 0 <= release < press <= 1");
    }
    if (gripper_motion_time_s_ <= 0.0) {
      throw std::runtime_error("gripper_motion_time must be positive");
    }

    left_twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(left_twist_topic_, 10);
    right_twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(right_twist_topic_, 10);
    left_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/quest/left_controller/pose", 10);
    right_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/quest/right_controller/pose", 10);
    left_deadman_pub_ = create_publisher<std_msgs::msg::Bool>("/quest/left_controller/deadman", 10);
    right_deadman_pub_ = create_publisher<std_msgs::msg::Bool>("/quest/right_controller/deadman", 10);
    left_trigger_pub_ = create_publisher<std_msgs::msg::Float32>("/quest/left_controller/trigger", 10);
    right_trigger_pub_ = create_publisher<std_msgs::msg::Float32>("/quest/right_controller/trigger", 10);
    left_gripper_closed_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/quest/left_controller/gripper_closed", 10);
    right_gripper_closed_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/quest/right_controller/gripper_closed", 10);
    left_gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      left_gripper_topic_, 10);
    right_gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      right_gripper_topic_, 10);
    connected_pub_ = create_publisher<std_msgs::msg::Bool>("/quest/connected", 10);

    const auto timer_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&QuestDualArmTeleopNode::controlLoop, this));

    network_thread_ = std::thread(&QuestDualArmTeleopNode::networkLoop, this);

    RCLCPP_INFO(
      get_logger(),
      "Quest dual-arm teleop listening on %s:%d; hold each controller grip to enable its arm. "
      "Each trigger press toggles its gripper open/closed. "
      "Scales linear=%.2f angular=%.2f, limits linear=%.2f m/s angular=%.2f rad/s, "
      "vertical command inverted=%s",
      bind_address_.c_str(), tcp_port_, translation_scale_, rotation_scale_,
      max_linear_speed_, max_angular_speed_, invert_vertical_command_ ? "true" : "false");
  }

  ~QuestDualArmTeleopNode() override
  {
    running_.store(false);
    const int client = client_fd_.exchange(-1);
    if (client >= 0) {
      ::shutdown(client, SHUT_RDWR);
      ::close(client);
    }
    const int server = server_fd_.exchange(-1);
    if (server >= 0) {
      ::shutdown(server, SHUT_RDWR);
      ::close(server);
    }
    if (network_thread_.joinable()) {
      network_thread_.join();
    }
  }

private:
  struct HandState
  {
    bool initialized{false};
    bool deadman{false};
    double previous_device_time_s{0.0};
    Eigen::Vector3d previous_position_unity{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d previous_rotation_ros{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d filtered_linear{Eigen::Vector3d::Zero()};
    Eigen::Vector3d filtered_angular{Eigen::Vector3d::Zero()};
  };

  struct GripperState
  {
    bool initialized{false};
    bool trigger_pressed{false};
    bool closed{false};
  };

  bool receiveExact(const int fd, std::array<std::uint8_t, kPacketSize> & buffer)
  {
    std::size_t received = 0U;
    while (running_.load() && received < buffer.size()) {
      const ssize_t result = ::recv(
        fd,
        buffer.data() + received,
        buffer.size() - received,
        0);
      if (result > 0) {
        received += static_cast<std::size_t>(result);
        continue;
      }
      if (result == 0) {
        return false;
      }
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      return false;
    }
    return received == buffer.size();
  }

  void networkLoop()
  {
    int server = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server < 0) {
      RCLCPP_ERROR(get_logger(), "socket() failed: %s", std::strerror(errno));
      return;
    }
    server_fd_.store(server);

    int reuse = 1;
    ::setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(tcp_port_));
    if (::inet_pton(AF_INET, bind_address_.c_str(), &address.sin_addr) != 1) {
      RCLCPP_ERROR(get_logger(), "Invalid bind_address: %s", bind_address_.c_str());
      return;
    }

    if (::bind(server, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      RCLCPP_ERROR(get_logger(), "bind() failed: %s", std::strerror(errno));
      return;
    }
    if (::listen(server, 1) < 0) {
      RCLCPP_ERROR(get_logger(), "listen() failed: %s", std::strerror(errno));
      return;
    }

    while (running_.load()) {
      fd_set read_set;
      FD_ZERO(&read_set);
      FD_SET(server, &read_set);
      timeval timeout{};
      timeout.tv_sec = 0;
      timeout.tv_usec = 200000;
      const int ready = ::select(server + 1, &read_set, nullptr, nullptr, &timeout);
      if (!running_.load()) {
        break;
      }
      if (ready < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "select() failed: %s", std::strerror(errno));
        break;
      }
      if (ready == 0) {
        continue;
      }

      sockaddr_in client_address{};
      socklen_t client_length = sizeof(client_address);
      const int client = ::accept(
        server,
        reinterpret_cast<sockaddr *>(&client_address),
        &client_length);
      if (client < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_WARN(get_logger(), "accept() failed: %s", std::strerror(errno));
        continue;
      }

      client_fd_.store(client);
      int no_delay = 1;
      ::setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));
      timeval receive_timeout{};
      receive_timeout.tv_sec = 0;
      receive_timeout.tv_usec = 200000;
      ::setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout, sizeof(receive_timeout));
      connected_.store(true);

      char client_ip[INET_ADDRSTRLEN]{};
      ::inet_ntop(AF_INET, &client_address.sin_addr, client_ip, sizeof(client_ip));
      RCLCPP_INFO(get_logger(), "Quest client connected from %s", client_ip);

      std::array<std::uint8_t, kPacketSize> buffer{};
      while (running_.load() && receiveExact(client, buffer)) {
        QuestPacket packet;
        if (!parsePacket(buffer, packet)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Discarded invalid Quest packet; check Unity and ROS protocol versions");
          continue;
        }

        {
          std::lock_guard<std::mutex> lock(packet_mutex_);
          latest_packet_ = packet;
          latest_receive_time_ = std::chrono::steady_clock::now();
          ++receive_counter_;
        }
      }

      connected_.store(false);
      const int owned_client = client_fd_.exchange(-1);
      if (owned_client >= 0) {
        ::shutdown(owned_client, SHUT_RDWR);
        ::close(owned_client);
      }
      RCLCPP_WARN(get_logger(), "Quest client disconnected; both arms halted");
    }
  }

  void publishPose(
    const HandPacket & hand,
    const Eigen::Matrix3d & rotation_ros,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & publisher)
  {
    const Eigen::Vector3d position_ros = unity_to_ros_ * hand.position_unity;
    Eigen::Quaterniond orientation_ros(rotation_ros);
    orientation_ros.normalize();

    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = quest_frame_;
    message.pose.position.x = position_ros.x();
    message.pose.position.y = position_ros.y();
    message.pose.position.z = position_ros.z();
    message.pose.orientation.x = orientation_ros.x();
    message.pose.orientation.y = orientation_ros.y();
    message.pose.orientation.z = orientation_ros.z();
    message.pose.orientation.w = orientation_ros.w();
    publisher->publish(message);
  }

  void publishTwist(
    const Eigen::Vector3d & linear,
    const Eigen::Vector3d & angular,
    const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr & publisher)
  {
    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = now();
    message.header.frame_id = base_frame_;
    message.twist.linear.x = linear.x();
    message.twist.linear.y = linear.y();
    message.twist.linear.z = linear.z();
    message.twist.angular.x = angular.x();
    message.twist.angular.y = angular.y();
    message.twist.angular.z = angular.z();
    publisher->publish(message);
  }

  void haltHand(
    HandState & state,
    const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr & publisher)
  {
    // Only publish a stop command when this hand was actually active.
    // Previously, an idle/disconnected Quest published zero Twist forever at
    // control_rate_hz_, which could overwrite servo_smoke_test.py or any other
    // temporary Servo command publisher on the same topic.
    const bool was_active =
      state.initialized || state.deadman ||
      state.filtered_linear.squaredNorm() > 0.0 ||
      state.filtered_angular.squaredNorm() > 0.0;

    state.initialized = false;
    state.deadman = false;
    state.filtered_linear.setZero();
    state.filtered_angular.setZero();

    if (was_active) {
      publishTwist(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), publisher);
    }
  }

  void processHand(
    const HandPacket & hand,
    const double device_time_s,
    HandState & state,
    const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr & twist_publisher,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pose_publisher,
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & deadman_publisher)
  {
    Eigen::Matrix3d current_rotation_ros = Eigen::Matrix3d::Identity();
    const bool rotation_valid = hand.valid && handRotationInRos(hand, unity_to_ros_, current_rotation_ros);
    const bool deadman = hand.valid && rotation_valid && hand.grip >= grip_threshold_;

    std_msgs::msg::Bool deadman_message;
    deadman_message.data = deadman;
    deadman_publisher->publish(deadman_message);

    if (hand.valid && rotation_valid) {
      publishPose(hand, current_rotation_ros, pose_publisher);
    }

    if (!deadman) {
      haltHand(state, twist_publisher);
      return;
    }

    if (!state.initialized) {
      state.initialized = true;
      state.deadman = true;
      state.previous_device_time_s = device_time_s;
      state.previous_position_unity = hand.position_unity;
      state.previous_rotation_ros = current_rotation_ros;
      state.filtered_linear.setZero();
      state.filtered_angular.setZero();
      publishTwist(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), twist_publisher);
      return;
    }

    const double dt = device_time_s - state.previous_device_time_s;
    const Eigen::Vector3d delta_position_unity =
      hand.position_unity - state.previous_position_unity;
    const Eigen::Matrix3d delta_rotation_ros =
      current_rotation_ros * state.previous_rotation_ros.transpose();
    const Eigen::AngleAxisd angle_axis(delta_rotation_ros);
    const double delta_angle = std::abs(angle_axis.angle());

    state.previous_device_time_s = device_time_s;
    state.previous_position_unity = hand.position_unity;
    state.previous_rotation_ros = current_rotation_ros;

    if (!std::isfinite(dt) || dt < 0.002 || dt > 0.20 ||
        !delta_position_unity.allFinite() ||
        delta_position_unity.norm() > max_packet_translation_jump_ ||
        !std::isfinite(delta_angle) ||
        delta_angle > max_packet_rotation_jump_)
    {
      state.filtered_linear.setZero();
      state.filtered_angular.setZero();
      publishTwist(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), twist_publisher);
      return;
    }

    Eigen::Vector3d linear =
      translation_scale_ * (unity_to_ros_ * delta_position_unity) / dt;
    // The Quest pose topic keeps the standard OpenXR-to-ROS mapping for diagnostics.
    // Only the robot command is inverted to match this physical robot's installed
    // base-frame vertical direction.
    if (invert_vertical_command_) {
      linear.z() = -linear.z();
    }
    Eigen::Vector3d angular = Eigen::Vector3d::Zero();
    if (delta_angle > 1e-6 && angle_axis.axis().allFinite()) {
      angular = rotation_scale_ * angle_axis.axis() * (angle_axis.angle() / dt);
    }

    linear = clampNorm(linear, max_linear_speed_);
    angular = clampNorm(angular, max_angular_speed_);

    state.filtered_linear =
      filter_alpha_ * linear + (1.0 - filter_alpha_) * state.filtered_linear;
    state.filtered_angular =
      filter_alpha_ * angular + (1.0 - filter_alpha_) * state.filtered_angular;

    publishTwist(state.filtered_linear, state.filtered_angular, twist_publisher);
  }

  void haltBoth()
  {
    haltHand(left_state_, left_twist_pub_);
    haltHand(right_state_, right_twist_pub_);

    std_msgs::msg::Bool disabled;
    disabled.data = false;
    left_deadman_pub_->publish(disabled);
    right_deadman_pub_->publish(disabled);
  }

  void publishGripperCommand(
    const bool closed,
    const std::string & joint_name,
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr & publisher)
  {
    trajectory_msgs::msg::JointTrajectory message;
    message.joint_names.push_back(joint_name);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.push_back(closed ? gripper_closed_position_ : gripper_open_position_);
    const auto duration_ns = static_cast<std::int64_t>(
      std::llround(gripper_motion_time_s_ * 1.0e9));
    point.time_from_start.sec = static_cast<std::int32_t>(duration_ns / 1000000000LL);
    point.time_from_start.nanosec = static_cast<std::uint32_t>(duration_ns % 1000000000LL);
    message.points.push_back(point);
    publisher->publish(message);
  }

  void processGripper(
    const HandPacket & hand,
    GripperState & state,
    const std::string & side,
    const std::string & joint_name,
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr & command_publisher,
    const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & trigger_publisher,
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & closed_publisher)
  {
    std_msgs::msg::Float32 trigger_message;
    trigger_message.data = static_cast<float>(std::clamp(hand.trigger, 0.0, 1.0));
    trigger_publisher->publish(trigger_message);

    if (!hand.valid || !std::isfinite(hand.trigger)) {
      return;
    }

    const double trigger = std::clamp(hand.trigger, 0.0, 1.0);
    bool send_command = false;

    if (!state.initialized) {
      // Synchronize the edge detector without moving the gripper.  This avoids
      // an unexpected toggle if TCP reconnects while the trigger is held.
      state.trigger_pressed = trigger >= trigger_press_threshold_;
      state.initialized = true;
    } else if (!state.trigger_pressed && trigger >= trigger_press_threshold_) {
      state.trigger_pressed = true;
      state.closed = !state.closed;
      send_command = true;
    } else if (state.trigger_pressed && trigger <= trigger_release_threshold_) {
      state.trigger_pressed = false;
    }

    if (send_command) {
      publishGripperCommand(state.closed, joint_name, command_publisher);
      RCLCPP_INFO(
        get_logger(), "%s trigger pressed -> gripper %s (%.3f m)",
        side.c_str(), state.closed ? "CLOSE" : "OPEN",
        state.closed ? gripper_closed_position_ : gripper_open_position_);
    }

    std_msgs::msg::Bool closed_message;
    closed_message.data = state.closed;
    closed_publisher->publish(closed_message);
  }

  void controlLoop()
  {
    std_msgs::msg::Bool connected_message;
    connected_message.data = connected_.load();
    connected_pub_->publish(connected_message);

    QuestPacket packet;
    std::uint64_t counter = 0U;
    std::chrono::steady_clock::time_point receive_time;
    {
      std::lock_guard<std::mutex> lock(packet_mutex_);
      packet = latest_packet_;
      counter = receive_counter_;
      receive_time = latest_receive_time_;
    }

    const auto now_steady = std::chrono::steady_clock::now();
    const bool have_packet = counter > 0U;
    const double age_s = have_packet ?
      std::chrono::duration<double>(now_steady - receive_time).count() :
      std::numeric_limits<double>::infinity();

    if (!connected_.load() || !have_packet || age_s > input_timeout_s_) {
      haltBoth();
      // Do not open a gripper automatically on a link loss: a held object must
      // not be dropped. Re-synchronize the trigger edge after reconnect.
      left_gripper_state_.initialized = false;
      right_gripper_state_.initialized = false;
      last_processed_counter_ = counter;
      return;
    }

    if (counter == last_processed_counter_) {
      return;
    }
    last_processed_counter_ = counter;

    processHand(
      packet.left,
      packet.device_time_s,
      left_state_,
      left_twist_pub_,
      left_pose_pub_,
      left_deadman_pub_);
    processHand(
      packet.right,
      packet.device_time_s,
      right_state_,
      right_twist_pub_,
      right_pose_pub_,
      right_deadman_pub_);
    processGripper(
      packet.left,
      left_gripper_state_,
      "Left",
      left_gripper_joint_,
      left_gripper_pub_,
      left_trigger_pub_,
      left_gripper_closed_pub_);
    processGripper(
      packet.right,
      right_gripper_state_,
      "Right",
      right_gripper_joint_,
      right_gripper_pub_,
      right_trigger_pub_,
      right_gripper_closed_pub_);
  }

  std::string bind_address_;
  int tcp_port_{5005};
  std::string base_frame_;
  std::string quest_frame_;
  std::string left_twist_topic_;
  std::string right_twist_topic_;
  std::string left_gripper_topic_;
  std::string right_gripper_topic_;
  std::string left_gripper_joint_;
  std::string right_gripper_joint_;
  bool invert_vertical_command_{true};

  double translation_scale_{0.70};
  double rotation_scale_{0.75};
  double max_linear_speed_{0.15};
  double max_angular_speed_{0.60};
  double grip_threshold_{0.65};
  double trigger_press_threshold_{0.65};
  double trigger_release_threshold_{0.35};
  double gripper_open_position_{0.0};
  double gripper_closed_position_{0.06};
  double gripper_motion_time_s_{0.50};
  double input_timeout_s_{0.15};
  double filter_alpha_{0.35};
  double max_packet_translation_jump_{0.12};
  double max_packet_rotation_jump_{0.70};
  double control_rate_hz_{100.0};

  const Eigen::Matrix3d unity_to_ros_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr left_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr right_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_deadman_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_deadman_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_gripper_closed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_gripper_closed_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_gripper_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  HandState left_state_;
  HandState right_state_;
  GripperState left_gripper_state_;
  GripperState right_gripper_state_;

  std::atomic<bool> running_{true};
  std::atomic<bool> connected_{false};
  std::atomic<int> server_fd_{-1};
  std::atomic<int> client_fd_{-1};
  std::thread network_thread_;

  std::mutex packet_mutex_;
  QuestPacket latest_packet_;
  std::chrono::steady_clock::time_point latest_receive_time_{};
  std::uint64_t receive_counter_{0U};
  std::uint64_t last_processed_counter_{0U};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<QuestDualArmTeleopNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "quest_dual_arm_teleop failed: %s\n", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
