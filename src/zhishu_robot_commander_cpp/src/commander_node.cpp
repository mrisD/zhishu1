#include "zhishu_robot_commander_cpp/commander.hpp"

#include <zhishu_robot_interfaces/msg/commander_status.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>

namespace zhishu_robot_commander_cpp
{

template<typename ParameterT>
ParameterT getOrDeclareParameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const ParameterT & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<ParameterT>();
  }
  return node->declare_parameter<ParameterT>(name, default_value);
}

class CommanderBridge
{
public:
  CommanderBridge(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<Commander> & commander)
  : node_(node), commander_(commander)
  {
    // Keep the original String status topic source-compatible for existing
    // guohua_ws consumers. New SDK clients use the structured companion topic.
    status_pub_ = node_->create_publisher<std_msgs::msg::String>("~/status", 10);
    structured_status_pub_ =
      node_->create_publisher<zhishu_robot_interfaces::msg::CommanderStatus>(
      "~/status_structured", 10);

    left_joint_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/left_arm/joint_command", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        publishResult("left_arm/joint_command", commander_->moveLeftArmJointsResult(msg->data));
      });
    right_joint_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/right_arm/joint_command", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        publishResult("right_arm/joint_command", commander_->moveRightArmJointsResult(msg->data));
      });
    both_joint_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/both_arms/joint_command", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        publishResult("both_arms/joint_command", commander_->moveBothArmsJointsResult(msg->data));
      });

    // Legacy arm-tip pose commands: targets refer to L_hand_link / R_hand_link by default.
    left_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/left_arm/pose_command", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        publishResult("left_arm/pose_command", commander_->moveLeftArmPoseResult(*msg, false));
      });
    right_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/right_arm/pose_command", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        publishResult("right_arm/pose_command", commander_->moveRightArmPoseResult(*msg, false));
      });
    both_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
      "~/both_arms/pose_command", 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (msg->poses.size() != 2) {
          publishResult(
            "both_arms/pose_command",
            Result::failure(
              ErrorCode::kInvalidArgument,
              "both_arms/pose_command requires [left_pose, right_pose]"));
          return;
        }
        const auto targets = poseArrayTargets(*msg);
        publishResult(
          "both_arms/pose_command",
          commander_->moveBothArmsPosesResult(targets.first, targets.second));
      });
    left_cartesian_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/left_arm/cartesian_pose_command", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        publishResult(
          "left_arm/cartesian_pose_command", commander_->moveLeftArmPoseResult(*msg, true));
      });
    right_cartesian_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/right_arm/cartesian_pose_command", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        publishResult(
          "right_arm/cartesian_pose_command", commander_->moveRightArmPoseResult(*msg, true));
      });

    // True gripper-TCP commands. Commander converts TCP targets to the arm IK tips using
    // the fixed transforms from MoveIt's RobotModel, so no 145 mm offset is hard-coded here.
    left_tcp_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/left_arm/tcp_pose_command", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        publishResult("left_arm/tcp_pose_command", commander_->moveLeftTcpPoseResult(*msg, false));
      });
    right_tcp_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/right_arm/tcp_pose_command", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        publishResult("right_arm/tcp_pose_command", commander_->moveRightTcpPoseResult(*msg, false));
      });
    both_tcp_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
      "~/both_arms/tcp_pose_command", 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (msg->poses.size() != 2) {
          publishResult(
            "both_arms/tcp_pose_command",
            Result::failure(
              ErrorCode::kInvalidArgument,
              "both_arms/tcp_pose_command requires [left_tcp_pose, right_tcp_pose]"));
          return;
        }
        const auto targets = poseArrayTargets(*msg);
        publishResult(
          "both_arms/tcp_pose_command",
          commander_->moveBothTcpPosesResult(targets.first, targets.second));
      });
    left_cartesian_tcp_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/left_arm/cartesian_tcp_pose_command", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        publishResult(
          "left_arm/cartesian_tcp_pose_command", commander_->moveLeftTcpPoseResult(*msg, true));
      });
    right_cartesian_tcp_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/right_arm/cartesian_tcp_pose_command", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        publishResult(
          "right_arm/cartesian_tcp_pose_command", commander_->moveRightTcpPoseResult(*msg, true));
      });

    left_named_sub_ = createNamedSubscription(
      "~/left_arm/named_target",
      "left_arm/named_target",
      [this](const std::string & name) {return commander_->moveLeftArmNamedResult(name);});
    right_named_sub_ = createNamedSubscription(
      "~/right_arm/named_target",
      "right_arm/named_target",
      [this](const std::string & name) {return commander_->moveRightArmNamedResult(name);});
    both_named_sub_ = createNamedSubscription(
      "~/both_arms/named_target",
      "both_arms/named_target",
      [this](const std::string & name) {return commander_->moveBothArmsNamedResult(name);});

    left_gripper_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "~/left_gripper/command", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        publishResult("left_gripper/command", commander_->moveLeftGripperResult(msg->data));
      });
    right_gripper_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "~/right_gripper/command", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        publishResult("right_gripper/command", commander_->moveRightGripperResult(msg->data));
      });
    both_grippers_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/both_grippers/command", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 2) {
          publishResult(
            "both_grippers/command",
            Result::failure(
              ErrorCode::kInvalidArgument,
              "both_grippers/command requires [left, right]"));
          return;
        }
        publishResult(
          "both_grippers/command",
          commander_->moveBothGrippersResult(msg->data[0], msg->data[1]));
      });

    gripper_named_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "~/grippers/named_command", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        Result result;
        if (msg->data == "open_left") {
          result = commander_->openLeftGripperResult();
        } else if (msg->data == "close_left") {
          result = commander_->closeLeftGripperResult();
        } else if (msg->data == "open_right") {
          result = commander_->openRightGripperResult();
        } else if (msg->data == "close_right") {
          result = commander_->closeRightGripperResult();
        } else if (msg->data == "open_both") {
          result = commander_->openBothGrippersResult();
        } else if (msg->data == "close_both") {
          result = commander_->closeBothGrippersResult();
        } else {
          result = Result::failure(
            ErrorCode::kInvalidArgument,
            "unknown gripper command; expected open_left, close_left, open_right, "
            "close_right, open_both, or close_both");
        }
        publishResult("grippers/named_command:" + msg->data, result);
      });

    stop_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions stop_options;
    stop_options.callback_group = stop_callback_group_;
    stop_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "~/stop", 10,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        commander_->stop();
        publishResult("stop", Result::success("MoveIt stop requested"));
      },
      stop_options);

    RCLCPP_INFO(node_->get_logger(), "Commander ROS topic bridge is ready");
  }

private:
  using NamedCallback = std::function<Result(const std::string &)>;

  static std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped>
  poseArrayTargets(const geometry_msgs::msg::PoseArray & msg)
  {
    geometry_msgs::msg::PoseStamped left_target;
    geometry_msgs::msg::PoseStamped right_target;
    left_target.header = msg.header;
    right_target.header = msg.header;
    left_target.pose = msg.poses[0];
    right_target.pose = msg.poses[1];
    return {left_target, right_target};
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr createNamedSubscription(
    const std::string & topic,
    const std::string & command,
    NamedCallback callback)
  {
    return node_->create_subscription<std_msgs::msg::String>(
      topic, 10,
      [this, command, callback = std::move(callback)](
        const std_msgs::msg::String::SharedPtr msg) {
        publishResult(command, callback(msg->data));
      });
  }

  void publishResult(const std::string & command, const Result & result)
  {
    zhishu_robot_interfaces::msg::CommanderStatus status;
    status.stamp = node_->now();
    status.command = command;
    status.success = result.ok();
    status.error_code = static_cast<std::int32_t>(result.code);
    status.error_name = toString(result.code);
    status.native_code = result.native_code;
    status.message = result.message;
    structured_status_pub_->publish(status);

    std_msgs::msg::String text;
    text.data = result.ok() ? "SUCCESS: " + command : "FAILED: " + command;
    if (!result.message.empty()) {
      text.data += " [" + std::string(toString(result.code)) + "] " + result.message;
    }
    status_pub_->publish(text);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Commander> commander_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<zhishu_robot_interfaces::msg::CommanderStatus>::SharedPtr
    structured_status_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr both_joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr both_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_cartesian_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_cartesian_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_tcp_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_tcp_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr both_tcp_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_cartesian_tcp_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_cartesian_tcp_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_named_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_named_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr both_named_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_gripper_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_gripper_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr both_grippers_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_named_sub_;
  rclcpp::CallbackGroup::SharedPtr stop_callback_group_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
};

}  // namespace zhishu_robot_commander_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "zhishu_robot_commander",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  try {
    zhishu_robot_commander_cpp::Commander::Options options;
    options.left_end_effector_link =
      zhishu_robot_commander_cpp::getOrDeclareParameter<std::string>(
      node, "commander.left_end_effector_link", "L_hand_link");
    options.right_end_effector_link =
      zhishu_robot_commander_cpp::getOrDeclareParameter<std::string>(
      node, "commander.right_end_effector_link", "R_hand_link");
    options.left_tcp_link = zhishu_robot_commander_cpp::getOrDeclareParameter<std::string>(
      node, "commander.left_tcp_link", "left_gripper_tcp");
    options.right_tcp_link = zhishu_robot_commander_cpp::getOrDeclareParameter<std::string>(
      node, "commander.right_tcp_link", "right_gripper_tcp");
    options.velocity_scaling = zhishu_robot_commander_cpp::getOrDeclareParameter<double>(
      node, "commander.velocity_scaling", 0.30);
    options.acceleration_scaling = zhishu_robot_commander_cpp::getOrDeclareParameter<double>(
      node, "commander.acceleration_scaling", 0.30);
    options.planning_time = zhishu_robot_commander_cpp::getOrDeclareParameter<double>(
      node, "commander.planning_time", 5.0);
    options.planning_attempts = static_cast<int>(
      zhishu_robot_commander_cpp::getOrDeclareParameter<int64_t>(
        node, "commander.planning_attempts", 5));
    options.cartesian_eef_step = zhishu_robot_commander_cpp::getOrDeclareParameter<double>(
      node, "commander.cartesian_eef_step", 0.005);
    options.minimum_cartesian_fraction =
      zhishu_robot_commander_cpp::getOrDeclareParameter<double>(
      node, "commander.minimum_cartesian_fraction", 0.95);

    auto commander = std::make_shared<zhishu_robot_commander_cpp::Commander>(node, options);
    auto bridge = std::make_shared<zhishu_robot_commander_cpp::CommanderBridge>(node, commander);
    (void)bridge;
    spinner.join();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(node->get_logger(), "Commander startup failed: %s", exception.what());
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
