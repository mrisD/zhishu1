#include "zhishu_robot_commander_cpp/commander.hpp"

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
    status_pub_ = node_->create_publisher<std_msgs::msg::String>("~/status", 10);

    left_joint_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/left_arm/joint_command", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        publishResult("left_arm/joint_command", commander_->moveLeftArmJoints(msg->data));
      });
    right_joint_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/right_arm/joint_command", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        publishResult("right_arm/joint_command", commander_->moveRightArmJoints(msg->data));
      });
    both_joint_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/both_arms/joint_command", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        publishResult("both_arms/joint_command", commander_->moveBothArmsJoints(msg->data));
      });

    left_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/left_arm/pose_command", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        publishResult("left_arm/pose_command", commander_->moveLeftArmPose(*msg, false));
      });
    right_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/right_arm/pose_command", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        publishResult("right_arm/pose_command", commander_->moveRightArmPose(*msg, false));
      });
    both_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
      "~/both_arms/pose_command", 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (msg->poses.size() != 2) {
          RCLCPP_ERROR(node_->get_logger(), "both_arms/pose_command requires [left_pose, right_pose]");
          publishResult("both_arms/pose_command", false);
          return;
        }
        geometry_msgs::msg::PoseStamped left_target;
        geometry_msgs::msg::PoseStamped right_target;
        left_target.header = msg->header;
        right_target.header = msg->header;
        left_target.pose = msg->poses[0];
        right_target.pose = msg->poses[1];
        publishResult(
          "both_arms/pose_command",
          commander_->moveBothArmsPoses(left_target, right_target));
      });
    left_cartesian_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/left_arm/cartesian_pose_command", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        publishResult("left_arm/cartesian_pose_command", commander_->moveLeftArmPose(*msg, true));
      });
    right_cartesian_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/right_arm/cartesian_pose_command", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        publishResult("right_arm/cartesian_pose_command", commander_->moveRightArmPose(*msg, true));
      });

    left_named_sub_ = createNamedSubscription(
      "~/left_arm/named_target",
      [this](const std::string & name) {return commander_->moveLeftArmNamed(name);});
    right_named_sub_ = createNamedSubscription(
      "~/right_arm/named_target",
      [this](const std::string & name) {return commander_->moveRightArmNamed(name);});
    both_named_sub_ = createNamedSubscription(
      "~/both_arms/named_target",
      [this](const std::string & name) {return commander_->moveBothArmsNamed(name);});

    left_gripper_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "~/left_gripper/command", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        publishResult("left_gripper/command", commander_->moveLeftGripper(msg->data));
      });
    right_gripper_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "~/right_gripper/command", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        publishResult("right_gripper/command", commander_->moveRightGripper(msg->data));
      });
    both_grippers_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/both_grippers/command", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 2) {
          RCLCPP_ERROR(node_->get_logger(), "both_grippers/command requires [left, right]");
          publishResult("both_grippers/command", false);
          return;
        }
        publishResult(
          "both_grippers/command",
          commander_->moveBothGrippers(msg->data[0], msg->data[1]));
      });

    gripper_named_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "~/grippers/named_command", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        bool result = false;
        if (msg->data == "open_left") {
          result = commander_->openLeftGripper();
        } else if (msg->data == "close_left") {
          result = commander_->closeLeftGripper();
        } else if (msg->data == "open_right") {
          result = commander_->openRightGripper();
        } else if (msg->data == "close_right") {
          result = commander_->closeRightGripper();
        } else if (msg->data == "open_both") {
          result = commander_->openBothGrippers();
        } else if (msg->data == "close_both") {
          result = commander_->closeBothGrippers();
        } else {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Unknown gripper command '%s'. Expected open_left, close_left, open_right, "
            "close_right, open_both, or close_both",
            msg->data.c_str());
        }
        publishResult("grippers/named_command:" + msg->data, result);
      });

    stop_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions stop_options;
    stop_options.callback_group = stop_callback_group_;
    stop_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "~/stop", 10,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        commander_->stop();
        publishResult("stop", true);
      },
      stop_options);

    RCLCPP_INFO(node_->get_logger(), "Commander ROS topic bridge is ready");
  }

private:
  using NamedCallback = std::function<bool(const std::string &)>;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr createNamedSubscription(
    const std::string & topic,
    NamedCallback callback)
  {
    return node_->create_subscription<std_msgs::msg::String>(
      topic, 10,
      [this, topic, callback = std::move(callback)](const std_msgs::msg::String::SharedPtr msg) {
        publishResult(topic, callback(msg->data));
      });
  }

  void publishResult(const std::string & command, bool success)
  {
    std_msgs::msg::String status;
    status.data = success ? "SUCCESS: " + command : "FAILED: " + command;
    status_pub_->publish(status);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Commander> commander_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr both_joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr both_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_cartesian_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_cartesian_sub_;
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
