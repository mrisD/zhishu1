#include "zhishu_robot_hardware/ethercat_driver.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>

namespace
{
uint32_t parseU32Param(rclcpp::Node & node, const std::string & name, int64_t default_value)
{
  const int64_t value = node.declare_parameter<int64_t>(name, default_value);
  if (value < 0 || value > static_cast<int64_t>(UINT32_MAX)) {
    throw std::runtime_error("Parameter '" + name + "' is out of uint32 range");
  }
  return static_cast<uint32_t>(value);
}

uint16_t parseU16Param(rclcpp::Node & node, const std::string & name, int64_t default_value)
{
  const int64_t value = node.declare_parameter<int64_t>(name, default_value);
  if (value < 0 || value > static_cast<int64_t>(UINT16_MAX)) {
    throw std::runtime_error("Parameter '" + name + "' is out of uint16 range");
  }
  return static_cast<uint16_t>(value);
}

std::string normalizePrefix(std::string prefix)
{
  while (!prefix.empty() && prefix.front() == '/') {
    prefix.erase(prefix.begin());
  }
  while (!prefix.empty() && prefix.back() == '/') {
    prefix.pop_back();
  }
  if (prefix.empty()) {
    return "motor";
  }
  return prefix;
}

std::string topic(const std::string & prefix, const std::string & name)
{
  return "/" + prefix + "/" + name;
}

std::string cia402StateToString(int state)
{
  switch (state) {
    case 0: return "not_ready_to_switch_on";
    case 1: return "switch_on_disabled";
    case 2: return "ready_to_switch_on";
    case 3: return "switched_on";
    case 4: return "operation_enabled";
    case 5: return "quick_stop_active";
    case 6: return "fault_reaction_active";
    case 7: return "fault";
    case 8: return "unknown";
    default: return "unknown";
  }
}
}  // namespace

class EthercatDriverNode final : public rclcpp::Node
{
public:
  EthercatDriverNode()
  : Node("ethercat_driver_node")
  {
    motor_name_ = declare_parameter<std::string>("motor_name", "drive3");
    topic_prefix_ = normalizePrefix(declare_parameter<std::string>("topic_prefix", motor_name_));
    joint_name_ = declare_parameter<std::string>("joint_name", "joint3");

    address_.master_index = static_cast<unsigned int>(declare_parameter<int>("master_index", 0));
    address_.alias = parseU16Param(*this, "alias", 0);
    address_.slave_position = parseU16Param(*this, "slave_position", 2);
    address_.vendor_id = parseU32Param(*this, "vendor_id", 0x00206688);
    address_.product_code = parseU32Param(*this, "product_code", 0x26483052);

    ethercat_driver::DriverConfig driver_config{};
    driver_config.cycle_ns = static_cast<int64_t>(declare_parameter<int>("cycle_ns", 1000000));
    driver_config.enable_dc = declare_parameter<bool>("enable_dc", true);
    driver_config.assign_activate = parseU16Param(*this, "assign_activate", 0x0300);
    driver_config.sync0_shift_ns = static_cast<int64_t>(declare_parameter<int>("sync0_shift_ns", 622620));
    driver_config.select_dc_reference_clock = declare_parameter<bool>("select_dc_reference_clock", true);
    driver_config.dc_reference_alias = parseU16Param(*this, "dc_reference_alias", 0);
    driver_config.dc_reference_slave_position = parseU16Param(*this, "dc_reference_slave_position", 0);
    driver_config.enable_realtime = declare_parameter<bool>("enable_realtime", true);
    driver_config.realtime_priority = declare_parameter<int>("realtime_priority", 80);

    ethercat_driver::MotorConfig motor_config{};
    motor_config.name = motor_name_;
    motor_config.joint_name = joint_name_;
    motor_config.address = address_;
    motor_config.configure_sdo = declare_parameter<bool>("configure_sdo", true);
    motor_config.configure_pdos = declare_parameter<bool>("configure_pdos", true);
    motor_config.mode_of_operation = static_cast<int8_t>(declare_parameter<int>("mode_of_operation", 8));
    motor_config.auto_enable = declare_parameter<bool>("auto_enable", true);
    motor_config.auto_fault_reset = declare_parameter<bool>("auto_fault_reset", true);
    motor_config.max_velocity_counts_s = declare_parameter<double>("max_velocity_counts_s", 8000.0);
    motor_config.max_accel_counts_s2 = declare_parameter<double>("max_accel_counts_s2", 20000.0);
    motor_config.target_tolerance_counts = static_cast<int64_t>(declare_parameter<int>("target_tolerance_counts", 20));
    motor_config.counts_per_rev = declare_parameter<double>("counts_per_rev", 131072.0);
    counts_per_rev_ = motor_config.counts_per_rev;

    status_publish_hz_ = declare_parameter<double>("status_publish_hz", 20.0);
    publish_joint_states_ = declare_parameter<bool>("publish_joint_states", true);

    if (driver_config.cycle_ns <= 0) {
      throw std::runtime_error("cycle_ns must be positive");
    }
    if (motor_config.counts_per_rev <= 0.0) {
      throw std::runtime_error("counts_per_rev must be positive");
    }

    driver_config.motors.push_back(motor_config);

    actual_counts_pub_ = create_publisher<std_msgs::msg::Int32>(topic(topic_prefix_, "actual_counts"), 10);
    target_feedback_pub_ = create_publisher<std_msgs::msg::Int32>(topic(topic_prefix_, "target_counts_feedback"), 10);
    statusword_pub_ = create_publisher<std_msgs::msg::UInt16>(topic(topic_prefix_, "statusword"), 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(topic(topic_prefix_, "state"), 10);
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    target_counts_sub_ = create_subscription<std_msgs::msg::Int32>(
      topic(topic_prefix_, "target_counts"), 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        if (!driver_.setAbsoluteTargetCounts(address_, msg->data)) {
          RCLCPP_ERROR(get_logger(), "setAbsoluteTargetCounts failed for %s", ethercat_driver::toString(address_).c_str());
        }
      });

    relative_counts_sub_ = create_subscription<std_msgs::msg::Int32>(
      topic(topic_prefix_, "relative_counts"), 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        if (!driver_.setRelativeTargetCounts(address_, msg->data)) {
          RCLCPP_ERROR(get_logger(), "setRelativeTargetCounts failed for %s", ethercat_driver::toString(address_).c_str());
        }
      });

    target_deg_sub_ = create_subscription<std_msgs::msg::Float64>(
      topic(topic_prefix_, "target_deg"), 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        if (!driver_.setTargetDegrees(address_, msg->data)) {
          RCLCPP_ERROR(get_logger(), "setTargetDegrees failed for %s", ethercat_driver::toString(address_).c_str());
        }
      });

    relative_deg_sub_ = create_subscription<std_msgs::msg::Float64>(
      topic(topic_prefix_, "relative_deg"), 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        if (!driver_.setRelativeDegrees(address_, msg->data)) {
          RCLCPP_ERROR(get_logger(), "setRelativeDegrees failed for %s", ethercat_driver::toString(address_).c_str());
        }
      });

    target_rad_sub_ = create_subscription<std_msgs::msg::Float64>(
      topic(topic_prefix_, "target_rad"), 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        if (!driver_.setTargetRadians(address_, msg->data)) {
          RCLCPP_ERROR(get_logger(), "setTargetRadians failed for %s", ethercat_driver::toString(address_).c_str());
        }
      });

    relative_rad_sub_ = create_subscription<std_msgs::msg::Float64>(
      topic(topic_prefix_, "relative_rad"), 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        if (!driver_.setRelativeRadians(address_, msg->data)) {
          RCLCPP_ERROR(get_logger(), "setRelativeRadians failed for %s", ethercat_driver::toString(address_).c_str());
        }
      });

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      topic(topic_prefix_, "enable"), 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        driver_.setEnable(address_, msg->data);
        RCLCPP_WARN(get_logger(), "%s command_enable set to %s", motor_name_.c_str(), msg->data ? "true" : "false");
      });

    fault_reset_sub_ = create_subscription<std_msgs::msg::Bool>(
      topic(topic_prefix_, "fault_reset"), 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          driver_.requestFaultReset(address_);
          RCLCPP_WARN(get_logger(), "%s fault reset requested", motor_name_.c_str());
        }
      });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, status_publish_hz_));
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishStatus(); });

    if (!driver_.start(driver_config)) {
      throw std::runtime_error("EthercatDriver start failed");
    }

    RCLCPP_INFO(
      get_logger(),
      "ethercat_driver_node started for %s: %s topic_prefix=/%s joint=%s",
      motor_name_.c_str(), ethercat_driver::toString(address_).c_str(), topic_prefix_.c_str(), joint_name_.c_str());
  }

  ~EthercatDriverNode() override
  {
    driver_.stop();
  }

private:
  void publishStatus()
  {
    ethercat_driver::MotorState state{};
    if (!driver_.getMotorState(address_, state)) {
      return;
    }

    auto actual_msg = std_msgs::msg::Int32();
    actual_msg.data = state.actual_counts;
    actual_counts_pub_->publish(actual_msg);

    auto target_msg = std_msgs::msg::Int32();
    target_msg.data = state.command_counts;
    target_feedback_pub_->publish(target_msg);

    auto sw_msg = std_msgs::msg::UInt16();
    sw_msg.data = state.status_word;
    statusword_pub_->publish(sw_msg);

    auto state_msg = std_msgs::msg::String();
    std::ostringstream oss;
    oss << cia402StateToString(state.cia402_state)
        << " ethercat_ok=" << (state.ethercat_ok ? "true" : "false")
        << " enabled=" << (state.operation_enabled ? "true" : "false")
        << " target_reached=" << (state.target_reached ? "true" : "false")
        << " domain_wc=" << state.domain_wc
        << " wc_state=" << state.domain_wc_state
        << " slave_online=" << (state.slave_online ? "true" : "false")
        << " slave_op=" << (state.slave_operational ? "true" : "false")
        << " slave_al=0x" << std::hex << state.slave_al_state << std::dec
        << " actual_counts=" << state.actual_counts
        << " command_counts=" << state.command_counts
        << " goal_counts=" << state.goal_counts
        << " address=[" << ethercat_driver::toString(address_) << "]";
    state_msg.data = oss.str();
    state_pub_->publish(state_msg);

    if (publish_joint_states_) {
      auto js = sensor_msgs::msg::JointState();
      js.header.stamp = now();
      js.name.push_back(joint_name_);
      const double actual_rad = static_cast<double>(state.actual_counts) / counts_per_rev_ * 2.0 * M_PI;
      js.position.push_back(actual_rad);
      js.velocity.push_back(0.0);
      js.effort.push_back(0.0);
      joint_state_pub_->publish(js);
    }
  }

  ethercat_driver::EthercatDriver driver_;
  ethercat_driver::MotorAddress address_{};

  std::string motor_name_ = "drive3";
  std::string topic_prefix_ = "drive3";
  std::string joint_name_ = "joint3";
  double counts_per_rev_ = 131072.0;
  double status_publish_hz_ = 20.0;
  bool publish_joint_states_ = true;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr actual_counts_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr statusword_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_counts_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr relative_counts_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_deg_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr relative_deg_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_rad_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr relative_rad_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fault_reset_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<EthercatDriverNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "ethercat_driver_node fatal error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
