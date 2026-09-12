#include "zhishu_robot_hardware/es_motor_hardware_interface.hpp"
#include "zhishu_robot_hardware/motor_fault_decoder.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace es_motor_hardware
{
namespace
{
constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr uint32_t kDefaultVendorId = 0x00206688;
constexpr uint32_t kDefaultProductCode = 0x26483052;
constexpr char kVelocityRawInterface[] = "velocity_raw";
constexpr char kTorqueRawInterface[] = "torque_raw";

const rclcpp::Logger & logger()
{
  static const rclcpp::Logger instance = rclcpp::get_logger("EsMotorHardwareInterface");
  return instance;
}

template<typename MapT>
bool has_param(const MapT & params, const std::string & key)
{
  return params.find(key) != params.end();
}

template<typename MapT>
std::string get_string_param(const MapT & params, const std::string & key, const std::string & default_value)
{
  const auto it = params.find(key);
  return it == params.end() ? default_value : it->second;
}

template<typename LocalMapT, typename GlobalMapT>
std::string get_local_or_global_param(
  const LocalMapT & local_params,
  const GlobalMapT & global_params,
  const std::string & key,
  const std::string & default_value)
{
  const auto local_it = local_params.find(key);
  if (local_it != local_params.end()) {
    return local_it->second;
  }
  return get_string_param(global_params, key, default_value);
}

int64_t parse_i64(const std::string & text, const std::string & field_name)
{
  try {
    size_t pos = 0;
    const long long value = std::stoll(text, &pos, 0);
    if (pos != text.size()) {
      throw std::invalid_argument("trailing characters");
    }
    return static_cast<int64_t>(value);
  } catch (const std::exception & e) {
    std::ostringstream oss;
    oss << "Invalid integer parameter '" << field_name << "'='" << text << "': " << e.what();
    throw std::runtime_error(oss.str());
  }
}

uint64_t parse_u64(const std::string & text, const std::string & field_name)
{
  try {
    size_t pos = 0;
    const unsigned long long value = std::stoull(text, &pos, 0);
    if (pos != text.size()) {
      throw std::invalid_argument("trailing characters");
    }
    return static_cast<uint64_t>(value);
  } catch (const std::exception & e) {
    std::ostringstream oss;
    oss << "Invalid unsigned integer parameter '" << field_name << "'='" << text << "': " << e.what();
    throw std::runtime_error(oss.str());
  }
}

double parse_double(const std::string & text, const std::string & field_name)
{
  try {
    size_t pos = 0;
    const double value = std::stod(text, &pos);
    if (pos != text.size()) {
      throw std::invalid_argument("trailing characters");
    }
    return value;
  } catch (const std::exception & e) {
    std::ostringstream oss;
    oss << "Invalid floating parameter '" << field_name << "'='" << text << "': " << e.what();
    throw std::runtime_error(oss.str());
  }
}

bool parse_bool(const std::string & text, const std::string & field_name)
{
  std::string value = text;
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "1" || value == "true" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off") {
    return false;
  }
  std::ostringstream oss;
  oss << "Invalid bool parameter '" << field_name << "'='" << text << "'";
  throw std::runtime_error(oss.str());
}

bool right_arm_default_slave_position(const std::string & joint_name, uint16_t & slave_position)
{
  // 右臂 7 轴默认映射：从机械臂根部到腕部，EtherCAT 从站位置 0 -> 6。
  static const std::vector<std::pair<std::string, uint16_t>> mapping = {
    {"right_shoulder_pitch_joint", 0},
    {"right_shoulder_roll_joint", 1},
    {"right_shoulder_yaw_joint", 2},
    {"right_elbow_pitch_joint", 3},
    {"right_wrist_yaw_joint", 4},
    {"right_wrist_pitch_joint", 5},
    {"right_wrist_roll_joint", 6},
  };

  for (const auto & item : mapping) {
    if (item.first == joint_name) {
      slave_position = item.second;
      return true;
    }
  }
  return false;
}

bool has_position_state_interface(const hardware_interface::ComponentInfo & joint)
{
  for (const auto & interface : joint.state_interfaces) {
    if (interface.name == hardware_interface::HW_IF_POSITION) {
      return true;
    }
  }
  return false;
}

bool has_position_command_interface(const hardware_interface::ComponentInfo & joint)
{
  for (const auto & interface : joint.command_interfaces) {
    if (interface.name == hardware_interface::HW_IF_POSITION) {
      return true;
    }
  }
  return false;
}

int32_t clamp_i64_to_i32(int64_t value)
{
  if (value > static_cast<int64_t>(std::numeric_limits<int32_t>::max())) {
    return std::numeric_limits<int32_t>::max();
  }
  if (value < static_cast<int64_t>(std::numeric_limits<int32_t>::min())) {
    return std::numeric_limits<int32_t>::min();
  }
  return static_cast<int32_t>(value);
}

}  // namespace

hardware_interface::CallbackReturn EsMotorHardwareInterface::
    on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  driver_ = std::make_shared<ethercat_driver::EthercatDriver>();

  position_cache_.assign(info_.joints.size(), 0.0);
  velocity_cache_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  effort_cache_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  velocity_raw_cache_.assign(info_.joints.size(), 0.0);
  torque_raw_cache_.assign(info_.joints.size(), 0.0);
  command_cache_.assign(info_.joints.size(), 0.0);
  position_state_handles_.assign(info_.joints.size(), nullptr);
  velocity_state_handles_.assign(info_.joints.size(), nullptr);
  effort_state_handles_.assign(info_.joints.size(), nullptr);
  velocity_raw_state_handles_.assign(info_.joints.size(), nullptr);
  torque_raw_state_handles_.assign(info_.joints.size(), nullptr);
  position_command_handles_.assign(info_.joints.size(), nullptr);
  joint_to_motor_index_.assign(info_.joints.size(), kNoMotor);
  joint_sign_.assign(info_.joints.size(), 1.0);
  joint_offset_rad_.assign(info_.joints.size(), 0.0);
  joint_reduction_ratio_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_rated_torque_nm_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_velocity_scale_rad_s_per_raw_.assign(
    info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_effort_scale_nm_per_raw_.assign(
    info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_position_wraparound_.assign(info_.joints.size(), false);
  joint_position_wrap_reference_rad_.assign(
    info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_turn_offset_rad_.assign(info_.joints.size(), 0.0);
  joint_turn_offset_initialized_.assign(info_.joints.size(), false);
  joint_snap_lower_limit_rad_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_snap_upper_limit_rad_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  if (!build_driver_config_from_ros2_control_info()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (auto executor = params.executor.lock()) {
    recovery_node_ = std::make_shared<rclcpp::Node>("right_arm_fault_recovery");
    executor->add_node(recovery_node_->get_node_base_interface());

    reset_fault_service_ = recovery_node_->create_service<std_srvs::srv::Trigger>(
      "/right_arm/reset_fault",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        response->success = reset_faults_manually(response->message);
      });

    enable_service_ = recovery_node_->create_service<std_srvs::srv::SetBool>(
      "/right_arm/enable",
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {
        response->success = set_motors_enabled_manually(request->data, response->message);
      });

    pdo_record_service_ = recovery_node_->create_service<std_srvs::srv::SetBool>(
      "/right_arm/record_pdo",
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {
        response->success = request->data ?
          driver_->startPdoRecording(response->message) :
          driver_->stopPdoRecording(response->message);
      });

    RCLCPP_INFO(
      logger(),
      "Hardware services ready: /right_arm/reset_fault, /right_arm/enable, "
      "/right_arm/record_pdo");
  } else {
    RCLCPP_ERROR(logger(), "Controller manager executor is unavailable; cannot create recovery services.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool EsMotorHardwareInterface::build_driver_config_from_ros2_control_info()
{
  try {
    driver_config_ = ethercat_driver::DriverConfig{};

    driver_config_.cycle_ns = parse_i64(
      get_string_param(info_.hardware_parameters, "cycle_ns", "1000000"), "cycle_ns");
    driver_config_.enable_dc = parse_bool(
      get_string_param(info_.hardware_parameters, "enable_dc", "true"), "enable_dc");
    driver_config_.assign_activate = static_cast<uint16_t>(parse_u64(
      get_string_param(info_.hardware_parameters, "assign_activate", "0x0300"), "assign_activate"));
    driver_config_.sync0_shift_ns = parse_i64(
      get_string_param(info_.hardware_parameters, "sync0_shift_ns", "622620"), "sync0_shift_ns");
    driver_config_.select_dc_reference_clock = parse_bool(
      get_string_param(info_.hardware_parameters, "select_dc_reference_clock", "true"), "select_dc_reference_clock");
    driver_config_.dc_reference_alias = static_cast<uint16_t>(parse_u64(
      get_string_param(info_.hardware_parameters, "dc_reference_alias", "0"), "dc_reference_alias"));
    driver_config_.dc_reference_slave_position = static_cast<uint16_t>(parse_u64(
      get_string_param(info_.hardware_parameters, "dc_reference_slave_position", "0"), "dc_reference_slave_position"));
    driver_config_.enable_realtime = parse_bool(
      get_string_param(info_.hardware_parameters, "enable_realtime", "true"), "enable_realtime");
    driver_config_.realtime_priority = static_cast<int>(parse_i64(
      get_string_param(info_.hardware_parameters, "realtime_priority", "80"), "realtime_priority"));
    driver_config_.realtime_cpu = static_cast<int>(parse_i64(
      get_string_param(info_.hardware_parameters, "realtime_cpu", "-1"), "realtime_cpu"));
    driver_config_.csp_direct_position_commands = parse_bool(
      get_string_param(info_.hardware_parameters, "csp_direct_position_commands", "false"),
      "csp_direct_position_commands");
    driver_config_.enable_velocity_torque_feedback = parse_bool(
      get_string_param(info_.hardware_parameters, "enable_velocity_torque_feedback", "false"),
      "enable_velocity_torque_feedback");
    velocity_raw_rpm_per_count_ = parse_double(
      get_string_param(info_.hardware_parameters, "velocity_raw_rpm_per_count", "0.1"),
      "velocity_raw_rpm_per_count");
    torque_raw_full_scale_ = parse_double(
      get_string_param(info_.hardware_parameters, "torque_raw_full_scale", "10000.0"),
      "torque_raw_full_scale");
    if (!std::isfinite(velocity_raw_rpm_per_count_) || velocity_raw_rpm_per_count_ <= 0.0 ||
      !std::isfinite(torque_raw_full_scale_) || torque_raw_full_scale_ <= 0.0)
    {
      RCLCPP_ERROR(
        logger(),
        "Feedback unit scales must be finite and positive: velocity_raw_rpm_per_count=%.9f "
        "torque_raw_full_scale=%.3f.",
        velocity_raw_rpm_per_count_, torque_raw_full_scale_);
      return false;
    }
    driver_config_.pdo_diagnostics_buffer_cycles = static_cast<size_t>(parse_u64(
      get_string_param(
        info_.hardware_parameters, "pdo_diagnostics_buffer_cycles", "16384"),
      "pdo_diagnostics_buffer_cycles"));
    driver_config_.pdo_diagnostics_auto_start = parse_bool(
      get_string_param(info_.hardware_parameters, "pdo_diagnostics_auto_start", "false"),
      "pdo_diagnostics_auto_start");
    driver_config_.pdo_diagnostics_max_cycles = parse_u64(
      get_string_param(info_.hardware_parameters, "pdo_diagnostics_max_cycles", "0"),
      "pdo_diagnostics_max_cycles");
    if (driver_config_.pdo_diagnostics_buffer_cycles < 1024U ||
      driver_config_.pdo_diagnostics_buffer_cycles > 1048576U)
    {
      RCLCPP_ERROR(
        logger(), "pdo_diagnostics_buffer_cycles must be in [1024, 1048576], got %zu.",
        driver_config_.pdo_diagnostics_buffer_cycles);
      return false;
    }
    if (driver_config_.pdo_diagnostics_auto_start &&
      driver_config_.pdo_diagnostics_max_cycles == 0U)
    {
      RCLCPP_WARN(
        logger(),
        "Automatic PDO recording has no cycle limit and can consume disk space continuously.");
    }
    enable_motors_on_activate_ = parse_bool(
      get_string_param(info_.hardware_parameters, "enable_motors_on_activate", "true"),
      "enable_motors_on_activate");

    joint_limit_snap_tolerance_rad_ = parse_double(
      get_string_param(info_.hardware_parameters, "joint_limit_snap_tolerance_rad", "0.005"),
      "joint_limit_snap_tolerance_rad");
    if (joint_limit_snap_tolerance_rad_ < 0.0) {
      RCLCPP_ERROR(logger(), "joint_limit_snap_tolerance_rad must be >= 0, got %.9f.", joint_limit_snap_tolerance_rad_);
      return false;
    }

    for (size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) {
      const auto & joint = info_.joints[joint_index];
      uint16_t slave_position = 0;
      bool use_ethercat = false;

      if (has_param(joint.parameters, "slave_position")) {
        slave_position = static_cast<uint16_t>(parse_u64(
          get_string_param(joint.parameters, "slave_position", "0"), joint.name + ".slave_position"));
        use_ethercat = true;
      } else {
        use_ethercat = right_arm_default_slave_position(joint.name, slave_position);
      }

      if (has_param(joint.parameters, "use_ethercat")) {
        use_ethercat = parse_bool(
          get_string_param(joint.parameters, "use_ethercat", "false"), joint.name + ".use_ethercat");
      }

      if (!use_ethercat) {
        continue;
      }

      if (!has_position_state_interface(joint)) {
        RCLCPP_ERROR(logger(), "Joint '%s' is configured as EtherCAT motor but has no position state_interface.", joint.name.c_str());
        return false;
      }

      ethercat_driver::MotorConfig motor_config{};
      motor_config.name = get_local_or_global_param(joint.parameters, info_.hardware_parameters, "motor_name", joint.name);
      motor_config.joint_name = joint.name;
      motor_config.address.master_index = static_cast<unsigned int>(parse_u64(
      get_local_or_global_param(joint.parameters, info_.hardware_parameters, "master_index", "1"), joint.name + ".master_index"));
      motor_config.address.alias = static_cast<uint16_t>(parse_u64(
        get_local_or_global_param(joint.parameters, info_.hardware_parameters, "alias", "0"), joint.name + ".alias"));
      motor_config.address.slave_position = slave_position;
      motor_config.address.vendor_id = static_cast<uint32_t>(parse_u64(
        get_local_or_global_param(joint.parameters, info_.hardware_parameters, "vendor_id", "0x00206688"), joint.name + ".vendor_id"));
      motor_config.address.product_code = static_cast<uint32_t>(parse_u64(
        get_local_or_global_param(joint.parameters, info_.hardware_parameters, "product_code", "0x26483052"), joint.name + ".product_code"));
      motor_config.configure_sdo = parse_bool(
        get_local_or_global_param(joint.parameters, info_.hardware_parameters, "configure_sdo", "true"), joint.name + ".configure_sdo");
      motor_config.configure_pdos = parse_bool(
        get_local_or_global_param(joint.parameters, info_.hardware_parameters, "configure_pdos", "true"), joint.name + ".configure_pdos");
      motor_config.mode_of_operation = static_cast<int8_t>(parse_i64(
        get_local_or_global_param(joint.parameters, info_.hardware_parameters, "mode_of_operation", "8"), joint.name + ".mode_of_operation"));
      motor_config.auto_enable = parse_bool(
        get_local_or_global_param(joint.parameters, info_.hardware_parameters, "auto_enable", "false"), joint.name + ".auto_enable");
      motor_config.max_velocity_counts_s = parse_double(
        get_local_or_global_param(joint.parameters, info_.hardware_parameters, "max_velocity_counts_s", "8000.0"), joint.name + ".max_velocity_counts_s");
      motor_config.max_accel_counts_s2 = parse_double(
        get_local_or_global_param(joint.parameters, info_.hardware_parameters, "max_accel_counts_s2", "20000.0"), joint.name + ".max_accel_counts_s2");
      motor_config.target_tolerance_counts = parse_i64(
        get_local_or_global_param(joint.parameters, info_.hardware_parameters, "target_tolerance_counts", "20"), joint.name + ".target_tolerance_counts");
      motor_config.counts_per_rev = parse_double(
        get_local_or_global_param(joint.parameters, info_.hardware_parameters, "counts_per_rev", "131072.0"), joint.name + ".counts_per_rev");

      const size_t motor_index = driver_config_.motors.size();
      driver_config_.motors.push_back(motor_config);
      joint_to_motor_index_[joint_index] = static_cast<int>(motor_index);
      joint_sign_[joint_index] = parse_double(
        get_local_or_global_param(joint.parameters, info_.hardware_parameters, "joint_sign", "1.0"), joint.name + ".joint_sign");
      joint_offset_rad_[joint_index] = parse_double(
        get_local_or_global_param(joint.parameters, info_.hardware_parameters, "position_offset_rad", "0.0"), joint.name + ".position_offset_rad");
      joint_reduction_ratio_[joint_index] = parse_double(
        get_local_or_global_param(
          joint.parameters, info_.hardware_parameters, "reduction_ratio", "nan"),
        joint.name + ".reduction_ratio");
      joint_rated_torque_nm_[joint_index] = parse_double(
        get_local_or_global_param(
          joint.parameters, info_.hardware_parameters, "rated_torque_nm", "nan"),
        joint.name + ".rated_torque_nm");

      if (driver_config_.enable_velocity_torque_feedback &&
        (!std::isfinite(joint_reduction_ratio_[joint_index]) ||
        joint_reduction_ratio_[joint_index] <= 0.0 ||
        !std::isfinite(joint_rated_torque_nm_[joint_index]) ||
        joint_rated_torque_nm_[joint_index] <= 0.0))
      {
        RCLCPP_ERROR(
          logger(),
          "Joint '%s' requires finite positive reduction_ratio and rated_torque_nm "
          "when velocity/torque feedback is enabled (got %.6f, %.6f).",
          joint.name.c_str(), joint_reduction_ratio_[joint_index],
          joint_rated_torque_nm_[joint_index]);
        return false;
      }

      joint_velocity_scale_rad_s_per_raw_[joint_index] =
        joint_sign_[joint_index] * velocity_raw_rpm_per_count_ * kTwoPi /
        (60.0 * joint_reduction_ratio_[joint_index]);
      joint_effort_scale_nm_per_raw_[joint_index] =
        joint_sign_[joint_index] * joint_rated_torque_nm_[joint_index] /
        torque_raw_full_scale_;
      driver_config_.motors[motor_index].velocity_scale_rad_s_per_raw =
        joint_velocity_scale_rad_s_per_raw_[joint_index];
      driver_config_.motors[motor_index].effort_scale_nm_per_raw =
        joint_effort_scale_nm_per_raw_[joint_index];
      driver_config_.motors[motor_index].position_scale_rad_per_count =
        joint_sign_[joint_index] * kTwoPi / motor_config.counts_per_rev;
      driver_config_.motors[motor_index].position_offset_rad =
        joint_offset_rad_[joint_index];
      joint_position_wraparound_[joint_index] = parse_bool(
        get_local_or_global_param(
          joint.parameters, info_.hardware_parameters, "position_wraparound", "false"),
        joint.name + ".position_wraparound");
      joint_position_wrap_reference_rad_[joint_index] = parse_double(
        get_local_or_global_param(
          joint.parameters, info_.hardware_parameters, "position_wrap_reference_rad", "nan"),
        joint.name + ".position_wrap_reference_rad");

      if (has_param(joint.parameters, "snap_lower_limit_rad")) {
        joint_snap_lower_limit_rad_[joint_index] = parse_double(
          get_string_param(joint.parameters, "snap_lower_limit_rad", "0.0"), joint.name + ".snap_lower_limit_rad");
      }
      if (has_param(joint.parameters, "snap_upper_limit_rad")) {
        joint_snap_upper_limit_rad_[joint_index] = parse_double(
          get_string_param(joint.parameters, "snap_upper_limit_rad", "0.0"), joint.name + ".snap_upper_limit_rad");
      }
      if (std::isfinite(joint_snap_lower_limit_rad_[joint_index]) &&
          std::isfinite(joint_snap_upper_limit_rad_[joint_index]) &&
          joint_snap_lower_limit_rad_[joint_index] > joint_snap_upper_limit_rad_[joint_index])
      {
        RCLCPP_ERROR(
          logger(), "Joint '%s' has snap_lower_limit_rad > snap_upper_limit_rad.", joint.name.c_str());
        return false;
      }

      if (std::abs(joint_sign_[joint_index]) < 1e-9) {
        RCLCPP_ERROR(logger(), "Joint '%s' has invalid joint_sign=0.", joint.name.c_str());
        return false;
      }

      RCLCPP_INFO(
        logger(),
        "EtherCAT joint mapping: joint=%s motor=%s slave_position=%u counts_per_rev=%.1f "
        "sign=%.1f offset_rad=%.6f reduction_ratio=%.3f rated_torque_nm=%.3f "
        "velocity_scale=%.12f_rad_s_per_raw effort_scale=%.9f_Nm_per_raw "
        "wraparound=%d wrap_reference_rad=%.6f",
        joint.name.c_str(), motor_config.name.c_str(), motor_config.address.slave_position,
        motor_config.counts_per_rev, joint_sign_[joint_index], joint_offset_rad_[joint_index],
        joint_reduction_ratio_[joint_index], joint_rated_torque_nm_[joint_index],
        joint_velocity_scale_rad_s_per_raw_[joint_index],
        joint_effort_scale_nm_per_raw_[joint_index],
        static_cast<int>(joint_position_wraparound_[joint_index]),
        joint_position_wrap_reference_rad_[joint_index]);

      if (std::isfinite(joint_snap_lower_limit_rad_[joint_index]) ||
          std::isfinite(joint_snap_upper_limit_rad_[joint_index]))
      {
        RCLCPP_INFO(
          logger(),
          "Joint limit snap enabled: joint=%s lower=%.6f upper=%.6f tolerance=%.6f rad",
          joint.name.c_str(),
          joint_snap_lower_limit_rad_[joint_index],
          joint_snap_upper_limit_rad_[joint_index],
          joint_limit_snap_tolerance_rad_);
      }
    }

    if (driver_config_.motors.empty()) {
      RCLCPP_ERROR(logger(), "No EtherCAT motors were configured. Add slave_position params or keep the right_arm joint names.");
      return false;
    }

    RCLCPP_INFO(logger(), "Configured %zu EtherCAT motors for ROS 2 Jazzy ros2_control.", driver_config_.motors.size());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Failed to build EtherCAT driver config: %s", e.what());
    return false;
  }
}

hardware_interface::CallbackReturn EsMotorHardwareInterface::
    on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  if (!driver_ || driver_config_.motors.empty()) {
    RCLCPP_ERROR(logger(), "on_configure failed: driver or motor config is empty.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger(), "Hardware configured. EtherCAT will start in on_activate().");
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool EsMotorHardwareInterface::wait_for_all_motors_ready(std::vector<ethercat_driver::MotorState> & states) const
{
  states.assign(driver_config_.motors.size(), ethercat_driver::MotorState{});

  // IgH 的 slave/domain 状态在底层线程里按 1 Hz 节流打印和刷新；7 个从站从 PREOP/SAFEOP
  // 全部切到 OP 有时会接近 10 s。这里等待更久，并要求 full working counter 稳定，避免
  // on_activate() 在最后两个从站刚切 OP 时误判失败。
  const int expected_domain_wc = static_cast<int>(driver_config_.motors.size() * 3U);
  int stable_ready_cycles = 0;

  for (int retry = 0; retry < 2400; ++retry) {  // 1200 * 25 ms = 30 s
    bool all_ready = true;
    bool domain_ready = false;

    for (size_t motor_index = 0; motor_index < driver_config_.motors.size(); ++motor_index) {
      const auto & motor = driver_config_.motors[motor_index];
      ethercat_driver::MotorState state{};
      if (!driver_->getMotorState(motor.address, state)) {
        all_ready = false;
        states[motor_index] = state;
        continue;
      }

      states[motor_index] = state;
      domain_ready =
        state.ethercat_ok &&
        state.domain_wc >= expected_domain_wc &&
        state.domain_wc_state == 2;

      if (!state.ethercat_ok ||
        !state.slave_online ||
        !state.slave_operational ||
        state.status_word == 0 ||
        !domain_ready)
      {
        all_ready = false;
      }
    }

    if (all_ready) {
      ++stable_ready_cycles;
      if (stable_ready_cycles >= 5) {
        return true;
      }
    } else {
      stable_ready_cycles = 0;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }

  return false;
}

hardware_interface::CallbackReturn EsMotorHardwareInterface::
    on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  const std::lock_guard<std::mutex> recovery_lock(recovery_mutex_);
  const bool recovering_from_hardware_fault = hardware_fault_latched_.load();

  // on_error() deliberately leaves the EtherCAT worker alive for diagnostics
  // and manual CiA402 recovery. Re-activation must reuse that worker.
  if (!driver_->isRunning() && !driver_->start(driver_config_)) {
    RCLCPP_ERROR(logger(), "Failed to start EtherCAT driver.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::vector<ethercat_driver::MotorState> initial_states;
  if (!wait_for_all_motors_ready(initial_states)) {
    RCLCPP_ERROR(logger(), "Failed to read ready state from all EtherCAT motors.");
    for (size_t i = 0; i < driver_config_.motors.size(); ++i) {
      const auto & motor = driver_config_.motors[i];
      const auto & state = initial_states[i];
      RCLCPP_ERROR(
        logger(),
        "  motor=%s slave_position=%u ethercat_ok=%d online=%d operational=%d status_word=0x%04X al=0x%02X domain_wc=%d wc_state=%d",
        motor.name.c_str(), motor.address.slave_position, state.ethercat_ok, state.slave_online,
        state.slave_operational, state.status_word, state.slave_al_state, state.domain_wc, state.domain_wc_state);
    }
    driver_->stop();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 关键：启动后先用 7 个电机的真实当前位置初始化 ros2_control 命令，
  // 再把底层目标写成当前 counts，最后才 setEnable(true)。这样控制器默认 0 不会把电机拉回零点。
  // 每次激活都重新学习本次上电的整圈分支，因为无电池绝对编码器断电后只保留单圈角度。
  std::fill(joint_turn_offset_rad_.begin(), joint_turn_offset_rad_.end(), 0.0);
  std::fill(joint_turn_offset_initialized_.begin(), joint_turn_offset_initialized_.end(), false);
  for (size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) {
    const int motor_index = joint_to_motor_index_[joint_index];
    if (motor_index == kNoMotor) {
      continue;
    }

    const auto & motor = driver_config_.motors[static_cast<size_t>(motor_index)];
    const auto & state = initial_states[static_cast<size_t>(motor_index)];
    const double actual_rad = counts_to_radians(state.actual_counts, motor, joint_index);

    set_position_state(joint_index, actual_rad);
    set_position_command(joint_index, actual_rad);

    if (!driver_->setAbsoluteTargetCounts(motor.address, state.actual_counts)) {
      RCLCPP_ERROR(logger(), "Failed to set initial hold target for joint '%s'.", info_.joints[joint_index].name.c_str());
      driver_->stop();
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Wait briefly for the low-rate 0x603F mailbox diagnostics before deciding
  // whether a startup fault is the expected communication-loss latch.
  const auto diagnostic_deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
  while (std::chrono::steady_clock::now() < diagnostic_deadline) {
    bool all_fault_diagnostics_ready = true;
    for (const auto & motor : driver_config_.motors) {
      ethercat_driver::MotorState state{};
      if (!driver_->getMotorState(motor.address, state) ||
        ((state.fault || state.fault_reaction_active) && !state.error_code_valid))
      {
        all_fault_diagnostics_ready = false;
        break;
      }
    }
    if (all_fault_diagnostics_ready) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Some drives report the expected communication fault (0x7500) after the
  // EtherCAT master has been stopped.  Clear a fault exactly once here, while
  // the target is already holding the measured position and before any motor
  // is enabled.  Runtime faults are still latched by read() and are never
  // reset automatically.
  bool startup_fault_reset_requested = false;
  for (size_t i = 0; i < driver_config_.motors.size(); ++i) {
    const auto & state = initial_states[i];
    if (!state.fault && !state.fault_reaction_active) {
      continue;
    }
    const auto & motor = driver_config_.motors[i];
    ethercat_driver::MotorState diagnosed_state{};
    if (!driver_->getMotorState(motor.address, diagnosed_state) ||
      !diagnosed_state.error_code_valid)
    {
      RCLCPP_ERROR(logger(), "Refusing startup fault reset for motor '%s': 0x603F is unavailable.", motor.name.c_str());
      latch_hardware_fault("startup fault diagnostics unavailable for motor '" + motor.name + "'");
      return hardware_interface::CallbackReturn::ERROR;
    }
    const std::string decoded_faults =
      zhishu_robot_hardware::format_motor_fault_mask(diagnosed_state.error_code);
    // A non-zero vendor fault mask can contain hardware or mechanical faults
    // (for example encoder failure or motor stall). Never clear it
    // automatically during startup; require an explicit operator recovery.
    if (diagnosed_state.error_code != 0x0000) {
      RCLCPP_ERROR(
        logger(),
        "Refusing startup fault reset for motor '%s': error_code=0x%04X "
        "status_word=0x%04X decoded_faults=[%s].",
        motor.name.c_str(), diagnosed_state.error_code, diagnosed_state.status_word,
        decoded_faults.c_str());
      latch_hardware_fault("startup vendor fault requires manual recovery for motor '" + motor.name + "'");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!driver_->requestFaultReset(motor.address)) {
      RCLCPP_ERROR(logger(), "Failed to request startup fault reset for motor '%s'.", motor.name.c_str());
      latch_hardware_fault("startup fault reset request failed for motor '" + motor.name + "'");
      return hardware_interface::CallbackReturn::ERROR;
    }
    startup_fault_reset_requested = true;
    RCLCPP_WARN(
      logger(),
      "One-shot startup fault reset requested for motor '%s' while motion remains disabled "
      "(status_word=0x%04X error_code=0x%04X decoded_faults=[%s]).",
      motor.name.c_str(), diagnosed_state.status_word, diagnosed_state.error_code,
      decoded_faults.c_str());
  }

  if (startup_fault_reset_requested) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    bool all_faults_cleared = false;
    int stable_clear_cycles = 0;
    while (std::chrono::steady_clock::now() < deadline) {
      bool clear_this_cycle = true;
      for (const auto & motor : driver_config_.motors) {
        ethercat_driver::MotorState state{};
        if (!driver_->getMotorState(motor.address, state) || !state.ethercat_ok ||
          state.fault || state.fault_reaction_active || state.quick_stop_active)
        {
          clear_this_cycle = false;
          break;
        }
      }
      stable_clear_cycles = clear_this_cycle ? stable_clear_cycles + 1 : 0;
      if (stable_clear_cycles >= 5) {
        all_faults_cleared = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!all_faults_cleared) {
      RCLCPP_ERROR(logger(), "Startup fault reset did not clear every motor; motors will remain disabled.");
      latch_hardware_fault("startup one-shot fault reset timed out");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger(), "One-shot startup fault reset completed; proceeding with normal activation.");
  }

  const bool enable_on_this_activation =
    enable_motors_on_activate_ && !recovering_from_hardware_fault;
  if (enable_on_this_activation) {
    for (const auto & motor : driver_config_.motors) {
      if (!driver_->setEnable(motor.address, true)) {
        RCLCPP_ERROR(logger(), "Failed to enable motor '%s'.", motor.name.c_str());
        driver_->stop();
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  } else {
    RCLCPP_WARN(
      logger(),
      recovering_from_hardware_fault ?
      "Hardware fault recovery activation: all motors remain Operation Disabled; "
      "explicit operator enable is required." :
      "EtherCAT communication-only mode: all motors remain Operation Disabled; "
      "commands cannot move the arm.");
  }

  safety_stop_reported_.assign(driver_config_.motors.size(), false);
  last_fault_error_code_valid_.assign(driver_config_.motors.size(), false);
  last_fault_error_code_.assign(driver_config_.motors.size(), 0);
  manual_motion_enabled_.store(enable_on_this_activation);
  hardware_fault_latched_.store(false);
  // Publish the active state last so service callbacks cannot observe partially
  // initialized diagnostic storage.
  activated_.store(true);

  for (size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) {
    const int motor_index = joint_to_motor_index_[joint_index];
    if (motor_index == kNoMotor) {
      continue;
    }
    const auto & motor = driver_config_.motors[static_cast<size_t>(motor_index)];
    RCLCPP_INFO(
      logger(),
      "Activated joint '%s' on slave_position=%u, hold at %.6f rad / %.2f deg.",
      info_.joints[joint_index].name.c_str(), motor.address.slave_position,
      position_cache_[joint_index], position_cache_[joint_index] * 180.0 / 3.14159265358979323846);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EsMotorHardwareInterface::
    on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  const std::lock_guard<std::mutex> recovery_lock(recovery_mutex_);

  activated_.store(false);
  manual_motion_enabled_.store(false);

  if (driver_) {
    for (const auto & motor : driver_config_.motors) {
      driver_->setEnable(motor.address, false);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    driver_->stop();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EsMotorHardwareInterface::
    on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  const std::lock_guard<std::mutex> recovery_lock(recovery_mutex_);

  activated_.store(false);
  manual_motion_enabled_.store(false);
  hardware_fault_latched_.store(true);
  if (driver_) {
    for (const auto & motor : driver_config_.motors) {
      driver_->setEnable(motor.address, false);
    }
  }

  RCLCPP_ERROR(
    logger(),
    "Hardware entered error processing. Controllers using this arm are stopped; "
    "EtherCAT remains active for diagnostics and manual fault reset.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr> EsMotorHardwareInterface::
    on_export_state_interfaces()
{
  auto state_interfaces = hardware_interface::SystemInterface::on_export_state_interfaces();

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & interface : info_.joints[i].state_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        const std::string interface_name =
          info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
        position_state_handles_[i] = get_state_interface_handle(interface_name);
        if (const auto initial_value = position_state_handles_[i]->get_optional<double>()) {
          position_cache_[i] = *initial_value;
        }
      } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
        const std::string interface_name =
          info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
        velocity_state_handles_[i] = get_state_interface_handle(interface_name);
      } else if (interface.name == hardware_interface::HW_IF_EFFORT) {
        const std::string interface_name =
          info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT;
        effort_state_handles_[i] = get_state_interface_handle(interface_name);
      } else if (interface.name == kVelocityRawInterface) {
        const std::string interface_name = info_.joints[i].name + "/" + kVelocityRawInterface;
        velocity_raw_state_handles_[i] = get_state_interface_handle(interface_name);
      } else if (interface.name == kTorqueRawInterface) {
        const std::string interface_name = info_.joints[i].name + "/" + kTorqueRawInterface;
        torque_raw_state_handles_[i] = get_state_interface_handle(interface_name);
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> EsMotorHardwareInterface::
    on_export_command_interfaces()
{
  auto command_interfaces = hardware_interface::SystemInterface::on_export_command_interfaces();

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & interface : info_.joints[i].command_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        const std::string interface_name =
          info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
        position_command_handles_[i] = get_command_interface_handle(interface_name);
        set_position_command(i, position_cache_[i]);
      }
    }
  }

  return command_interfaces;
}

bool EsMotorHardwareInterface::set_position_state(size_t joint_index, double value)
{
  position_cache_[joint_index] = value;
  const auto & handle = position_state_handles_[joint_index];
  return handle ? handle->set_value(value, false) : false;
}

bool EsMotorHardwareInterface::set_velocity_state(size_t joint_index, double value)
{
  velocity_cache_[joint_index] = value;
  const auto & handle = velocity_state_handles_[joint_index];
  return handle ? handle->set_value(value, false) : false;
}

bool EsMotorHardwareInterface::set_effort_state(size_t joint_index, double value)
{
  effort_cache_[joint_index] = value;
  const auto & handle = effort_state_handles_[joint_index];
  return handle ? handle->set_value(value, false) : false;
}

bool EsMotorHardwareInterface::set_velocity_raw_state(size_t joint_index, double value)
{
  velocity_raw_cache_[joint_index] = value;
  const auto & handle = velocity_raw_state_handles_[joint_index];
  return handle ? handle->set_value(value, false) : false;
}

bool EsMotorHardwareInterface::set_torque_raw_state(size_t joint_index, double value)
{
  torque_raw_cache_[joint_index] = value;
  const auto & handle = torque_raw_state_handles_[joint_index];
  return handle ? handle->set_value(value, false) : false;
}

bool EsMotorHardwareInterface::set_position_command(size_t joint_index, double value)
{
  command_cache_[joint_index] = value;
  const auto & handle = position_command_handles_[joint_index];
  return handle ? handle->set_value(value, false) : false;
}

bool EsMotorHardwareInterface::refresh_position_command(size_t joint_index)
{
  const auto & handle = position_command_handles_[joint_index];
  if (!handle) {
    return false;
  }
  const auto value = handle->get_optional<double>();
  if (!value) {
    return false;
  }
  command_cache_[joint_index] = *value;
  return true;
}

double EsMotorHardwareInterface::counts_to_radians(
  int32_t counts,
  const ethercat_driver::MotorConfig & motor,
  size_t joint_index)
{
  const double raw_position =
    joint_sign_[joint_index] * (static_cast<double>(counts) / motor.counts_per_rev * kTwoPi) +
    joint_offset_rad_[joint_index];
  double position = raw_position;

  if (joint_position_wraparound_[joint_index]) {
    double reference = position_cache_[joint_index];
    if (!joint_turn_offset_initialized_[joint_index] &&
      std::isfinite(joint_position_wrap_reference_rad_[joint_index]))
    {
      reference = joint_position_wrap_reference_rad_[joint_index];
    }
    if (!std::isfinite(reference)) {
      reference = 0.0;
    }

    // Pick the raw + n*2*pi value nearest the expected/previous ROS position.
    // Re-evaluating against the previous sample also preserves continuity if a
    // drive resets its single-turn branch while ros2_control is still running.
    const double learned_turn_offset =
      std::round((reference - raw_position) / kTwoPi) * kTwoPi;
    const bool offset_changed =
      !joint_turn_offset_initialized_[joint_index] ||
      std::abs(learned_turn_offset - joint_turn_offset_rad_[joint_index]) > 1e-9;

    if (offset_changed) {
      const bool was_initialized = joint_turn_offset_initialized_[joint_index];
      joint_turn_offset_rad_[joint_index] = learned_turn_offset;
      if (!driver_->setDiagnosticPositionOffsetRad(
          motor.address, joint_offset_rad_[joint_index] + learned_turn_offset))
      {
        RCLCPP_WARN(
          logger(), "Failed to update PDO diagnostic position branch for joint '%s'.",
          info_.joints[joint_index].name.c_str());
      }
      RCLCPP_INFO(
        logger(),
        "%s position branch for joint '%s': raw=%.6f rad reference=%.6f rad "
        "turn_offset=%+.6f rad reported=%.6f rad.",
        was_initialized ? "Adjusted" : "Initialized",
        info_.joints[joint_index].name.c_str(), raw_position, reference,
        learned_turn_offset, raw_position + learned_turn_offset);
    }
    joint_turn_offset_initialized_[joint_index] = true;
    position = raw_position + joint_turn_offset_rad_[joint_index];
  }

  // Only absorb tiny *out-of-range* encoder/zero-point errors. In-range values are
  // left untouched, and a real violation larger than the tolerance remains visible
  // to ros2_control / MoveIt instead of being silently hidden.
  if (joint_limit_snap_tolerance_rad_ > 0.0) {
    const double lower = joint_snap_lower_limit_rad_[joint_index];
    if (std::isfinite(lower) && position < lower &&
        position >= lower - joint_limit_snap_tolerance_rad_)
    {
      position = lower;
    }

    const double upper = joint_snap_upper_limit_rad_[joint_index];
    if (std::isfinite(upper) && position > upper &&
        position <= upper + joint_limit_snap_tolerance_rad_)
    {
      position = upper;
    }
  }

  return position;
}

int32_t EsMotorHardwareInterface::radians_to_counts(
  double radians,
  const ethercat_driver::MotorConfig & motor,
  size_t joint_index) const
{
  // Undo the same learned branch correction used on the state side. Without this,
  // e.g. raw=-350 deg/report=10 deg followed by a 10 deg command would request a
  // real +360 deg move.
  const double raw_radians = radians - joint_turn_offset_rad_[joint_index];
  const double motor_radians =
    (raw_radians - joint_offset_rad_[joint_index]) / joint_sign_[joint_index];
  const double counts = motor_radians / kTwoPi * motor.counts_per_rev;
  return clamp_i64_to_i32(static_cast<int64_t>(std::llround(counts)));
}

bool EsMotorHardwareInterface::reset_faults_manually(std::string & message)
{
  const std::lock_guard<std::mutex> lock(recovery_mutex_);

  if (!driver_ || !driver_->isRunning() || driver_config_.motors.empty()) {
    message = "EtherCAT is not running; manual reset is unavailable";
    return false;
  }

  // Fault reset never implies re-enable. Lock out write() first and explicitly
  // disable every axis before issuing CiA402 controlword 0x0080 requests.
  manual_motion_enabled_.store(false);
  for (const auto & motor : driver_config_.motors) {
    driver_->setEnable(motor.address, false);
  }

  size_t recovery_count = 0;
  for (const auto & motor : driver_config_.motors) {
    ethercat_driver::MotorState state{};
    if (!driver_->getMotorState(motor.address, state)) {
      message = "failed to read motor state for " + motor.name;
      return false;
    }
    if (!state.ethercat_ok || !state.slave_online) {
      message = "EtherCAT is not healthy for " + motor.name;
      return false;
    }
    if (state.fault || state.fault_reaction_active) {
      const std::string decoded_faults = state.error_code_valid ?
        zhishu_robot_hardware::format_motor_fault_mask(state.error_code) : "unavailable";
      RCLCPP_WARN(
        logger(),
        "Manual fault reset requested for motor '%s': status_word=0x%04X "
        "error_code=%s0x%04X decoded_faults=[%s]",
        motor.name.c_str(), state.status_word, state.error_code_valid ? "" : "unavailable/",
        state.error_code, decoded_faults.c_str());
      if (!driver_->requestFaultReset(motor.address)) {
        message = "failed to request fault reset for " + motor.name;
        return false;
      }
      ++recovery_count;
    } else if (state.quick_stop_active) {
      RCLCPP_WARN(
        logger(),
        "Manual Quick Stop release requested for motor '%s': status_word=0x%04X",
        motor.name.c_str(), state.status_word);
      if (!driver_->requestQuickStopRelease(motor.address)) {
        message = "failed to request Quick Stop release for " + motor.name;
        return false;
      }
      ++recovery_count;
    }
  }

  if (recovery_count == 0) {
    message = "no CiA402 Fault/FaultReactionActive/QuickStopActive state found; motors remain disabled";
    return true;
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    bool clear = true;
    for (const auto & motor : driver_config_.motors) {
      ethercat_driver::MotorState state{};
      if (!driver_->getMotorState(motor.address, state) || !state.ethercat_ok ||
        !state.slave_online || !state.slave_operational || state.status_word == 0U ||
        state.fault || state.fault_reaction_active || state.quick_stop_active)
      {
        clear = false;
        break;
      }
    }
    if (clear) {
      message = "safety-state recovery completed for " + std::to_string(recovery_count) +
        " motor(s); motors are still disabled, call /right_arm/enable with data=true";
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  message = "safety-state recovery timed out; motors remain disabled";
  return false;
}

void EsMotorHardwareInterface::latch_hardware_fault(const std::string & reason)
{
  manual_motion_enabled_.store(false);
  const bool first_report = !hardware_fault_latched_.exchange(true);
  for (const auto & motor : driver_config_.motors) {
    driver_->setEnable(motor.address, false);
  }
  if (first_report) {
    RCLCPP_ERROR(
      logger(),
      "Hardware fault propagated to controller_manager: %s. "
      "The active trajectory will be aborted and motors remain disabled.",
      reason.c_str());
  }
}

bool EsMotorHardwareInterface::set_motors_enabled_manually(bool enable, std::string & message)
{
  const std::lock_guard<std::mutex> lock(recovery_mutex_);

  if (!activated_.load() || !driver_ || driver_config_.motors.empty()) {
    message = "hardware is not active";
    return false;
  }

  if (!enable) {
    manual_motion_enabled_.store(false);
    bool ok = true;
    for (const auto & motor : driver_config_.motors) {
      ok = driver_->setEnable(motor.address, false) && ok;
    }
    message = ok ? "motors disabled" : "one or more motors could not be disabled";
    return ok;
  }

  // Validate every axis and synchronize both ros2_control and CSP targets to the
  // measured position before enabling. This prevents an immediate jump to a stale target.
  std::vector<ethercat_driver::MotorState> states(driver_config_.motors.size());
  for (size_t i = 0; i < driver_config_.motors.size(); ++i) {
    const auto & motor = driver_config_.motors[i];
    auto & state = states[i];
    if (!driver_->getMotorState(motor.address, state)) {
      message = "failed to read motor state for " + motor.name;
      return false;
    }
    if (!state.ethercat_ok || !state.slave_online || !state.slave_operational ||
      state.status_word == 0U)
    {
      message = "EtherCAT/slave is not ready for " + motor.name;
      return false;
    }
    if (state.fault || state.fault_reaction_active || state.quick_stop_active) {
      std::ostringstream oss;
      oss << "motor " << motor.name << " is not safe to enable: status_word=0x"
          << std::hex << state.status_word << " error_code=0x" << state.error_code
          << " decoded_faults=["
          << (state.error_code_valid ?
          zhishu_robot_hardware::format_motor_fault_mask(state.error_code) : "unavailable")
          << "]";
      message = oss.str();
      return false;
    }
  }

  manual_motion_enabled_.store(false);
  // write() is still running while the motion latch is false. It continuously
  // keeps each CSP target at the measured counts without rewriting the
  // controller-owned ros2_control command handle.
  for (size_t i = 0; i < driver_config_.motors.size(); ++i) {
    const auto & motor = driver_config_.motors[i];
    const auto & state = states[i];
    if (!driver_->setAbsoluteTargetCounts(motor.address, state.actual_counts)) {
      message = "failed to synchronize CSP target for " + motor.name;
      return false;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  for (const auto & motor : driver_config_.motors) {
    if (!driver_->setEnable(motor.address, true)) {
      for (const auto & rollback_motor : driver_config_.motors) {
        driver_->setEnable(rollback_motor.address, false);
      }
      message = "failed to request enable for " + motor.name;
      return false;
    }
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  int stable_cycles = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    bool enabled_this_cycle = true;
    for (const auto & motor : driver_config_.motors) {
      ethercat_driver::MotorState state{};
      if (!driver_->getMotorState(motor.address, state) || !state.ethercat_ok ||
        state.fault || state.fault_reaction_active || state.quick_stop_active ||
        !state.operation_enabled)
      {
        enabled_this_cycle = false;
        break;
      }
    }
    stable_cycles = enabled_this_cycle ? stable_cycles + 1 : 0;
    if (stable_cycles >= 5) {
      manual_motion_enabled_.store(true);
      message = "all motors are Operation Enabled; release/re-acquire VR grip before motion";
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  for (const auto & motor : driver_config_.motors) {
    driver_->setEnable(motor.address, false);
  }
  manual_motion_enabled_.store(false);
  message = "enable timed out; motors were disabled again";
  return false;
}

hardware_interface::return_type EsMotorHardwareInterface::
    read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;

  if (!driver_ || driver_config_.motors.empty()) {
    return hardware_interface::return_type::OK;
  }

  for (size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) {
    const int motor_index = joint_to_motor_index_[joint_index];
    if (motor_index == kNoMotor) {
      continue;
    }

    const auto & motor = driver_config_.motors[static_cast<size_t>(motor_index)];
    ethercat_driver::MotorState state{};
    if (!driver_->getMotorState(motor.address, state)) {
      latch_hardware_fault("getMotorState failed for joint '" + info_.joints[joint_index].name + "'");
      return hardware_interface::return_type::ERROR;
    }

    const int expected_domain_wc = static_cast<int>(driver_config_.motors.size() * 3U);
    const bool bus_ready =
      state.ethercat_ok && state.slave_online && state.slave_operational &&
      state.status_word != 0U && state.domain_wc >= expected_domain_wc &&
      state.domain_wc_state == 2;
    if (!bus_ready) {
      std::ostringstream reason;
      reason << "EtherCAT not ready at joint '" << info_.joints[joint_index].name
             << "': online=" << state.slave_online
             << " op=" << state.slave_operational
             << " al=0x" << std::hex << static_cast<unsigned int>(state.slave_al_state)
             << std::dec << " domain_wc=" << state.domain_wc << "/" << expected_domain_wc
             << " wc_state=" << state.domain_wc_state;
      latch_hardware_fault(reason.str());
      return hardware_interface::return_type::ERROR;
    }

    if (state.status_word != 0) {
      set_position_state(
        joint_index, counts_to_radians(state.actual_counts, motor, joint_index));

      if (driver_config_.enable_velocity_torque_feedback) {
        const double velocity_raw = static_cast<double>(state.actual_velocity_raw);
        const double torque_raw = static_cast<double>(state.actual_torque_raw);
        set_velocity_raw_state(joint_index, velocity_raw);
        set_torque_raw_state(joint_index, torque_raw);
        // GH1 manual: 0x606C is 0.1 motor RPM/count and 0x6077 is
        // 1/10000 rated current/count. The module data sheets provide the
        // 101:1 reduction and rated output torque used for ROS SI units.
        set_velocity_state(
          joint_index, velocity_raw * joint_velocity_scale_rad_s_per_raw_[joint_index]);
        set_effort_state(
          joint_index, torque_raw * joint_effort_scale_nm_per_raw_[joint_index]);
      }
    }

    if (state.fault || state.fault_reaction_active || state.quick_stop_active) {
      const size_t diagnostic_index = static_cast<size_t>(motor_index);
      const bool error_code_changed = state.error_code_valid &&
        (!last_fault_error_code_valid_[diagnostic_index] ||
        state.error_code != last_fault_error_code_[diagnostic_index]);
      if (!safety_stop_reported_[diagnostic_index] || error_code_changed) {
        const std::string decoded_faults = state.error_code_valid ?
          zhishu_robot_hardware::format_motor_fault_mask(state.error_code) : "unavailable";
        RCLCPP_ERROR(
          logger(),
          "Safety stop latched for joint '%s': fault=%d fault_reaction=%d quick_stop=%d "
          "status_word=0x%04X error_code=%s0x%04X decoded_faults=[%s]. "
          "EtherCAT stays active for diagnostics, "
          "but motion remains disabled until manual reset and explicit reactivation.",
          info_.joints[joint_index].name.c_str(), state.fault, state.fault_reaction_active,
          state.quick_stop_active, state.status_word, state.error_code_valid ? "" : "unavailable/",
          state.error_code, decoded_faults.c_str());
        safety_stop_reported_[diagnostic_index] = true;
        last_fault_error_code_valid_[diagnostic_index] = state.error_code_valid;
        last_fault_error_code_[diagnostic_index] = state.error_code;
      }
      latch_hardware_fault(
        "drive safety state at joint '" + info_.joints[joint_index].name + "'");
      return hardware_interface::return_type::ERROR;
    }

    // Only the ros2_control read thread owns these diagnostic vectors. Reset the
    // per-axis report latch after the safety state has genuinely cleared so a
    // later occurrence (including the same 0x603F code) is logged again.
    const size_t diagnostic_index = static_cast<size_t>(motor_index);
    safety_stop_reported_[diagnostic_index] = false;
    last_fault_error_code_valid_[diagnostic_index] = false;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EsMotorHardwareInterface::
    write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;

  if (!activated_.load() || !driver_ || driver_config_.motors.empty()) {
    return hardware_interface::return_type::OK;
  }

  const bool motion_enabled = manual_motion_enabled_.load();

  // 7 个电机、每个从站当前 PDO 工作计数为 3，所以满 WC 应为 21。
  // EtherCAT 出现 WC 不满、SAFEOP、同步错误时，禁止继续追 MoveIt 下发的轨迹点，
  // 先锁住真实当前位置，避免电机抖动、掉使能后恢复时突然跳动。
  const int expected_domain_wc = static_cast<int>(driver_config_.motors.size() * 3U);
  static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);

  for (size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) {
    // Controllers update command handles from a different thread. A failed try-lock
    // keeps the previous safe command for this control cycle.
    refresh_position_command(joint_index);

    const int motor_index = joint_to_motor_index_[joint_index];
    if (motor_index == kNoMotor) {
      // 非 EtherCAT 关节仍然导出 command_interface，用于让左臂/夹爪等控制器可以启动。
      // 这里把命令值回写到状态值，等价于 fake/passive joint，不会下发到真实 EtherCAT 电机。
      if (has_position_command_interface(info_.joints[joint_index]) &&
        std::isfinite(command_cache_[joint_index]))
      {
        set_position_state(joint_index, command_cache_[joint_index]);
      }
      continue;
    }

    const auto & motor = driver_config_.motors[static_cast<size_t>(motor_index)];
    ethercat_driver::MotorState state{};
    if (!driver_->getMotorState(motor.address, state)) {
      latch_hardware_fault(
        "getMotorState failed for joint '" + info_.joints[joint_index].name + "'");
      return hardware_interface::return_type::ERROR;
    }

    // status_word != 0 时才认为 actual_counts 有效；否则保持上一次位置，避免状态跳到 0。
    double hold_rad = position_cache_[joint_index];
    if (!std::isfinite(hold_rad)) {
      hold_rad = 0.0;
    }
    if (state.status_word != 0U) {
      hold_rad = counts_to_radians(state.actual_counts, motor, joint_index);
      set_position_state(joint_index, hold_rad);
    }

    bool command_enable = false;
    const bool enable_status_ok = driver_->getEnabledStatus(motor.address, command_enable);

    const bool domain_ready =
      state.ethercat_ok &&
      state.domain_wc >= expected_domain_wc &&
      state.domain_wc_state == 2;

    const bool slave_ready =
      state.slave_online &&
      state.slave_operational &&
      state.status_word != 0U;

    const bool bus_ready = domain_ready && slave_ready;

    // EtherCAT 不健康时：不接受控制器新命令，只把目标锁到当前真实位置。
    // 对应你的 dmesg：WC 0/21、UNMATCHED/SKIPPED、SAFEOP、0x001A Synchronization error。
    if (!bus_ready) {
      if (state.status_word != 0U) {
        driver_->setAbsoluteTargetCounts(motor.address, state.actual_counts);
      }

      RCLCPP_WARN_THROTTLE(
        logger(), throttle_clock, 1000,
        "EtherCAT not ready for joint '%s': motor=%s slave_position=%u "
        "ethercat_ok=%d online=%d op=%d status_word=0x%04X al=0x%02X "
        "domain_wc=%d/%d wc_state=%d. Holding current position.",
        info_.joints[joint_index].name.c_str(),
        motor.name.c_str(),
        motor.address.slave_position,
        state.ethercat_ok,
        state.slave_online,
        state.slave_operational,
        state.status_word,
        state.slave_al_state,
        state.domain_wc,
        expected_domain_wc,
        state.domain_wc_state);
      std::ostringstream reason;
      reason << "EtherCAT not ready at joint '" << info_.joints[joint_index].name
             << "': online=" << state.slave_online
             << " op=" << state.slave_operational
             << " al=0x" << std::hex << static_cast<unsigned int>(state.slave_al_state)
             << std::dec << " domain_wc=" << state.domain_wc << "/" << expected_domain_wc
             << " wc_state=" << state.domain_wc_state;
      latch_hardware_fault(reason.str());
      return hardware_interface::return_type::ERROR;
    }

    // 未真正进入 OperationEnabled 前，持续锁当前真实位置，不接受控制器默认 0。
    if (!motion_enabled || !enable_status_ok || !command_enable || !state.operation_enabled ||
      !std::isfinite(command_cache_[joint_index]))
    {
      driver_->setAbsoluteTargetCounts(motor.address, state.actual_counts);

      RCLCPP_WARN_THROTTLE(
        logger(), throttle_clock, 1000,
        "Joint '%s' is not command-enabled yet: motion_enabled=%d enable_status_ok=%d "
        "command_enable=%d operation_enabled=%d status_word=0x%04X. Holding current position.",
        info_.joints[joint_index].name.c_str(),
        motion_enabled,
        enable_status_ok,
        command_enable,
        state.operation_enabled,
        state.status_word);
      continue;
    }

    const int32_t target_counts =
      radians_to_counts(command_cache_[joint_index], motor, joint_index);
    if (!driver_->setAbsoluteTargetCounts(motor.address, target_counts)) {
      driver_->setAbsoluteTargetCounts(motor.address, state.actual_counts);

      RCLCPP_WARN_THROTTLE(
        logger(), throttle_clock, 1000,
        "setAbsoluteTargetCounts failed for joint '%s'. Holding current position.",
        info_.joints[joint_index].name.c_str());
      latch_hardware_fault(
        "setAbsoluteTargetCounts failed for joint '" + info_.joints[joint_index].name + "'");
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace es_motor_hardware

PLUGINLIB_EXPORT_CLASS(es_motor_hardware::EsMotorHardwareInterface, hardware_interface::SystemInterface)
