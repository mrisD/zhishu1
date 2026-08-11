#include "zhishu_robot_hardware/es_motor_hardware_interface.hpp"

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
  command_cache_.assign(info_.joints.size(), 0.0);
  position_state_handles_.assign(info_.joints.size(), nullptr);
  position_command_handles_.assign(info_.joints.size(), nullptr);
  joint_to_motor_index_.assign(info_.joints.size(), kNoMotor);
  joint_sign_.assign(info_.joints.size(), 1.0);
  joint_offset_rad_.assign(info_.joints.size(), 0.0);
  joint_snap_lower_limit_rad_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_snap_upper_limit_rad_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  if (!build_driver_config_from_ros2_control_info()) {
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
      motor_config.auto_fault_reset = parse_bool(
        get_local_or_global_param(joint.parameters, info_.hardware_parameters, "auto_fault_reset", "true"), joint.name + ".auto_fault_reset");
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
        "EtherCAT joint mapping: joint=%s motor=%s slave_position=%u counts_per_rev=%.1f sign=%.1f offset_rad=%.6f",
        joint.name.c_str(), motor_config.name.c_str(), motor_config.address.slave_position,
        motor_config.counts_per_rev, joint_sign_[joint_index], joint_offset_rad_[joint_index]);

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

  if (!driver_->start(driver_config_)) {
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

  if (enable_motors_on_activate_) {
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
      "EtherCAT communication-only mode: all motors remain Operation Disabled; commands cannot move the arm.");
  }

  activated_ = true;

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

  activated_ = false;

  if (driver_) {
    for (const auto & motor : driver_config_.motors) {
      driver_->setEnable(motor.address, false);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    driver_->stop();
  }

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
  size_t joint_index) const
{
  double position =
    joint_sign_[joint_index] * (static_cast<double>(counts) / motor.counts_per_rev * kTwoPi) +
    joint_offset_rad_[joint_index];

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
  const double motor_radians = (radians - joint_offset_rad_[joint_index]) / joint_sign_[joint_index];
  const double counts = motor_radians / kTwoPi * motor.counts_per_rev;
  return clamp_i64_to_i32(static_cast<int64_t>(std::llround(counts)));
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
    if (driver_->getMotorState(motor.address, state) && state.status_word != 0) {
      set_position_state(
        joint_index, counts_to_radians(state.actual_counts, motor, joint_index));
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EsMotorHardwareInterface::
    write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;

  if (!activated_ || !driver_ || driver_config_.motors.empty()) {
    return hardware_interface::return_type::OK;
  }

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
      if (std::isfinite(position_cache_[joint_index])) {
        set_position_command(joint_index, position_cache_[joint_index]);
      }
      RCLCPP_WARN_THROTTLE(
        logger(), throttle_clock, 1000,
        "getMotorState failed for joint '%s'. Holding last ros2_control position.",
        info_.joints[joint_index].name.c_str());
      continue;
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
      set_position_command(joint_index, hold_rad);
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
      continue;
    }

    // 未真正进入 OperationEnabled 前，持续锁当前真实位置，不接受控制器默认 0。
    if (!enable_status_ok || !command_enable || !state.operation_enabled ||
      !std::isfinite(command_cache_[joint_index]))
    {
      set_position_command(joint_index, hold_rad);
      driver_->setAbsoluteTargetCounts(motor.address, state.actual_counts);

      RCLCPP_WARN_THROTTLE(
        logger(), throttle_clock, 1000,
        "Joint '%s' is not command-enabled yet: enable_status_ok=%d command_enable=%d "
        "operation_enabled=%d status_word=0x%04X. Holding current position.",
        info_.joints[joint_index].name.c_str(),
        enable_status_ok,
        command_enable,
        state.operation_enabled,
        state.status_word);
      continue;
    }

    const int32_t target_counts =
      radians_to_counts(command_cache_[joint_index], motor, joint_index);
    if (!driver_->setAbsoluteTargetCounts(motor.address, target_counts)) {
      set_position_command(joint_index, hold_rad);
      driver_->setAbsoluteTargetCounts(motor.address, state.actual_counts);

      RCLCPP_WARN_THROTTLE(
        logger(), throttle_clock, 1000,
        "setAbsoluteTargetCounts failed for joint '%s'. Holding current position.",
        info_.joints[joint_index].name.c_str());
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace es_motor_hardware

PLUGINLIB_EXPORT_CLASS(es_motor_hardware::EsMotorHardwareInterface, hardware_interface::SystemInterface)
