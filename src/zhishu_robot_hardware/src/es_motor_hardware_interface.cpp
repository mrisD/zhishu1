#include "zhishu_robot_hardware/es_motor_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace es_motor_hardware
{

hardware_interface::CallbackReturn EsMotorHardwareInterface::on_init
    (const hardware_interface::HardwareInfo& info)
{
    if(hardware_interface::SystemInterface::on_init(info) != 
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    info_ = info;
    driver_ = std::make_shared<ethercat_driver::EthercatDriver>();

    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EsMotorHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
     // EthercatDriver::init_master() now requires a DriverConfig argument and
    // returns bool.  Keep this configuration aligned with config/driver.yaml.
    // ethercat_driver::DriverConfig driver_config{};
    driver_config.cycle_ns = 1000000;
    driver_config.enable_dc = true;
    driver_config.assign_activate = 0x0300;
    driver_config.sync0_shift_ns = 622620;
    driver_config.select_dc_reference_clock = true;
    driver_config.dc_reference_alias = 0;
    driver_config.dc_reference_slave_position = 0;
    driver_config.enable_realtime = true;
    driver_config.realtime_priority = 80;

    ethercat_driver::MotorConfig motor_config{};
    motor_config.name = "drive3";
    motor_config.joint_name = "joint3";
    motor_config.address.master_index = 0;
    motor_config.address.alias = 0;
    motor_config.address.slave_position = 2;
    motor_config.address.vendor_id = 0x00206688;
    motor_config.address.product_code = 0x26483052;
    motor_config.configure_sdo = true;
    motor_config.configure_pdos = true;
    motor_config.mode_of_operation = 8;
    motor_config.auto_enable = true;
    motor_config.auto_fault_reset = true;
    motor_config.max_velocity_counts_s = 8000.0;
    motor_config.max_accel_counts_s2 = 20000.0;
    motor_config.target_tolerance_counts = 20;
    motor_config.counts_per_rev = 131072.0;
    driver_config.motors.push_back(motor_config);

    // if (driver_->init_master(driver_config) != true)
    // {
    //     RCLCPP_ERROR(rclcpp::get_logger("EsMotorHardwareInterface"),
    //                  "Failed to initialize EtherCAT master.");
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

    // RCLCPP_INFO(rclcpp::get_logger("EsMotorHardwareInterface"),
    //             "EtherCAT master initialized successfully.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EsMotorHardwareInterface::on_activate
    (const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    // 防止启动瞬间命令跳变
    for (size_t i = 0; i < hw_commands_.size(); ++i) {
        hw_commands_[i] = hw_positions_[i];
    }

    if (!driver_->start(driver_config)) {
        RCLCPP_ERROR(
            rclcpp::get_logger("EsMotorHardwareInterface"),
            "Failed to start EtherCAT driver.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("EsMotorHardwareInterface"),
        "EtherCAT driver started.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EsMotorHardwareInterface::on_deactivate
(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    if (driver_) {
        driver_->stop();
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EsMotorHardwareInterface::
    export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(info_.joints.size());
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        for (const auto & interface : info_.joints[i].state_interfaces) {
            if (interface.name == hardware_interface::HW_IF_POSITION) {
                state_interfaces.emplace_back(
                    info_.joints[i].name,
                    hardware_interface::HW_IF_POSITION,
                    &hw_positions_[i]);
            }
        }
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EsMotorHardwareInterface::
    export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(info_.joints.size());
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        for (const auto & interface : info_.joints[i].command_interfaces) {
            if (interface.name == hardware_interface::HW_IF_POSITION) {
                command_interfaces.emplace_back(
                    info_.joints[i].name,
                    hardware_interface::HW_IF_POSITION,
                    &hw_commands_[i]);
            }
        }
    }
    return command_interfaces;
}

hardware_interface::return_type EsMotorHardwareInterface::read
    (const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    if (!driver_config.motors.empty()) {
        ethercat_driver::MotorState state{};
        const auto & motor = driver_config.motors[0];

        if (driver_->getMotorState(motor.address, state)) {
            // counts -> rad
            hw_positions_[0] =
                static_cast<double>(state.actual_counts) /
                motor.counts_per_rev * 2.0 * M_PI;
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type EsMotorHardwareInterface::write
    (const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    if (!driver_config.motors.empty()) {
        const auto & motor = driver_config.motors[0];

        if (!driver_->setTargetRadians(motor.address, hw_commands_[0])) {
            RCLCPP_WARN(
                rclcpp::get_logger("EsMotorHardwareInterface"),
                "setTargetRadians failed.");
        }
    }
    return hardware_interface::return_type::OK;

    // (void)time;
    // (void)period;
    // driver_->setTargetDegrees(driver_config.motors[0].address, hw_commands_[0] * 180.0 / M_PI);
    // return hardware_interface::return_type::OK;
}



} // namespace es_motor_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(es_motor_hardware::EsMotorHardwareInterface, hardware_interface::SystemInterface)
