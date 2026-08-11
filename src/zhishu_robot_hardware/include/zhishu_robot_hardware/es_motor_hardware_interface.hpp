#ifndef ES_MOTOR_HARDWARE_INTERFACE_HPP
#define ES_MOTOR_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

#include "zhishu_robot_hardware/ethercat_driver.hpp"

namespace es_motor_hardware
{

class EsMotorHardwareInterface : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn 
        on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
    hardware_interface::CallbackReturn 
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn 
        on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn 
        on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    
    std::vector<hardware_interface::StateInterface::ConstSharedPtr>
        on_export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface::SharedPtr>
        on_export_command_interfaces() override;
    hardware_interface::return_type 
        read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type 
        write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    bool build_driver_config_from_ros2_control_info();
    bool wait_for_all_motors_ready(std::vector<ethercat_driver::MotorState> & states) const;

    double counts_to_radians(int32_t counts, const ethercat_driver::MotorConfig & motor, size_t joint_index) const;
    int32_t radians_to_counts(double radians, const ethercat_driver::MotorConfig & motor, size_t joint_index) const;
    bool set_position_state(size_t joint_index, double value);
    bool set_position_command(size_t joint_index, double value);
    bool refresh_position_command(size_t joint_index);
 
    static constexpr int kNoMotor = -1;
  
    std::shared_ptr<ethercat_driver::EthercatDriver> driver_;
    ethercat_driver::DriverConfig driver_config_{};
    
    // Local safety caches are intentionally separate from the interface storage.
    // Jazzy owns the exported handles and the real-time loop accesses them through
    // the non-blocking handle API.
    std::vector<double> position_cache_;
    std::vector<double> command_cache_;
    std::vector<hardware_interface::StateInterface::SharedPtr> position_state_handles_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> position_command_handles_;
    
    // joint_to_motor_index_[joint_index] == driver_config_.motors[motor_index].
    // Non EtherCAT joints stay at their initial/state value and are not commanded.
    std::vector<int> joint_to_motor_index_;
    std::vector<double> joint_sign_;
    std::vector<double> joint_offset_rad_;

    // Optional reporting-side limit snap. This only clamps tiny encoder/zero-point
    // excursions that are just outside a configured physical joint boundary.
    // It does not clamp normal in-range motion or large true limit violations.
    std::vector<double> joint_snap_lower_limit_rad_;
    std::vector<double> joint_snap_upper_limit_rad_;
    double joint_limit_snap_tolerance_rad_ = 0.005;

    bool enable_motors_on_activate_ = true;
    bool activated_ = false;


}; // class EsMotorHardwareInterface 

} // namespace es_motor_hardware

#endif // ES_MOTOR_HARDWARE_INTERFACE_HPP
