#ifndef ES_MOTOR_HARDWARE_INTERFACE_HPP
#define ES_MOTOR_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include "zhishu_robot_hardware/ethercat_driver.hpp"
#include <rclcpp/rclcpp.hpp>

namespace es_motor_hardware
{
class EsMotorHardwareInterface : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn 
        on_init(const hardware_interface::HardwareInfo& info) override;
    hardware_interface::CallbackReturn 
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn 
        on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn 
        on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    
    std::vector<hardware_interface::StateInterface> 
        export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> 
        export_command_interfaces() override;
    hardware_interface::return_type 
        read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type 
        write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    std::shared_ptr<ethercat_driver::EthercatDriver> driver_;
    int motor_id;

    std::vector<double> hw_positions_;
    std::vector<double> hw_commands_;

    ethercat_driver::DriverConfig driver_config{};




}; // class EsMotorHardwareInterface 

} // namespace es_motor_hardware

#endif // ES_MOTOR_HARDWARE_INTERFACE_HPP