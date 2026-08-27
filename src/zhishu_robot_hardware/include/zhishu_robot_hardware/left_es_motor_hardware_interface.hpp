#ifndef LEFT_ES_MOTOR_HARDWARE_INTERFACE_HPP
#define LEFT_ES_MOTOR_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "zhishu_robot_hardware/ethercat_driver.hpp"
namespace left_es_motor_hardware
{

class LeftEsMotorHardwareInterface : public hardware_interface::SystemInterface
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
    hardware_interface::CallbackReturn
        on_error(const rclcpp_lifecycle::State& previous_state) override;

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
    bool reset_faults_manually(std::string & message);
    bool set_motors_enabled_manually(bool enable, std::string & message);
    void latch_hardware_fault(const std::string & reason);

    double counts_to_radians(int32_t counts, const ethercat_driver::MotorConfig & motor, size_t joint_index);
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

    // A battery-less absolute encoder can restart on a different, but equivalent,
    // 2*pi branch. The learned turn offset must be applied in both conversion
    // directions so a normalized ROS command never asks the motor to move a full turn.
    std::vector<bool> joint_position_wraparound_;
    std::vector<double> joint_position_wrap_reference_rad_;
    std::vector<double> joint_turn_offset_rad_;
    std::vector<bool> joint_turn_offset_initialized_;

    // Optional reporting-side limit snap. This only clamps tiny encoder/zero-point
    // excursions that are just outside a configured physical joint boundary.
    // It does not clamp normal in-range motion or large true limit violations.
    std::vector<double> joint_snap_lower_limit_rad_;
    std::vector<double> joint_snap_upper_limit_rad_;
    double joint_limit_snap_tolerance_rad_ = 0.005;

    bool enable_motors_on_activate_ = true;
    std::atomic<bool> activated_{false};
    // Set before returning ERROR from read()/write(). controller_manager then
    // aborts every controller using this hardware. The EtherCAT worker remains
    // alive so the manual reset service can still inspect and clear drive faults.
    std::atomic<bool> hardware_fault_latched_{false};
    std::vector<bool> safety_stop_reported_;
    std::vector<bool> last_fault_error_code_valid_;
    std::vector<uint16_t> last_fault_error_code_;

    // Runtime fault recovery is intentionally two-step: reset first, then explicit enable.
    // The flag is checked by write() so ros2_control cannot resume motion between those steps.
    std::atomic<bool> manual_motion_enabled_{false};
    std::mutex recovery_mutex_;
    rclcpp::Node::SharedPtr recovery_node_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;


}; // class LeftEsMotorHardwareInterface

} // namespace left_es_motor_hardware

#endif // LEFT_ES_MOTOR_HARDWARE_INTERFACE_HPP
