#ifndef ES_MOTOR_HARDWARE_INTERFACE_HPP
#define ES_MOTOR_HARDWARE_INTERFACE_HPP

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
    bool set_velocity_state(size_t joint_index, double value);
    bool set_effort_state(size_t joint_index, double value);
    bool set_velocity_raw_state(size_t joint_index, double value);
    bool set_torque_raw_state(size_t joint_index, double value);
    bool set_position_command(size_t joint_index, double value);
    bool refresh_position_command(size_t joint_index);
 
    static constexpr int kNoMotor = -1;
  
    std::shared_ptr<ethercat_driver::EthercatDriver> driver_;
    ethercat_driver::DriverConfig driver_config_{};
    
    // Local safety caches are intentionally separate from the interface storage.
    // Jazzy owns the exported handles and the real-time loop accesses them through
    // the non-blocking handle API.
    std::vector<double> position_cache_;
    std::vector<double> velocity_cache_;
    std::vector<double> effort_cache_;
    std::vector<double> velocity_raw_cache_;
    std::vector<double> torque_raw_cache_;
    std::vector<double> command_cache_;
    std::vector<hardware_interface::StateInterface::SharedPtr> position_state_handles_;
    std::vector<hardware_interface::StateInterface::SharedPtr> velocity_state_handles_;
    std::vector<hardware_interface::StateInterface::SharedPtr> effort_state_handles_;
    std::vector<hardware_interface::StateInterface::SharedPtr> velocity_raw_state_handles_;
    std::vector<hardware_interface::StateInterface::SharedPtr> torque_raw_state_handles_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> position_command_handles_;
    
    // joint_to_motor_index_[joint_index] == driver_config_.motors[motor_index].
    // Non EtherCAT joints stay at their initial/state value and are not commanded.
    std::vector<int> joint_to_motor_index_;
    std::vector<double> joint_sign_;
    std::vector<double> joint_offset_rad_;
    std::vector<double> joint_reduction_ratio_;
    std::vector<double> joint_rated_torque_nm_;
    std::vector<double> joint_velocity_scale_rad_s_per_raw_;
    std::vector<double> joint_effort_scale_nm_per_raw_;
    double velocity_raw_rpm_per_count_ = 0.1;
    double torque_raw_full_scale_ = 10000.0;

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
    // Service namespace is configurable so the generic EtherCAT plugin can be
    // reused by independent hardware components (right arm, torso/head, ...).
    // Defaults preserve the existing /right_arm/* API.
    std::string service_namespace_{"right_arm"};
    std::string reset_fault_service_name_{"/right_arm/reset_fault"};
    std::string enable_service_name_{"/right_arm/enable"};
    std::string pdo_record_service_name_{"/right_arm/record_pdo"};
    rclcpp::Node::SharedPtr recovery_node_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pdo_record_service_;


}; // class EsMotorHardwareInterface 

} // namespace es_motor_hardware

#endif // ES_MOTOR_HARDWARE_INTERFACE_HPP
