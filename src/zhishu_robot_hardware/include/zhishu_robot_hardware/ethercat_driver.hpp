#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ethercat_driver
{

struct MotorAddress
{
  unsigned int master_index = 0;
  uint16_t alias = 0;
  uint16_t slave_position = 0;
  uint32_t vendor_id = 0;
  uint32_t product_code = 0;
};

struct MotorAddressLess
{
  bool operator()(const MotorAddress & lhs, const MotorAddress & rhs) const;
};

bool operator==(const MotorAddress & lhs, const MotorAddress & rhs);
std::string toString(const MotorAddress & address);

struct MotorConfig
{
  std::string name = "motor";
  std::string joint_name = "joint";
  MotorAddress address{};

  bool configure_sdo = true;
  bool configure_pdos = true;
  int8_t mode_of_operation = 8;  // CiA402 CSP

  bool auto_enable = true;
  bool auto_fault_reset = true;

  double max_velocity_counts_s = 8000.0;
  double max_accel_counts_s2 = 20000.0;
  int64_t target_tolerance_counts = 20;
  double counts_per_rev = 131072.0;
};

struct DriverConfig
{
  int64_t cycle_ns = 1000000;

  bool enable_dc = true;
  uint16_t assign_activate = 0x0300;
  int64_t sync0_shift_ns = 622620;
  bool select_dc_reference_clock = true;
  uint16_t dc_reference_alias = 0;
  uint16_t dc_reference_slave_position = 0;

  bool enable_realtime = true;
  int realtime_priority = 80;

  std::vector<MotorConfig> motors;
};

struct MotorState
{
  bool ethercat_ok = false;
  bool slave_online = false;
  bool slave_operational = false;
  bool operation_enabled = false;
  bool target_reached = false;

  uint16_t status_word = 0;
  int cia402_state = 0;
  int slave_al_state = 0;
  int domain_wc = 0;
  int domain_wc_state = 0;

  int32_t actual_counts = 0;
  int32_t command_counts = 0;
  int64_t goal_counts = 0;
};

class EthercatDriver
{
public:
  EthercatDriver();
  ~EthercatDriver();

  EthercatDriver(const EthercatDriver &) = delete;
  EthercatDriver & operator=(const EthercatDriver &) = delete;

  // High-level API used by the current test node/plugin.
  // start() keeps the original behavior: it starts the realtime EtherCAT loop thread.
  bool start(const DriverConfig & config);
  void stop();

  // Split EtherCAT lifecycle API.
  // These functions expose the internal startup sequence explicitly, which is useful
  // when the driver is reused by a ros2_control hardware plugin.
  bool init_master(const DriverConfig & config);
  bool configure_slave();
  bool configure_pdo();
  bool activate_master();
  void shutdown();

  bool isRunning() const;
  bool isEthercatOk() const;

  bool setAbsoluteTargetCounts(const MotorAddress & address, int32_t counts);
  bool setRelativeTargetCounts(const MotorAddress & address, int32_t delta_counts);
  bool setTargetDegrees(const MotorAddress & address, double degrees);
  bool setRelativeDegrees(const MotorAddress & address, double delta_degrees);
  bool setTargetRadians(const MotorAddress & address, double radians);
  bool setRelativeRadians(const MotorAddress & address, double delta_radians);

  bool setEnable(const MotorAddress & address, bool enable);
  bool requestFaultReset(const MotorAddress & address);

  bool getMotorState(const MotorAddress & address, MotorState & state) const;
  std::vector<MotorState> getAllMotorStates() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  
};

}  // namespace ethercat_driver
