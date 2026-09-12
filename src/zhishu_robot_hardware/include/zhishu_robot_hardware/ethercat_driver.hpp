#pragma once

#include <cstddef>
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

  bool auto_enable = false;

  double max_velocity_counts_s = 8000.0;
  double max_accel_counts_s2 = 20000.0;
  int64_t target_tolerance_counts = 20;
  double counts_per_rev = 131072.0;

  // Configured joint-side conversions are retained alongside the raw PDO
  // values so diagnostic CSV files are directly usable while preserving the
  // original drive counts as the source of truth.
  double velocity_scale_rad_s_per_raw = 0.0;
  double effort_scale_nm_per_raw = 0.0;
  double position_scale_rad_per_count = 0.0;
  double position_offset_rad = 0.0;
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
  int realtime_cpu = -1;

  // In CiA-402 CSP mode, forward each ros2_control target directly to the PDO.
  // Keep this opt-in so the legacy software trapezoidal profile remains available
  // for non-CSP deployments and rollback.
  bool csp_direct_position_commands = false;

  // Keep the firmware-accepted TxPDO index 0x1A01, but remap it to include
  // the ESI-defined 0x606C velocity and 0x6077 torque feedback entries.
  // This avoids selecting the 13-byte 0x1A00 map that was rejected by the
  // tested GH1 firmware with AL 0x0017.
  bool enable_velocity_torque_feedback = false;

  // Number of complete EtherCAT cycles retained while the non-realtime CSV
  // writer catches up. Recording is started and stopped explicitly through
  // EthercatDriver; the cyclic thread never performs file I/O.
  size_t pdo_diagnostics_buffer_cycles = 16384;

  // Optionally start a bounded PDO capture as soon as EtherCAT initialization
  // succeeds. This is also a fallback when the controller-manager service
  // executor is unavailable. Zero max cycles means no automatic stop.
  bool pdo_diagnostics_auto_start = false;
  uint64_t pdo_diagnostics_max_cycles = 0;

  std::vector<MotorConfig> motors;
};

struct MotorState
{
  bool ethercat_ok = false;
  bool slave_online = false;
  bool slave_operational = false;
  bool operation_enabled = false;
  bool target_reached = false;
  bool fault = false;
  bool fault_reaction_active = false;
  bool quick_stop_active = false;

  uint16_t status_word = 0;
  uint16_t error_code = 0;
  bool error_code_valid = false;
  int cia402_state = 0;
  int slave_al_state = 0;
  int domain_wc = 0;
  int domain_wc_state = 0;

  int32_t actual_counts = 0;
  // Raw CiA-402 values from the ESI-defined feedback objects. Unit conversion
  // belongs in the ros2_control adapter because the ESI does not define the
  // physical scaling for these objects.
  int32_t actual_velocity_raw = 0;  // 0x606C:00, DINT
  int16_t actual_torque_raw = 0;    // 0x6077:00, INT
  int32_t command_counts = 0;
  int64_t goal_counts = 0;
  uint64_t goal_update_sequence = 0;
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
  // Keep diagnostic joint-position conversion on the same learned 2*pi branch
  // as ros2_control. This does not alter commands sent to the drive.
  bool setDiagnosticPositionOffsetRad(const MotorAddress & address, double offset_rad);
  bool setRelativeTargetCounts(const MotorAddress & address, int32_t delta_counts);
  bool setTargetDegrees(const MotorAddress & address, double degrees);
  bool setRelativeDegrees(const MotorAddress & address, double delta_degrees);
  bool setTargetRadians(const MotorAddress & address, double radians);
  bool setRelativeRadians(const MotorAddress & address, double delta_radians);

  bool setEnable(const MotorAddress & address, bool enable);
  bool getEnabledStatus(const MotorAddress & address, bool & enable) const;
  bool requestFaultReset(const MotorAddress & address);
  bool requestQuickStopRelease(const MotorAddress & address);

  bool getMotorState(const MotorAddress & address, MotorState & state) const;
  std::vector<MotorState> getAllMotorStates() const;

  // Capture the exact cyclic PDO process image without doing any file I/O in
  // the realtime EtherCAT thread. The returned message contains the CSV path
  // on start and sample/drop statistics on stop.
  bool startPdoRecording(std::string & message);
  bool stopPdoRecording(std::string & message);
  bool isPdoRecording() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  
};

}  // namespace ethercat_driver
