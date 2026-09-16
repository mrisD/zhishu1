#include "zhishu_robot_hardware/ethercat_driver.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <cstring>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <tuple>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

extern "C" {
#include <ecrt.h>
}

namespace ethercat_driver
{
namespace
{
constexpr uint16_t kPdoControlWord = 0x6040;
constexpr uint16_t kPdoTargetPosition = 0x607A;
constexpr uint16_t kPdoStatusWord = 0x6041;
constexpr uint16_t kPdoActualPosition = 0x6064;
constexpr uint16_t kPdoActualVelocity = 0x606C;
constexpr uint16_t kPdoActualTorque = 0x6077;
constexpr uint16_t kPdoActualCurrent = 0x6078;
constexpr uint16_t kPdoFollowingErrorActual = 0x60F4;
constexpr size_t kMaxPdoDiagnosticMotors = 16;

struct PdoOffsets
{
  unsigned int control_word = 0;
  unsigned int target_position = 0;
  unsigned int status_word = 0;
  unsigned int actual_position = 0;
  unsigned int actual_velocity = 0;
  unsigned int actual_torque = 0;
  unsigned int actual_current = 0;
  unsigned int following_error_actual = 0;
};

struct PdoDiagnosticMotorSample
{
  int32_t target_position = 0;
  int32_t actual_position = 0;
  int32_t position_error = 0;
  int32_t following_error_actual = 0;
  double sent_target_position_joint_rad = 0.0;
  double feedback_actual_position_joint_rad = 0.0;
  double position_error_joint_rad = 0.0;
  double position_conversion_offset_rad = 0.0;
  int32_t target_delta = 0;
  int32_t actual_delta = 0;
  int32_t actual_velocity = 0;
  int16_t actual_torque = 0;
  int16_t actual_current = 0;
  uint16_t control_word = 0;
  uint16_t status_word = 0;
  uint16_t error_code = 0;
  bool error_code_valid = false;
  uint64_t goal_update_sequence = 0;
  uint32_t goal_sequence_delta = 0;
};

struct PdoDiagnosticRecord
{
  int64_t monotonic_ns = 0;
  int64_t scheduled_cycle_start_ns = 0;
  int64_t cycle_period_ns = 0;
  int64_t period_jitter_ns = 0;
  int64_t start_lateness_ns = 0;
  int64_t wake_lateness_ns = 0;
  uint64_t execution_ns = 0;
  uint64_t deadline_miss_total = 0;
  uint64_t dropped_cycles_total = 0;
  uint64_t cycle = 0;
  uint32_t domain_wc = 0;
  uint8_t domain_wc_state = 0;
  uint8_t motor_count = 0;
  std::array<PdoDiagnosticMotorSample, kMaxPdoDiagnosticMotors> motors{};
};

enum class Cia402State
{
  NotReadyToSwitchOn,
  SwitchOnDisabled,
  ReadyToSwitchOn,
  SwitchedOn,
  OperationEnabled,
  QuickStopActive,
  FaultReactionActive,
  Fault,
  Unknown
};

Cia402State decodeStatusWord(uint16_t sw)
{
  if ((sw & 0x004F) == 0x0000) {
    return Cia402State::NotReadyToSwitchOn;
  }
  if ((sw & 0x004F) == 0x0040) {
    return Cia402State::SwitchOnDisabled;
  }
  if ((sw & 0x006F) == 0x0021) {
    return Cia402State::ReadyToSwitchOn;
  }
  if ((sw & 0x006F) == 0x0023) {
    return Cia402State::SwitchedOn;
  }
  if ((sw & 0x006F) == 0x0027) {
    return Cia402State::OperationEnabled;
  }
  if ((sw & 0x006F) == 0x0007) {
    return Cia402State::QuickStopActive;
  }
  if ((sw & 0x004F) == 0x000F) {
    return Cia402State::FaultReactionActive;
  }
  if ((sw & 0x004F) == 0x0008) {
    return Cia402State::Fault;
  }
  return Cia402State::Unknown;
}

uint64_t timespecToNs(const timespec & ts)
{
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

void addNs(timespec & ts, int64_t ns)
{
  ts.tv_nsec += ns;
  while (ts.tv_nsec >= 1000000000L) {
    ts.tv_nsec -= 1000000000L;
    ts.tv_sec += 1;
  }
  while (ts.tv_nsec < 0) {
    ts.tv_nsec += 1000000000L;
    ts.tv_sec -= 1;
  }
}

int32_t clampToInt32(double value)
{
  if (value > static_cast<double>(INT32_MAX)) {
    return INT32_MAX;
  }
  if (value < static_cast<double>(INT32_MIN)) {
    return INT32_MIN;
  }
  return static_cast<int32_t>(std::llround(value));
}

int64_t monotonicNs()
{
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(timespecToNs(ts));
}

struct MotorContext
{
  explicit MotorContext(const MotorConfig & cfg)
  : config(cfg), command_enable(cfg.auto_enable),
    diagnostic_position_offset_rad(cfg.position_offset_rad)
  {
  }

  MotorConfig config;
  ec_slave_config_t * sc = nullptr;
  ec_sdo_request_t * error_code_request = nullptr;
  PdoOffsets offsets{};

  std::atomic<bool> command_enable{true};
  std::atomic<bool> fault_reset_requested{false};
  std::atomic<bool> quick_stop_release_requested{false};
  std::atomic<bool> operation_enabled{false};
  std::atomic<bool> target_reached{true};
  std::atomic<bool> have_goal{false};
  std::atomic<bool> slave_online{false};
  std::atomic<bool> slave_operational{false};

  std::atomic<int32_t> actual_counts{0};
  std::atomic<int32_t> actual_velocity_raw{0};
  std::atomic<int16_t> actual_torque_raw{0};
  std::atomic<int32_t> command_counts{0};
  std::atomic<uint16_t> statusword{0};
  std::atomic<uint16_t> error_code{0};
  std::atomic<bool> error_code_valid{false};
  std::atomic<int> cia402_state{static_cast<int>(Cia402State::Unknown)};
  std::atomic<int64_t> goal_counts{0};
  std::atomic<double> diagnostic_position_offset_rad{0.0};
  // High 32 bits: update sequence; low 32 bits: signed target counts.
  // Publishing both in one atomic prevents the EtherCAT thread from pairing a
  // new target with an old sequence (or vice versa).
  std::atomic<uint64_t> goal_command_snapshot{0};
  std::atomic<int> slave_al_state{0};

  double cmd_position = 0.0;
  double profile_velocity = 0.0;
  int fault_reset_counter = 0;
  bool first_cycle = true;
  bool hold_latched_after_enable = false;
  bool error_code_request_pending = false;
  uint64_t next_error_code_cycle = 0;

  // Updated only by the EtherCAT cyclic thread and used to annotate exact
  // consecutive-cycle command/feedback changes in the diagnostic CSV.
  bool previous_pdo_sample_valid = false;
  int32_t previous_pdo_target = 0;
  int32_t previous_pdo_actual = 0;
  uint32_t previous_pdo_goal_sequence = 0;

  uint8_t last_slave_al_state = 0xFF;
  bool last_slave_online = false;
  bool last_slave_operational = false;
};

}  // namespace

bool operator==(const MotorAddress & lhs, const MotorAddress & rhs)
{
  return lhs.master_index == rhs.master_index &&
         lhs.alias == rhs.alias &&
         lhs.slave_position == rhs.slave_position &&
         lhs.vendor_id == rhs.vendor_id &&
         lhs.product_code == rhs.product_code;
}

bool MotorAddressLess::operator()(const MotorAddress & lhs, const MotorAddress & rhs) const
{
  return std::tie(lhs.master_index, lhs.alias, lhs.slave_position, lhs.vendor_id, lhs.product_code) <
         std::tie(rhs.master_index, rhs.alias, rhs.slave_position, rhs.vendor_id, rhs.product_code);
}

std::string toString(const MotorAddress & address)
{
  char buffer[160];
  std::snprintf(
    buffer, sizeof(buffer),
    "master=%u alias=%u position=%u vendor=0x%08X product=0x%08X",
    address.master_index, address.alias, address.slave_position,
    address.vendor_id, address.product_code);
  return std::string(buffer);
}

class EthercatDriver::Impl
{
public:
  ~Impl()
  {
    stop();
  }

  bool start(const DriverConfig & config)
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (running_.load()) {
      std::fprintf(stderr, "ethercat_driver: driver is already running\n");
      return false;
    }
    if (master_ != nullptr) {
      std::fprintf(stderr, "ethercat_driver: master is already initialized; call shutdown() before start()\n");
      return false;
    }

    if (!prepareConfigNoLock(config)) {
      return false;
    }

    running_.store(true);
    worker_ = std::thread([this]() { loop(); });
    return true;
  }

  void stop()
  {
    shutdown();
  }

  bool init_master(const DriverConfig & config)
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (running_.load()) {
      std::fprintf(stderr, "ethercat_driver: cannot call init_master() while start() loop is running\n");
      return false;
    }

    shutdownNoLock();
    if (!prepareConfigNoLock(config)) {
      return false;
    }
    return initMasterNoLock();
  }

  bool configure_slave()
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (running_.load()) {
      std::fprintf(stderr, "ethercat_driver: cannot call configure_slave() while start() loop is running\n");
      return false;
    }
    return configureSlaveNoLock();
  }

  bool configure_pdo()
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (running_.load()) {
      std::fprintf(stderr, "ethercat_driver: cannot call configure_pdo() while start() loop is running\n");
      return false;
    }
    return configurePdoNoLock();
  }

  bool activate_master()
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (running_.load()) {
      std::fprintf(stderr, "ethercat_driver: cannot call activate_master() while start() loop is running\n");
      return false;
    }
    return activateMasterNoLock();
  }

  void shutdown()
  {
    {
      std::lock_guard<std::mutex> diagnostics_lock(pdo_diagnostics_mutex_);
      std::string ignored_message;
      stopPdoRecordingNoLock(ignored_message);
    }
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
    shutdownNoLock();
  }

  bool isRunning() const
  {
    return running_.load();
  }

  bool isEthercatOk() const
  {
    return ethercat_ok_.load();
  }

  bool setAbsoluteTargetCounts(const MotorAddress & address, int32_t counts)
  {
    auto motor = findMotor(address);
    if (!motor) {
      return false;
    }
    setGoalCountsClamped(*motor, static_cast<int64_t>(counts));
    return true;
  }

  bool setDiagnosticPositionOffsetRad(const MotorAddress & address, double offset_rad)
  {
    auto motor = findMotor(address);
    if (!motor || !std::isfinite(offset_rad)) {
      return false;
    }
    motor->diagnostic_position_offset_rad.store(offset_rad, std::memory_order_release);
    return true;
  }

  bool setRelativeTargetCounts(const MotorAddress & address, int32_t delta_counts)
  {
    auto motor = findMotor(address);
    if (!motor) {
      return false;
    }
    const int64_t target = static_cast<int64_t>(motor->actual_counts.load()) + static_cast<int64_t>(delta_counts);
    setGoalCountsClamped(*motor, target);
    return true;
  }

  bool setTargetDegrees(const MotorAddress & address, double degrees)
  {
    auto motor = findMotor(address);
    if (!motor) {
      return false;
    }
    const int64_t counts = static_cast<int64_t>(std::llround(degrees / 360.0 * motor->config.counts_per_rev));
    setGoalCountsClamped(*motor, counts);
    return true;
  }

  bool setRelativeDegrees(const MotorAddress & address, double delta_degrees)
  {
    auto motor = findMotor(address);
    if (!motor) {
      return false;
    }
    const int64_t delta = static_cast<int64_t>(std::llround(delta_degrees / 360.0 * motor->config.counts_per_rev));
    const int64_t target = static_cast<int64_t>(motor->actual_counts.load()) + delta;
    setGoalCountsClamped(*motor, target);
    return true;
  }

  bool setTargetRadians(const MotorAddress & address, double radians)
  {
    return setTargetDegrees(address, radians * 180.0 / M_PI);
  }

  bool setRelativeRadians(const MotorAddress & address, double delta_radians)
  {
    return setRelativeDegrees(address, delta_radians * 180.0 / M_PI);
  }

  bool setEnable(const MotorAddress & address, bool enable)
  {
    auto motor = findMotor(address);
    if (!motor) {
      return false;
    }
    motor->command_enable.store(enable);
    return true;
  }

  bool getEnabledStatus(const MotorAddress & address, bool & enable) const
  {
    auto motor = findMotor(address);
    if (!motor) {
      return false;
    }
    enable = motor->command_enable.load();
    return true;
  }

  bool requestFaultReset(const MotorAddress & address)
  {
    auto motor = findMotor(address);
    if (!motor) {
      return false;
    }
    motor->fault_reset_requested.store(true);
    return true;
  }

  bool requestQuickStopRelease(const MotorAddress & address)
  {
    auto motor = findMotor(address);
    if (!motor) {
      return false;
    }
    motor->quick_stop_release_requested.store(true);
    return true;
  }

  bool getMotorState(const MotorAddress & address, MotorState & state) const
  {
    auto motor = findMotor(address);
    if (!motor) {
      return false;
    }
    fillMotorState(*motor, state);
    return true;
  }

  std::vector<MotorState> getAllMotorStates() const
  {
    std::vector<MotorState> states;
    states.reserve(motors_.size());
    for (const auto & motor : motors_) {
      MotorState state{};
      fillMotorState(*motor, state);
      states.push_back(state);
    }
    return states;
  }

  bool startPdoRecording(std::string & message)
  {
    std::lock_guard<std::mutex> lock(pdo_diagnostics_mutex_);
    if (!running_.load() || domain_pd_ == nullptr) {
      message = "EtherCAT is not running; PDO recording is unavailable";
      return false;
    }
    if (pdo_recording_enabled_.load()) {
      message = "PDO recording is already active: " + pdo_diagnostics_path_;
      return false;
    }
    // A bounded automatic capture can finish without a service callback. Reap
    // its writer before allowing a later manual capture to replace the thread.
    if (pdo_diagnostics_writer_.joinable()) {
      pdo_diagnostics_writer_.join();
    }
    if (pdo_diagnostics_file_ != nullptr) {
      std::fflush(pdo_diagnostics_file_);
      std::fclose(pdo_diagnostics_file_);
      pdo_diagnostics_file_ = nullptr;
    }
    if (motors_.size() > kMaxPdoDiagnosticMotors) {
      message = "too many motors for PDO diagnostic recorder";
      return false;
    }

    const size_t capacity = std::max<size_t>(
      config_.pdo_diagnostics_buffer_cycles, 1024U);
    bool write_csv_header = false;
    try {
      pdo_diagnostics_ring_.assign(capacity, PdoDiagnosticRecord{});

      if (pdo_diagnostics_path_.empty()) {
        std::filesystem::path output_directory;
        if (const char * ros_log_dir = std::getenv("ROS_LOG_DIR")) {
          output_directory = ros_log_dir;
          if (output_directory.filename() == "native") {
            output_directory = output_directory.parent_path();
          }
        } else {
          output_directory = std::filesystem::current_path();
        }
        std::filesystem::create_directories(output_directory);

        const auto now = std::chrono::system_clock::now();
        const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm local_time{};
        localtime_r(&now_time, &local_time);
        char timestamp_seconds[32]{};
        std::strftime(
          timestamp_seconds, sizeof(timestamp_seconds), "%Y%m%d_%H%M%S", &local_time);
        const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
          now.time_since_epoch()).count() % 1000000LL;
        char timestamp[48]{};
        std::snprintf(
          timestamp, sizeof(timestamp), "%s_%06lld", timestamp_seconds,
          static_cast<long long>(microseconds));
        const unsigned int master_index = config_.motors.front().address.master_index;
        output_directory /=
          "pdo_master" + std::to_string(master_index) + "_" + timestamp + ".csv";
        pdo_diagnostics_path_ = output_directory.string();
      }

      const std::filesystem::path output_path(pdo_diagnostics_path_);
      write_csv_header =
        !std::filesystem::exists(output_path) || std::filesystem::file_size(output_path) == 0U;
    } catch (const std::exception & error) {
      message = std::string("failed to prepare PDO recording: ") + error.what();
      return false;
    }

    // Keep one file per EtherCAT master for the lifetime of this driver. A later
    // service start resumes the same file instead of creating another CSV.
    pdo_diagnostics_file_ = std::fopen(
      pdo_diagnostics_path_.c_str(), write_csv_header ? "w" : "a");
    if (pdo_diagnostics_file_ == nullptr) {
      message = "failed to open PDO CSV '" + pdo_diagnostics_path_ + "': " +
        std::strerror(errno);
      return false;
    }

    if (write_csv_header) {
      std::fprintf(
        pdo_diagnostics_file_,
        "# Exact EtherCAT cyclic PDO capture using GH1 TxPDO 0x1A01.\n"
        "# With enable_velocity_torque_feedback=true, 0x1A01 is remapped to the ESI-defined "
        "0x6041/0x6064/0x606C/0x6077/0x6078/0x60F4 entries (18-byte SM3).\n"
        "# Values from 0x606C, 0x6077, 0x6078, and 0x60F4 are recorded raw; joint SI columns "
        "use the explicit ros2_control conversion parameters.\n"
        "# GH1 firmware validation found that 0x6078 TxPDO may duplicate the preceding mapped "
        "value and 0x60F4 may remain zero; use position_error_counts for following-error analysis.\n"
        "# The 1 kHz recording path is PDO-only; mailbox SDO reads are reserved for low-rate "
        "diagnostics.\n"
        "monotonic_ns,scheduled_cycle_start_ns,cycle_period_ns,period_jitter_ns,"
        "start_lateness_ns,wake_lateness_ns,execution_ns,deadline_miss_total,"
        "dropped_cycles_total,cycle,master_index,domain_wc,domain_wc_state,motor_name,"
        "slave_position,rxpdo_target_position_607a_counts,txpdo_position_actual_6064_counts,"
        "position_error_counts,txpdo_following_error_actual_60f4_counts,"
        "sent_target_position_joint_rad,feedback_actual_position_joint_rad,"
        "position_error_joint_rad,position_conversion_offset_rad,"
        "target_delta_counts,actual_delta_counts,txpdo_velocity_actual_606c_raw,"
        "txpdo_torque_actual_6077_raw,txpdo_current_actual_6078_raw,"
        "joint_velocity_rad_s,joint_effort_nm,rxpdo_control_word_6040,"
        "txpdo_status_word_6041,error_code_valid,"
        "error_code_603f,goal_update_sequence,goal_sequence_delta\n");
    } else {
      std::fprintf(
        pdo_diagnostics_file_,
        "# PDO recording resumed in the same driver session.\n");
    }
    std::fflush(pdo_diagnostics_file_);

    pdo_diagnostics_write_sequence_.store(0);
    pdo_diagnostics_read_sequence_.store(0);
    pdo_diagnostics_dropped_cycles_.store(0);
    pdo_diagnostics_producer_active_.store(0);
    pdo_recording_cycle_limit_.store(0, std::memory_order_relaxed);
    try {
      pdo_recording_enabled_.store(true, std::memory_order_release);
      pdo_diagnostics_writer_ = std::thread([this]() { pdoDiagnosticsWriterLoop(); });
    } catch (const std::exception & error) {
      pdo_recording_enabled_.store(false, std::memory_order_release);
      std::fclose(pdo_diagnostics_file_);
      pdo_diagnostics_file_ = nullptr;
      message = std::string("failed to start PDO CSV writer: ") + error.what();
      return false;
    }

    message = "PDO recording started: " + pdo_diagnostics_path_;
    return true;
  }

  bool stopPdoRecording(std::string & message)
  {
    std::lock_guard<std::mutex> lock(pdo_diagnostics_mutex_);
    return stopPdoRecordingNoLock(message);
  }

  bool isPdoRecording() const
  {
    return pdo_recording_enabled_.load(std::memory_order_acquire);
  }

private:
  bool stopPdoRecordingNoLock(std::string & message)
  {
    const bool was_recording = pdo_recording_enabled_.exchange(
      false, std::memory_order_acq_rel);
    if (!was_recording && !pdo_diagnostics_writer_.joinable()) {
      message = "PDO recording is not active";
      return false;
    }

    while (pdo_diagnostics_producer_active_.load(std::memory_order_acquire) != 0U) {
      std::this_thread::yield();
    }
    if (pdo_diagnostics_writer_.joinable()) {
      pdo_diagnostics_writer_.join();
    }
    if (pdo_diagnostics_file_ != nullptr) {
      std::fflush(pdo_diagnostics_file_);
      std::fclose(pdo_diagnostics_file_);
      pdo_diagnostics_file_ = nullptr;
    }

    const uint64_t written_cycles = pdo_diagnostics_read_sequence_.load();
    const uint64_t dropped_cycles = pdo_diagnostics_dropped_cycles_.load();
    message = "PDO recording stopped: " + pdo_diagnostics_path_ +
      " cycles=" + std::to_string(written_cycles) +
      " dropped=" + std::to_string(dropped_cycles);
    return true;
  }

  void pdoDiagnosticsWriterLoop()
  {
    const unsigned int master_index = config_.motors.front().address.master_index;
    int64_t last_flush_ns = monotonicNs();

    while (pdo_recording_enabled_.load(std::memory_order_acquire) ||
      pdo_diagnostics_read_sequence_.load(std::memory_order_relaxed) <
      pdo_diagnostics_write_sequence_.load(std::memory_order_acquire))
    {
      uint64_t read_sequence =
        pdo_diagnostics_read_sequence_.load(std::memory_order_relaxed);
      const uint64_t write_sequence =
        pdo_diagnostics_write_sequence_.load(std::memory_order_acquire);
      if (read_sequence == write_sequence) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }

      while (read_sequence < write_sequence) {
        const auto & record =
          pdo_diagnostics_ring_[read_sequence % pdo_diagnostics_ring_.size()];
        for (size_t i = 0; i < record.motor_count; ++i) {
          const auto & sample = record.motors[i];
          const auto & motor = motors_[i]->config;
          std::fprintf(
            pdo_diagnostics_file_,
            "%lld,%lld,%lld,%lld,%lld,%lld,%llu,%llu,%llu,%llu,%u,%u,%u,%s,%u,"
            "%d,%d,%d,%d,%.12g,%.12g,%.12g,%.12g,%d,%d,%d,%d,%d,%.12g,%.12g,"
            "0x%04X,0x%04X,%u,0x%04X,%llu,%u\n",
            static_cast<long long>(record.monotonic_ns),
            static_cast<long long>(record.scheduled_cycle_start_ns),
            static_cast<long long>(record.cycle_period_ns),
            static_cast<long long>(record.period_jitter_ns),
            static_cast<long long>(record.start_lateness_ns),
            static_cast<long long>(record.wake_lateness_ns),
            static_cast<unsigned long long>(record.execution_ns),
            static_cast<unsigned long long>(record.deadline_miss_total),
            static_cast<unsigned long long>(record.dropped_cycles_total),
            static_cast<unsigned long long>(record.cycle), master_index,
            record.domain_wc, static_cast<unsigned int>(record.domain_wc_state),
            motor.name.c_str(), motor.address.slave_position,
            static_cast<int>(sample.target_position),
            static_cast<int>(sample.actual_position),
            static_cast<int>(sample.position_error),
            static_cast<int>(sample.following_error_actual),
            sample.sent_target_position_joint_rad,
            sample.feedback_actual_position_joint_rad,
            sample.position_error_joint_rad,
            sample.position_conversion_offset_rad,
            static_cast<int>(sample.target_delta),
            static_cast<int>(sample.actual_delta),
            static_cast<int>(sample.actual_velocity),
            static_cast<int>(sample.actual_torque),
            static_cast<int>(sample.actual_current),
            static_cast<double>(sample.actual_velocity) *
            motor.velocity_scale_rad_s_per_raw,
            static_cast<double>(sample.actual_torque) *
            motor.effort_scale_nm_per_raw,
            static_cast<unsigned int>(sample.control_word),
            static_cast<unsigned int>(sample.status_word),
            sample.error_code_valid ? 1U : 0U,
            static_cast<unsigned int>(sample.error_code),
            static_cast<unsigned long long>(sample.goal_update_sequence),
            sample.goal_sequence_delta);
        }
        ++read_sequence;
        pdo_diagnostics_read_sequence_.store(read_sequence, std::memory_order_release);
      }

      const int64_t now_ns = monotonicNs();
      if (now_ns - last_flush_ns >= 1000000000LL) {
        std::fflush(pdo_diagnostics_file_);
        last_flush_ns = now_ns;
      }
    }
    if (pdo_diagnostics_file_ != nullptr) {
      std::fflush(pdo_diagnostics_file_);
    }
  }

  bool prepareConfigNoLock(const DriverConfig & config)
  {
    if (!validateConfig(config)) {
      return false;
    }

    config_ = config;
    motors_.clear();
    address_to_index_.clear();

    master_initialized_ = false;
    slaves_configured_ = false;
    pdo_configured_ = false;
    master_activated_ = false;
    ethercat_ok_.store(false);

    for (size_t i = 0; i < config_.motors.size(); ++i) {
      const auto & motor_cfg = config_.motors[i];
      if (address_to_index_.count(motor_cfg.address) != 0U) {
        std::fprintf(stderr, "ethercat_driver: duplicated motor address: %s\n", toString(motor_cfg.address).c_str());
        return false;
      }
      address_to_index_[motor_cfg.address] = i;
      motors_.push_back(std::make_shared<MotorContext>(motor_cfg));
    }
    return true;
  }

  bool validateConfig(const DriverConfig & config) const
  {
    if (config.motors.empty()) {
      std::fprintf(stderr, "ethercat_driver: at least one motor must be configured\n");
      return false;
    }
    if (config.cycle_ns <= 0) {
      std::fprintf(stderr, "ethercat_driver: cycle_ns must be positive\n");
      return false;
    }
    if (config.sync0_shift_ns < 0) {
      std::fprintf(stderr, "ethercat_driver: sync0_shift_ns must be >= 0\n");
      return false;
    }
    if (config.enable_dc && config.sync0_shift_ns >= config.cycle_ns) {
      std::fprintf(
        stderr,
        "ethercat_driver: sync0_shift_ns (%ld) must be smaller than cycle_ns (%ld)\n",
        static_cast<long>(config.sync0_shift_ns), static_cast<long>(config.cycle_ns));
      return false;
    }

    const unsigned int master_index = config.motors.front().address.master_index;
    for (const auto & motor : config.motors) {
      if (motor.address.master_index != master_index) {
        std::fprintf(stderr, "ethercat_driver: one driver instance can only control motors on one IgH master\n");
        return false;
      }
      if (motor.address.vendor_id == 0U || motor.address.product_code == 0U) {
        std::fprintf(stderr, "ethercat_driver: vendor_id/product_code must not be zero for %s\n", motor.name.c_str());
        return false;
      }
      if (motor.max_velocity_counts_s <= 0.0 || motor.max_accel_counts_s2 <= 0.0 || motor.counts_per_rev <= 0.0) {
        std::fprintf(stderr, "ethercat_driver: invalid motion parameter for %s\n", motor.name.c_str());
        return false;
      }
    }
    return true;
  }

  std::shared_ptr<MotorContext> findMotor(const MotorAddress & address) const
  {
    const auto iter = address_to_index_.find(address);
    if (iter == address_to_index_.end()) {
      return nullptr;
    }
    return motors_.at(iter->second);
  }

  void fillMotorState(const MotorContext & motor, MotorState & state) const
  {
    state.ethercat_ok = ethercat_ok_.load();
    state.slave_online = motor.slave_online.load();
    state.slave_operational = motor.slave_operational.load();
    state.operation_enabled = motor.operation_enabled.load();
    state.target_reached = motor.target_reached.load();
    const auto cia402_state = static_cast<Cia402State>(motor.cia402_state.load());
    state.fault = cia402_state == Cia402State::Fault;
    state.fault_reaction_active = cia402_state == Cia402State::FaultReactionActive;
    state.quick_stop_active = cia402_state == Cia402State::QuickStopActive;
    state.status_word = motor.statusword.load();
    state.error_code = motor.error_code.load();
    state.error_code_valid = motor.error_code_valid.load();
    state.cia402_state = motor.cia402_state.load();
    state.slave_al_state = motor.slave_al_state.load();
    state.domain_wc = domain_wc_.load();
    state.domain_wc_state = domain_wc_state_.load();
    state.actual_counts = motor.actual_counts.load();
    state.actual_velocity_raw = motor.actual_velocity_raw.load();
    state.actual_torque_raw = motor.actual_torque_raw.load();
    state.command_counts = motor.command_counts.load();
    state.goal_counts = motor.goal_counts.load();
    state.goal_update_sequence =
      static_cast<uint32_t>(motor.goal_command_snapshot.load() >> 32U);
  }

  void setupRealtime()
  {
    if (!config_.enable_realtime) {
      return;
    }

    if (config_.realtime_cpu >= 0) {
      if (config_.realtime_cpu >= CPU_SETSIZE) {
        std::fprintf(
          stderr, "ethercat_driver: realtime_cpu=%d exceeds CPU_SETSIZE=%d\n",
          config_.realtime_cpu, CPU_SETSIZE);
      } else {
        cpu_set_t cpu_set;
        CPU_ZERO(&cpu_set);
        CPU_SET(config_.realtime_cpu, &cpu_set);
        const int affinity_ret = pthread_setaffinity_np(
          pthread_self(), sizeof(cpu_set), &cpu_set);
        if (affinity_ret != 0) {
          std::fprintf(
            stderr, "ethercat_driver: pthread_setaffinity_np(cpu=%d) failed: %s\n",
            config_.realtime_cpu, std::strerror(affinity_ret));
        } else {
          std::fprintf(
            stderr, "ethercat_driver: realtime CPU affinity active cpu=%d\n",
            config_.realtime_cpu);
        }
      }
    }

    const bool memory_locked = mlockall(MCL_CURRENT | MCL_FUTURE) == 0;
    if (!memory_locked) {
      std::fprintf(stderr, "ethercat_driver: mlockall failed: %s\n", std::strerror(errno));
    }

    sched_param param{};
    param.sched_priority = config_.realtime_priority;
    const int sched_ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (sched_ret != 0) {
      std::fprintf(
        stderr,
        "ethercat_driver: pthread_setschedparam(SCHED_FIFO, %d) failed: %s. Run as root or grant CAP_SYS_NICE.\n",
        config_.realtime_priority, std::strerror(sched_ret));
    } else {
      std::fprintf(
        stderr, "ethercat_driver: realtime active policy=SCHED_FIFO priority=%d memory_locked=%s\n",
        config_.realtime_priority, memory_locked ? "yes" : "no");
    }
  }

  bool initEthercat()
  {
    if (!initMasterNoLock()) {
      return false;
    }
    if (!configureSlaveNoLock()) {
      return false;
    }
    if (!configurePdoNoLock()) {
      return false;
    }
    if (!activateMasterNoLock()) {
      return false;
    }
    return true;
  }

  bool initMasterNoLock()
  {
    if (master_ != nullptr) {
      std::fprintf(stderr, "ethercat_driver: init_master skipped, master already initialized\n");
      return true;
    }
    if (motors_.empty()) {
      std::fprintf(stderr, "ethercat_driver: init_master failed, no motor configured\n");
      return false;
    }

    const unsigned int master_index = config_.motors.front().address.master_index;

    master_ = ecrt_request_master(master_index);
    if (master_ == nullptr) {
      std::fprintf(stderr, "ethercat_driver: ecrt_request_master(%u) failed. Check IgH service and permissions.\n", master_index);
      return false;
    }

    domain_ = ecrt_master_create_domain(master_);
    if (domain_ == nullptr) {
      std::fprintf(stderr, "ethercat_driver: ecrt_master_create_domain failed\n");
      return false;
    }

    master_initialized_ = true;
    std::fprintf(stderr, "ethercat_driver: init_master OK, master=%u\n", master_index);
    return true;
  }

  bool configureSlaveNoLock()
  {
    if (!master_initialized_ || master_ == nullptr || domain_ == nullptr) {
      std::fprintf(stderr, "ethercat_driver: configure_slave failed, call init_master() first\n");
      return false;
    }

    for (auto & motor : motors_) {
      const auto & addr = motor->config.address;
      motor->sc = ecrt_master_slave_config(master_, addr.alias, addr.slave_position, addr.vendor_id, addr.product_code);
      if (motor->sc == nullptr) {
        std::fprintf(stderr, "ethercat_driver: failed to get slave config for %s (%s)\n", motor->config.name.c_str(), toString(addr).c_str());
        return false;
      }
      motor->error_code_request =
        ecrt_slave_config_create_sdo_request(motor->sc, 0x603F, 0x00, 2);
      if (motor->error_code_request == nullptr) {
        std::fprintf(stderr, "ethercat_driver: failed to create 0x603F SDO request for %s\n", motor->config.name.c_str());
        return false;
      }
      ecrt_sdo_request_timeout(motor->error_code_request, 500);
      motor->next_error_code_cycle = 200U +
        static_cast<uint64_t>(addr.slave_position) * 100U;
    }

    configureReferenceClock();

    for (auto & motor : motors_) {
      if (!configureMotorSdo(*motor)) {
        return false;
      }
    }

    slaves_configured_ = true;
    std::fprintf(stderr, "ethercat_driver: configure_slave OK, motors=%zu\n", motors_.size());
    return true;
  }

  bool configurePdoNoLock()
  {
    if (!slaves_configured_) {
      std::fprintf(stderr, "ethercat_driver: configure_pdo failed, call configure_slave() first\n");
      return false;
    }
    if (domain_ == nullptr) {
      std::fprintf(stderr, "ethercat_driver: configure_pdo failed, domain is null\n");
      return false;
    }

    for (auto & motor : motors_) {
      if (!configureMotorPdo(*motor)) {
        return false;
      }
    }

    std::vector<ec_pdo_entry_reg_t> domain_regs;
    const size_t entries_per_motor = config_.enable_velocity_torque_feedback ? 8U : 4U;
    domain_regs.reserve(motors_.size() * entries_per_motor + 1U);
    for (auto & motor : motors_) {
      const auto & addr = motor->config.address;
      domain_regs.push_back({addr.alias, addr.slave_position, addr.vendor_id, addr.product_code, kPdoControlWord, 0x00, &motor->offsets.control_word, nullptr});
      domain_regs.push_back({addr.alias, addr.slave_position, addr.vendor_id, addr.product_code, kPdoTargetPosition, 0x00, &motor->offsets.target_position, nullptr});
      domain_regs.push_back({addr.alias, addr.slave_position, addr.vendor_id, addr.product_code, kPdoStatusWord, 0x00, &motor->offsets.status_word, nullptr});
      domain_regs.push_back({addr.alias, addr.slave_position, addr.vendor_id, addr.product_code, kPdoActualPosition, 0x00, &motor->offsets.actual_position, nullptr});
      if (config_.enable_velocity_torque_feedback) {
        domain_regs.push_back({addr.alias, addr.slave_position, addr.vendor_id, addr.product_code, kPdoActualVelocity, 0x00, &motor->offsets.actual_velocity, nullptr});
        domain_regs.push_back({addr.alias, addr.slave_position, addr.vendor_id, addr.product_code, kPdoActualTorque, 0x00, &motor->offsets.actual_torque, nullptr});
        domain_regs.push_back({addr.alias, addr.slave_position, addr.vendor_id, addr.product_code, kPdoActualCurrent, 0x00, &motor->offsets.actual_current, nullptr});
        domain_regs.push_back({addr.alias, addr.slave_position, addr.vendor_id, addr.product_code, kPdoFollowingErrorActual, 0x00, &motor->offsets.following_error_actual, nullptr});
      }
    }
    domain_regs.push_back({});

    const int reg_ret = ecrt_domain_reg_pdo_entry_list(domain_, domain_regs.data());
    if (reg_ret != 0) {
      std::fprintf(stderr, "ethercat_driver: ecrt_domain_reg_pdo_entry_list failed: %d\n", reg_ret);
      return false;
    }

    if (config_.enable_dc) {
      for (auto & motor : motors_) {
        ecrt_slave_config_dc(
          motor->sc, config_.assign_activate, static_cast<uint32_t>(config_.cycle_ns),
          static_cast<int32_t>(config_.sync0_shift_ns), 0, 0);
      }
    }

    pdo_configured_ = true;
    if (config_.enable_velocity_torque_feedback) {
      std::fprintf(
        stderr,
        "ethercat_driver: configure_pdo OK, RxPDO=0x1601 "
        "TxPDO=0x1A01 remapped=[0x6041,0x6064,0x606C,0x6077,0x6078,0x60F4]\n");
    } else {
      std::fprintf(stderr, "ethercat_driver: configure_pdo OK, RxPDO=0x1601 TxPDO=0x1A01\n");
    }
    return true;
  }

  bool activateMasterNoLock()
  {
    if (!pdo_configured_) {
      std::fprintf(stderr, "ethercat_driver: activate_master failed, call configure_pdo() first\n");
      return false;
    }
    if (master_ == nullptr || domain_ == nullptr) {
      std::fprintf(stderr, "ethercat_driver: activate_master failed, master/domain is null\n");
      return false;
    }

    const int active_ret = ecrt_master_activate(master_);
    if (active_ret != 0) {
      std::fprintf(stderr, "ethercat_driver: ecrt_master_activate failed: %d\n", active_ret);
      return false;
    }

    domain_pd_ = ecrt_domain_data(domain_);
    if (domain_pd_ == nullptr) {
      std::fprintf(stderr, "ethercat_driver: ecrt_domain_data returned null\n");
      return false;
    }

    const unsigned int master_index = config_.motors.front().address.master_index;
    std::fprintf(
      stderr,
      "ethercat_driver: activate_master OK master=%u motors=%zu cycle=%ld ns DC=%s assign=0x%04X sync0_shift=%ld ns ref_alias=%u ref_position=%u\n",
      master_index, motors_.size(), static_cast<long>(config_.cycle_ns),
      config_.enable_dc ? "on" : "off", config_.assign_activate,
      static_cast<long>(config_.sync0_shift_ns), config_.dc_reference_alias, config_.dc_reference_slave_position);

    for (const auto & motor : motors_) {
      std::fprintf(stderr, "ethercat_driver: motor '%s' address %s\n", motor->config.name.c_str(), toString(motor->config.address).c_str());
    }

    master_activated_ = true;
    ethercat_ok_.store(true);
    return true;
  }

  void shutdownNoLock()
  {
    if (domain_pd_ != nullptr && domain_ != nullptr && master_ != nullptr) {
      for (auto & motor : motors_) {
        EC_WRITE_U16(domain_pd_ + motor->offsets.control_word, 0x0006);
      }
      ecrt_domain_queue(domain_);
      ecrt_master_send(master_);
    }

    if (master_ != nullptr) {
      ecrt_release_master(master_);
    }

    master_ = nullptr;
    domain_ = nullptr;
    ref_sc_ = nullptr;
    domain_pd_ = nullptr;

    master_initialized_ = false;
    slaves_configured_ = false;
    pdo_configured_ = false;
    master_activated_ = false;

    ethercat_ok_.store(false);
    domain_wc_.store(0);
    domain_wc_state_.store(0);
  }

  void configureReferenceClock()
  {
    if (!config_.enable_dc || !config_.select_dc_reference_clock) {
      return;
    }

    ref_sc_ = nullptr;
    for (auto & motor : motors_) {
      const auto & addr = motor->config.address;
      if (addr.alias == config_.dc_reference_alias && addr.slave_position == config_.dc_reference_slave_position) {
        ref_sc_ = motor->sc;
        break;
      }
    }

    if (ref_sc_ == nullptr) {
      ref_sc_ = motors_.front()->sc;
      std::fprintf(
        stderr,
        "ethercat_driver: configured DC reference alias=%u position=%u was not in motor list; using first motor as reference clock\n",
        config_.dc_reference_alias, config_.dc_reference_slave_position);
    }

    ecrt_master_select_reference_clock(master_, ref_sc_);
  }

  bool configureMotorSdo(MotorContext & motor)
  {
    if (!motor.config.configure_sdo) {
      return true;
    }

    const auto write_sdo8 = [&motor](uint16_t index, uint8_t subindex, uint8_t value) {
      const int ret = ecrt_slave_config_sdo8(motor.sc, index, subindex, value);
      if (ret != 0) {
        std::fprintf(
          stderr,
          "ethercat_driver: SDO config failed for %s at 0x%04X:%02X value=0x%02X: %d\n",
          motor.config.name.c_str(), index, subindex, value, ret);
        return false;
      }
      return true;
    };

    if (!write_sdo8(0x6060, 0x00, static_cast<uint8_t>(motor.config.mode_of_operation)) ||
      !write_sdo8(0x60C2, 0x01, 0x01) ||
      !write_sdo8(0x60C2, 0x02, 0xFD))  // 1 * 10^-3 s = 1 ms
    {
      return false;
    }
    return true;
  }

  bool configureMotorPdo(MotorContext & motor)
  {
    if (!motor.config.configure_pdos) {
      return true;
    }

    // Do not select the ESI's 0x1A00 PDO directly: the tested GH1 firmware
    // rejected that 13-byte SM3 layout with AL 0x0017. The ESI marks 0x1A01
    // as remappable (Fixed=0), so keep the known-good PDO index and optionally
    // extend it to 18 bytes with the ESI-defined feedback objects:
    //   0x6041:00 UINT  Status word
    //   0x6064:00 DINT  Position actual value
    //   0x606C:00 DINT  Velocity actual value
    //   0x6077:00 INT   Torque actual value
    //   0x6078:00 INT   Current actual value
    //   0x60F4:00 DINT  Following error actual value
    static ec_pdo_entry_info_t base_pdo_entries[] = {
      {kPdoControlWord, 0x00, 16},
      {kPdoTargetPosition, 0x00, 32},
      {kPdoStatusWord, 0x00, 16},
      {kPdoActualPosition, 0x00, 32},
    };
    static ec_pdo_info_t base_pdos[] = {
      {0x1601, 2, base_pdo_entries + 0},
      {0x1A01, 2, base_pdo_entries + 2},
    };
    static ec_sync_info_t base_syncs[] = {
      {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
      {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
      {2, EC_DIR_OUTPUT, 1, base_pdos + 0, EC_WD_ENABLE},
      {3, EC_DIR_INPUT, 1, base_pdos + 1, EC_WD_DISABLE},
      {0xFF, static_cast<ec_direction_t>(0), 0, nullptr, static_cast<ec_watchdog_mode_t>(0)}
    };

    static ec_pdo_entry_info_t extended_pdo_entries[] = {
      {kPdoControlWord, 0x00, 16},
      {kPdoTargetPosition, 0x00, 32},
      {kPdoStatusWord, 0x00, 16},
      {kPdoActualPosition, 0x00, 32},
      {kPdoActualVelocity, 0x00, 32},
      {kPdoActualTorque, 0x00, 16},
      {kPdoActualCurrent, 0x00, 16},
      {kPdoFollowingErrorActual, 0x00, 32},
    };
    static ec_pdo_info_t extended_pdos[] = {
      {0x1601, 2, extended_pdo_entries + 0},
      {0x1A01, 6, extended_pdo_entries + 2},
    };
    static ec_sync_info_t extended_syncs[] = {
      {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
      {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
      {2, EC_DIR_OUTPUT, 1, extended_pdos + 0, EC_WD_ENABLE},
      {3, EC_DIR_INPUT, 1, extended_pdos + 1, EC_WD_DISABLE},
      {0xFF, static_cast<ec_direction_t>(0), 0, nullptr, static_cast<ec_watchdog_mode_t>(0)}
    };

    ec_sync_info_t * syncs = config_.enable_velocity_torque_feedback ?
      extended_syncs : base_syncs;
    const int ret = ecrt_slave_config_pdos(motor.sc, EC_END, syncs);
    if (ret != 0) {
      std::fprintf(stderr, "ethercat_driver: ecrt_slave_config_pdos failed for %s: %d\n", motor.config.name.c_str(), ret);
      return false;
    }
    return true;
  }

  uint16_t controlwordFor(MotorContext & motor, uint16_t status_word)
  {
    const Cia402State state = decodeStatusWord(status_word);

    if (state == Cia402State::Fault) {
      // Latch the drive disabled on every fault. A reset pulse may clear the fault,
      // but motion can only resume after a separate explicit enable command.
      motor.command_enable.store(false);
      if (motor.fault_reset_requested.exchange(false)) {
        motor.fault_reset_counter = 50;
      }
      if (motor.fault_reset_counter > 0) {
        --motor.fault_reset_counter;
        return 0x0080;
      }
      return 0x0000;
    }

    motor.fault_reset_counter = 0;

    if (state == Cia402State::QuickStopActive) {
      // Keep Quick Stop asserted and require an explicit disable/enable recovery
      // sequence. Never jump directly back to Operation Enabled.
      motor.command_enable.store(false);
      if (motor.quick_stop_release_requested.load()) {
        // Disable Voltage is the conservative CiA402 exit from QuickStopActive.
        // Once the drive leaves this state, command_enable remains false and the
        // normal disabled path advances only as far as ReadyToSwitchOn (0x0006).
        return 0x0000;
      }
      return 0x0002;
    }

    // A release request is only consumed after the drive has actually left
    // QuickStopActive. A new Quick Stop therefore requires a new manual request.
    motor.quick_stop_release_requested.store(false);

    if (!motor.command_enable.load()) {
      return 0x0006;  // shutdown: voltage enabled, operation disabled
    }

    switch (state) {
      case Cia402State::SwitchOnDisabled:
        return 0x0006;  // shutdown
      case Cia402State::ReadyToSwitchOn:
        return 0x0007;  // switch on
      case Cia402State::SwitchedOn:
        return 0x000F;  // enable operation
      case Cia402State::OperationEnabled:
        return 0x000F;  // keep enabled
      case Cia402State::QuickStopActive:
        return 0x0002;
      case Cia402State::NotReadyToSwitchOn:
      case Cia402State::FaultReactionActive:
      case Cia402State::Fault:
      case Cia402State::Unknown:
        return 0x0006;
    }
    return 0x0006;
  }

  void updateProfile(MotorContext & motor, double dt, int32_t actual_position, bool operation_enabled)
  {
    if (!operation_enabled || !motor.command_enable.load()) {
      motor.cmd_position = static_cast<double>(actual_position);
      motor.profile_velocity = 0.0;

      // 未真正进入 OperationEnabled 前，只跟随当前位置，不执行旧目标
      motor.goal_counts.store(actual_position);
      motor.have_goal.store(true);
      motor.target_reached.store(true);
      motor.hold_latched_after_enable = false;
      return;
    }

    if (!motor.hold_latched_after_enable) {
      // 第一次真正使能后，再用当前 PDO 实际位置锁存保持目标
      // 防止启动瞬间 actual_position 读到 0 后，电机去归零
      motor.cmd_position = static_cast<double>(actual_position);
      motor.profile_velocity = 0.0;
      motor.goal_counts.store(actual_position);
      motor.have_goal.store(true);
      motor.target_reached.store(true);
      motor.hold_latched_after_enable = true;
      return;
    }

    const double goal = static_cast<double>(motor.goal_counts.load());



    const double dist = goal - motor.cmd_position;
    const double abs_dist = std::abs(dist);

    if (abs_dist <= static_cast<double>(motor.config.target_tolerance_counts) && std::abs(motor.profile_velocity) < 1.0) {
      motor.cmd_position = goal;
      motor.profile_velocity = 0.0;
      motor.target_reached.store(true);
      return;
    }

    motor.target_reached.store(false);
    const double direction = (dist >= 0.0) ? 1.0 : -1.0;
    const double braking_distance = (motor.profile_velocity * motor.profile_velocity) / (2.0 * motor.config.max_accel_counts_s2);
    double desired_velocity = direction * motor.config.max_velocity_counts_s;

    if (braking_distance >= abs_dist) {
      desired_velocity = 0.0;
    }

    const double max_delta_v = motor.config.max_accel_counts_s2 * dt;
    if (motor.profile_velocity < desired_velocity) {
      motor.profile_velocity = std::min(motor.profile_velocity + max_delta_v, desired_velocity);
    } else if (motor.profile_velocity > desired_velocity) {
      motor.profile_velocity = std::max(motor.profile_velocity - max_delta_v, desired_velocity);
    }

    double next_position = motor.cmd_position + motor.profile_velocity * dt;
    if ((direction > 0.0 && next_position > goal) || (direction < 0.0 && next_position < goal)) {
      next_position = goal;
      motor.profile_velocity = 0.0;
    }
    motor.cmd_position = next_position;
  }

  void setGoalCountsClamped(MotorContext & motor, int64_t target)
  {
    const int64_t clamped = std::clamp<int64_t>(target, INT32_MIN, INT32_MAX);
    motor.goal_counts.store(clamped);
    uint64_t previous = motor.goal_command_snapshot.load(std::memory_order_relaxed);
    while (true) {
      const uint32_t next_sequence =
        static_cast<uint32_t>(previous >> 32U) + 1U;
      const uint64_t next =
        (static_cast<uint64_t>(next_sequence) << 32U) |
        static_cast<uint32_t>(static_cast<int32_t>(clamped));
      if (motor.goal_command_snapshot.compare_exchange_weak(
          previous, next, std::memory_order_release, std::memory_order_relaxed))
      {
        break;
      }
    }
    motor.have_goal.store(true);
    motor.target_reached.store(false);
  }

  void loop()
  {
    if (!initEthercat()) {
      ethercat_ok_.store(false);
      running_.store(false);
      return;
    }
    if (config_.pdo_diagnostics_auto_start) {
      std::string message;
      if (startPdoRecording(message)) {
        pdo_recording_cycle_limit_.store(
          config_.pdo_diagnostics_max_cycles, std::memory_order_relaxed);
        if (config_.pdo_diagnostics_max_cycles > 0U) {
          message += " max_cycles=" + std::to_string(config_.pdo_diagnostics_max_cycles);
        }
        std::fprintf(stderr, "ethercat_driver: %s\n", message.c_str());
      } else {
        std::fprintf(
          stderr, "ethercat_driver: automatic PDO recording failed: %s\n",
          message.c_str());
      }
    }
    // Enter FIFO scheduling only after all allocation and EtherCAT setup work is done.
    setupRealtime();
    ethercat_ok_.store(true);

    timespec wake_time{};
    clock_gettime(CLOCK_MONOTONIC, &wake_time);
    const double dt = static_cast<double>(config_.cycle_ns) * 1e-9;
    uint64_t cycle_count = 0;
    uint64_t absolute_cycle_count = 0;
    uint64_t deadline_miss_count = 0;
    uint64_t total_deadline_miss_count = 0;
    uint64_t late_sample_count = 0;
    uint64_t late_total_ns = 0;
    uint64_t max_late_ns = 0;
    uint64_t max_execution_ns = 0;
    int64_t stats_start_ns = monotonicNs();
    int64_t previous_cycle_start_ns = 0;
    int64_t wake_lateness_for_cycle_ns = 0;

    while (running_.load()) {
      addNs(wake_time, config_.cycle_ns);
      const int64_t cycle_start_ns = monotonicNs();
      const int64_t scheduled_cycle_start_ns =
        static_cast<int64_t>(timespecToNs(wake_time)) - config_.cycle_ns;
      const int64_t cycle_period_ns = previous_cycle_start_ns == 0 ?
        config_.cycle_ns : cycle_start_ns - previous_cycle_start_ns;
      const int64_t period_jitter_ns = previous_cycle_start_ns == 0 ?
        0 : cycle_period_ns - config_.cycle_ns;
      const int64_t start_lateness_ns = cycle_start_ns - scheduled_cycle_start_ns;
      previous_cycle_start_ns = cycle_start_ns;

      // Apply a prepared recording transition on the first EtherCAT cycle at
      // or after the shared monotonic deadline. Both masters receive the same
      // deadline, so service scheduling latency cannot shift the capture window.
      if (config_.enable_dc) {
        timespec app_time{};
        clock_gettime(CLOCK_MONOTONIC, &app_time);
        ecrt_master_application_time(master_, timespecToNs(app_time));
      }

      ecrt_master_receive(master_);
      ecrt_domain_process(domain_);

      // Refresh working-counter health every cycle so ros2_control reacts within
      // one millisecond instead of waiting for the throttled diagnostic poll.
      ec_domain_state_t cyclic_domain_state{};
      ecrt_domain_state(domain_, &cyclic_domain_state);
      domain_wc_.store(static_cast<int>(cyclic_domain_state.working_counter));
      domain_wc_state_.store(static_cast<int>(cyclic_domain_state.wc_state));
      ethercat_ok_.store(cyclic_domain_state.wc_state == EC_WC_COMPLETE);

      PdoDiagnosticRecord * pdo_record = nullptr;
      uint64_t pdo_write_sequence = 0;
      if (pdo_recording_enabled_.load(std::memory_order_acquire)) {
        pdo_diagnostics_producer_active_.fetch_add(1, std::memory_order_acq_rel);
        if (pdo_recording_enabled_.load(std::memory_order_acquire)) {
          pdo_write_sequence =
            pdo_diagnostics_write_sequence_.load(std::memory_order_relaxed);
          const uint64_t pdo_read_sequence =
            pdo_diagnostics_read_sequence_.load(std::memory_order_acquire);
          if (!pdo_diagnostics_ring_.empty() &&
            pdo_write_sequence - pdo_read_sequence < pdo_diagnostics_ring_.size())
          {
            pdo_record =
              &pdo_diagnostics_ring_[pdo_write_sequence % pdo_diagnostics_ring_.size()];
            pdo_record->monotonic_ns = cycle_start_ns;
            pdo_record->scheduled_cycle_start_ns = scheduled_cycle_start_ns;
            pdo_record->cycle_period_ns = cycle_period_ns;
            pdo_record->period_jitter_ns = period_jitter_ns;
            pdo_record->start_lateness_ns = start_lateness_ns;
            pdo_record->wake_lateness_ns = wake_lateness_for_cycle_ns;
            pdo_record->deadline_miss_total = total_deadline_miss_count;
            pdo_record->dropped_cycles_total =
              pdo_diagnostics_dropped_cycles_.load(std::memory_order_relaxed);
            pdo_record->cycle = absolute_cycle_count;
            pdo_record->domain_wc = cyclic_domain_state.working_counter;
            pdo_record->domain_wc_state =
              static_cast<uint8_t>(cyclic_domain_state.wc_state);
            pdo_record->motor_count = static_cast<uint8_t>(motors_.size());
          } else {
            pdo_diagnostics_dropped_cycles_.fetch_add(1, std::memory_order_relaxed);
          }
        }
        if (pdo_record == nullptr) {
          pdo_diagnostics_producer_active_.fetch_sub(1, std::memory_order_release);
        }
      }

      for (size_t motor_index = 0; motor_index < motors_.size(); ++motor_index) {
        auto & motor = motors_[motor_index];
        const uint16_t sw = EC_READ_U16(domain_pd_ + motor->offsets.status_word);
        const int32_t actual = EC_READ_S32(domain_pd_ + motor->offsets.actual_position);
        const int32_t actual_velocity = config_.enable_velocity_torque_feedback ?
          EC_READ_S32(domain_pd_ + motor->offsets.actual_velocity) : 0;
        const int16_t actual_torque = config_.enable_velocity_torque_feedback ?
          EC_READ_S16(domain_pd_ + motor->offsets.actual_torque) : 0;
        const int16_t actual_current = config_.enable_velocity_torque_feedback ?
          EC_READ_S16(domain_pd_ + motor->offsets.actual_current) : 0;
        const int32_t following_error_actual = config_.enable_velocity_torque_feedback ?
          EC_READ_S32(domain_pd_ + motor->offsets.following_error_actual) : 0;
        const Cia402State state = decodeStatusWord(sw);
        const bool op_enabled = (state == Cia402State::OperationEnabled);

        motor->actual_counts.store(actual);
        motor->actual_velocity_raw.store(actual_velocity);
        motor->actual_torque_raw.store(actual_torque);
        motor->statusword.store(sw);
        motor->cia402_state.store(static_cast<int>(state));
        motor->operation_enabled.store(op_enabled);

        // Read the standard CiA-402 error code through the mailbox at 1 Hz per
        // drive. This keeps the fixed, firmware-supported PDO mapping unchanged.
        if (motor->error_code_request != nullptr) {
          const ec_request_state_t request_state =
            ecrt_sdo_request_state(motor->error_code_request);
          if (motor->error_code_request_pending) {
            if (request_state == EC_REQUEST_SUCCESS) {
              motor->error_code.store(EC_READ_U16(ecrt_sdo_request_data(motor->error_code_request)));
              motor->error_code_valid.store(true);
              motor->error_code_request_pending = false;
              motor->next_error_code_cycle = cycle_count + 1000U;
            } else if (request_state == EC_REQUEST_ERROR) {
              motor->error_code_request_pending = false;
              motor->next_error_code_cycle = cycle_count + 1000U;
            }
          } else if (cycle_count >= motor->next_error_code_cycle &&
            request_state != EC_REQUEST_BUSY)
          {
            ecrt_sdo_request_read(motor->error_code_request);
            motor->error_code_request_pending = true;
          }
        }

        if (motor->first_cycle) {
          motor->cmd_position = static_cast<double>(actual);
          motor->profile_velocity = 0.0;
          motor->goal_counts.store(actual);
          motor->have_goal.store(true);
          motor->first_cycle = false;
        }

        uint32_t goal_update_sequence_used = static_cast<uint32_t>(
          motor->goal_command_snapshot.load(std::memory_order_acquire) >> 32U);
        const bool direct_csp_command =
          config_.csp_direct_position_commands &&
          motor->config.mode_of_operation == 8 &&
          op_enabled &&
          motor->command_enable.load() &&
          motor->hold_latched_after_enable;
        if (direct_csp_command) {
          // The JointTrajectoryController already interpolates the MoveIt trajectory
          // at the ros2_control rate. Re-profiling each moving target creates a
          // deadband at target_tolerance_counts and causes visible stop-start motion.
          const uint64_t goal_snapshot =
            motor->goal_command_snapshot.load(std::memory_order_acquire);
          const int32_t goal = static_cast<int32_t>(
            static_cast<uint32_t>(goal_snapshot));
          goal_update_sequence_used =
            static_cast<uint32_t>(goal_snapshot >> 32U);
          motor->cmd_position = static_cast<double>(goal);
          motor->profile_velocity = 0.0;
          motor->target_reached.store(
            std::llabs(goal - static_cast<int64_t>(actual)) <= motor->config.target_tolerance_counts);
        } else {
          // Preserve the legacy profile and all safe hold behavior while disabled,
          // outside CSP, or when direct CSP is explicitly disabled.
          updateProfile(*motor, dt, actual, op_enabled);
        }
        const int32_t command_position = clampToInt32(motor->cmd_position);
        motor->command_counts.store(command_position);

        const uint16_t control_word = controlwordFor(*motor, sw);
        EC_WRITE_S32(domain_pd_ + motor->offsets.target_position, command_position);
        EC_WRITE_U16(domain_pd_ + motor->offsets.control_word, control_word);

        if (pdo_record != nullptr) {
          auto & sample = pdo_record->motors[motor_index];
          sample.target_position = command_position;
          sample.actual_position = actual;
          sample.position_error = clampToInt32(
            static_cast<double>(command_position) - static_cast<double>(actual));
          const double position_scale = motor->config.position_scale_rad_per_count;
          const double position_offset =
            motor->diagnostic_position_offset_rad.load(std::memory_order_acquire);
          sample.sent_target_position_joint_rad =
            static_cast<double>(command_position) * position_scale + position_offset;
          sample.feedback_actual_position_joint_rad =
            static_cast<double>(actual) * position_scale + position_offset;
          sample.position_error_joint_rad =
            static_cast<double>(sample.position_error) * position_scale;
          sample.position_conversion_offset_rad = position_offset;
          if (motor->previous_pdo_sample_valid) {
            sample.target_delta = clampToInt32(
              static_cast<double>(command_position) -
              static_cast<double>(motor->previous_pdo_target));
            sample.actual_delta = clampToInt32(
              static_cast<double>(actual) - static_cast<double>(motor->previous_pdo_actual));
            sample.goal_sequence_delta =
              goal_update_sequence_used - motor->previous_pdo_goal_sequence;
          } else {
            sample.target_delta = 0;
            sample.actual_delta = 0;
            sample.goal_sequence_delta = 0;
          }
          sample.actual_velocity = actual_velocity;
          sample.actual_torque = actual_torque;
          sample.actual_current = actual_current;
          sample.following_error_actual = following_error_actual;
          sample.control_word = control_word;
          sample.status_word = sw;
          sample.error_code = motor->error_code.load(std::memory_order_relaxed);
          sample.error_code_valid =
            motor->error_code_valid.load(std::memory_order_relaxed);
          sample.goal_update_sequence = goal_update_sequence_used;
        }

        motor->previous_pdo_target = command_position;
        motor->previous_pdo_actual = actual;
        motor->previous_pdo_goal_sequence = goal_update_sequence_used;
        motor->previous_pdo_sample_valid = true;
      }

      if (config_.enable_dc) {
        ecrt_master_sync_reference_clock(master_);
        ecrt_master_sync_slave_clocks(master_);
      }

      ecrt_domain_queue(domain_);
      ecrt_master_send(master_);

      checkMasterStateThrottled();

      const int64_t execution_end_ns = monotonicNs();
      const int64_t scheduled_wake_ns = static_cast<int64_t>(timespecToNs(wake_time));
      const uint64_t execution_ns = static_cast<uint64_t>(
        std::max<int64_t>(0, execution_end_ns - (scheduled_wake_ns - config_.cycle_ns)));
      max_execution_ns = std::max(max_execution_ns, execution_ns);

      if (pdo_record != nullptr) {
        pdo_record->execution_ns = execution_ns;
        const uint64_t completed_records = pdo_write_sequence + 1U;
        pdo_diagnostics_write_sequence_.store(completed_records, std::memory_order_release);
        const uint64_t cycle_limit =
          pdo_recording_cycle_limit_.load(std::memory_order_relaxed);
        if (cycle_limit > 0U && completed_records >= cycle_limit)
        {
          pdo_recording_enabled_.store(false, std::memory_order_release);
        }
        pdo_diagnostics_producer_active_.fetch_sub(1, std::memory_order_release);
      }

      int sleep_ret = 0;
      do {
        sleep_ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake_time, nullptr);
      } while (sleep_ret == EINTR);
      if (sleep_ret != 0 && sleep_ret != EINTR) {
        std::fprintf(stderr, "ethercat_driver: clock_nanosleep failed: %s\n", std::strerror(sleep_ret));
      }

      const int64_t actual_wake_ns = monotonicNs();
      const int64_t lateness_ns = actual_wake_ns - scheduled_wake_ns;
      wake_lateness_for_cycle_ns = lateness_ns;
      if (lateness_ns > 0) {
        const uint64_t positive_lateness_ns = static_cast<uint64_t>(lateness_ns);
        ++late_sample_count;
        late_total_ns += positive_lateness_ns;
        max_late_ns = std::max(max_late_ns, positive_lateness_ns);
        if (lateness_ns >= config_.cycle_ns) {
          ++deadline_miss_count;
          ++total_deadline_miss_count;
          // Do not run a burst of back-to-back catch-up cycles after a large stall.
          clock_gettime(CLOCK_MONOTONIC, &wake_time);
        }
      }
      ++cycle_count;
      ++absolute_cycle_count;

      if (actual_wake_ns - stats_start_ns >= 60000000000LL) {
        const double elapsed_s = static_cast<double>(actual_wake_ns - stats_start_ns) * 1e-9;
        const double rate_hz = elapsed_s > 0.0 ? static_cast<double>(cycle_count) / elapsed_s : 0.0;
        const double mean_late_us = late_sample_count > 0 ?
          static_cast<double>(late_total_ns) / static_cast<double>(late_sample_count) / 1000.0 : 0.0;
        std::fprintf(
          stderr,
          "ethercat_driver: timing rate=%.2f Hz cycles=%" PRIu64
          " deadline_misses=%" PRIu64 " mean_late=%.1f us max_late=%.1f us max_exec=%.1f us\n",
          rate_hz, cycle_count, deadline_miss_count, mean_late_us,
          static_cast<double>(max_late_ns) / 1000.0,
          static_cast<double>(max_execution_ns) / 1000.0);
        cycle_count = 0;
        deadline_miss_count = 0;
        late_sample_count = 0;
        late_total_ns = 0;
        max_late_ns = 0;
        max_execution_ns = 0;
        stats_start_ns = actual_wake_ns;
      }
    }

    if (domain_pd_ != nullptr) {
      for (auto & motor : motors_) {
        EC_WRITE_U16(domain_pd_ + motor->offsets.control_word, 0x0006);
      }
      ecrt_domain_queue(domain_);
      ecrt_master_send(master_);
    }
  }

  void checkMasterStateThrottled()
  {
    if (master_ == nullptr || domain_ == nullptr) {
      return;
    }

    const int64_t now_ns = monotonicNs();
    if (now_ns - last_state_log_ns_ < 1000000000LL) {
      return;
    }
    last_state_log_ns_ = now_ns;

    ec_master_state_t master_state{};
    ec_domain_state_t domain_state{};
    ecrt_master_state(master_, &master_state);
    ecrt_domain_state(domain_, &domain_state);

    domain_wc_.store(static_cast<int>(domain_state.working_counter));
    domain_wc_state_.store(static_cast<int>(domain_state.wc_state));

    if (master_state.slaves_responding != last_master_slaves_ ||
      master_state.al_states != last_master_al_states_ ||
      master_state.link_up != last_master_link_up_ ||
      domain_state.working_counter != last_domain_wc_ ||
      domain_state.wc_state != last_domain_wc_state_)
    {
      std::fprintf(
        stderr,
        "ethercat_driver: master%u slaves=%u al=0x%02X link=%u | domain wc=%u wc_state=%u\n",
        config_.motors.front().address.master_index,
        master_state.slaves_responding, master_state.al_states, master_state.link_up,
        domain_state.working_counter, domain_state.wc_state);

      last_master_slaves_ = master_state.slaves_responding;
      last_master_al_states_ = master_state.al_states;
      last_master_link_up_ = master_state.link_up;
      last_domain_wc_ = domain_state.working_counter;
      last_domain_wc_state_ = domain_state.wc_state;
    }

    for (auto & motor : motors_) {
      ec_slave_config_state_t slave_state{};
      ecrt_slave_config_state(motor->sc, &slave_state);
      motor->slave_al_state.store(static_cast<int>(slave_state.al_state));
      motor->slave_online.store(slave_state.online);
      motor->slave_operational.store(slave_state.operational);

      if (slave_state.al_state != motor->last_slave_al_state ||
        slave_state.online != motor->last_slave_online ||
        slave_state.operational != motor->last_slave_operational)
      {
        std::fprintf(
          stderr,
          "ethercat_driver: motor '%s' online=%u op=%u al=0x%02X\n",
          motor->config.name.c_str(), slave_state.online, slave_state.operational, slave_state.al_state);

        motor->last_slave_al_state = slave_state.al_state;
        motor->last_slave_online = slave_state.online;
        motor->last_slave_operational = slave_state.operational;
      }
    }
  }

  DriverConfig config_{};

  bool master_initialized_ = false;
  bool slaves_configured_ = false;
  bool pdo_configured_ = false;
  bool master_activated_ = false;

  ec_master_t * master_ = nullptr;
  ec_domain_t * domain_ = nullptr;
  ec_slave_config_t * ref_sc_ = nullptr;
  uint8_t * domain_pd_ = nullptr;

  std::vector<std::shared_ptr<MotorContext>> motors_;
  std::map<MotorAddress, size_t, MotorAddressLess> address_to_index_;

  std::atomic<bool> running_{false};
  std::atomic<bool> ethercat_ok_{false};
  std::atomic<int> domain_wc_{0};
  std::atomic<int> domain_wc_state_{0};

  std::thread worker_;
  mutable std::mutex lifecycle_mutex_;

  std::atomic<bool> pdo_recording_enabled_{false};
  std::atomic<unsigned int> pdo_diagnostics_producer_active_{0};
  std::atomic<uint64_t> pdo_diagnostics_write_sequence_{0};
  std::atomic<uint64_t> pdo_diagnostics_read_sequence_{0};
  std::atomic<uint64_t> pdo_diagnostics_dropped_cycles_{0};
  std::atomic<uint64_t> pdo_recording_cycle_limit_{0};
  std::vector<PdoDiagnosticRecord> pdo_diagnostics_ring_;
  std::thread pdo_diagnostics_writer_;
  std::mutex pdo_diagnostics_mutex_;
  std::FILE * pdo_diagnostics_file_ = nullptr;
  std::string pdo_diagnostics_path_;

  int64_t last_state_log_ns_ = 0;
  uint16_t last_master_slaves_ = 0xFFFF;
  uint8_t last_master_al_states_ = 0xFF;
  bool last_master_link_up_ = false;
  uint32_t last_domain_wc_ = UINT32_MAX;
  unsigned int last_domain_wc_state_ = UINT32_MAX;
};

EthercatDriver::EthercatDriver()
: impl_(std::make_unique<Impl>())
{
}

EthercatDriver::~EthercatDriver() = default;

bool EthercatDriver::start(const DriverConfig & config)
{
  return impl_->start(config);
}

void EthercatDriver::stop()
{
  impl_->stop();
}

bool EthercatDriver::init_master(const DriverConfig & config)
{
  return impl_->init_master(config);
}

bool EthercatDriver::configure_slave()
{
  return impl_->configure_slave();
}

bool EthercatDriver::configure_pdo()
{
  return impl_->configure_pdo();
}

bool EthercatDriver::activate_master()
{
  return impl_->activate_master();
}

void EthercatDriver::shutdown()
{
  impl_->shutdown();
}

bool EthercatDriver::isRunning() const
{
  return impl_->isRunning();
}

bool EthercatDriver::isEthercatOk() const
{
  return impl_->isEthercatOk();
}

bool EthercatDriver::setAbsoluteTargetCounts(const MotorAddress & address, int32_t counts)
{
  return impl_->setAbsoluteTargetCounts(address, counts);
}

bool EthercatDriver::setDiagnosticPositionOffsetRad(
  const MotorAddress & address, double offset_rad)
{
  return impl_->setDiagnosticPositionOffsetRad(address, offset_rad);
}

bool EthercatDriver::setRelativeTargetCounts(const MotorAddress & address, int32_t delta_counts)
{
  return impl_->setRelativeTargetCounts(address, delta_counts);
}

bool EthercatDriver::setTargetDegrees(const MotorAddress & address, double degrees)
{
  return impl_->setTargetDegrees(address, degrees);
}

bool EthercatDriver::setRelativeDegrees(const MotorAddress & address, double delta_degrees)
{
  return impl_->setRelativeDegrees(address, delta_degrees);
}

bool EthercatDriver::setTargetRadians(const MotorAddress & address, double radians)
{
  return impl_->setTargetRadians(address, radians);
}

bool EthercatDriver::setRelativeRadians(const MotorAddress & address, double delta_radians)
{
  return impl_->setRelativeRadians(address, delta_radians);
}

bool EthercatDriver::setEnable(const MotorAddress & address, bool enable)
{
  return impl_->setEnable(address, enable);
}

bool EthercatDriver::getEnabledStatus(const MotorAddress &address, bool &enable) const
{
    return impl_->getEnabledStatus(address, enable);
}
bool EthercatDriver::requestFaultReset(const MotorAddress &address)
{
  return impl_->requestFaultReset(address);
}

bool EthercatDriver::requestQuickStopRelease(const MotorAddress & address)
{
  return impl_->requestQuickStopRelease(address);
}

bool EthercatDriver::getMotorState(const MotorAddress & address, MotorState & state) const
{
  return impl_->getMotorState(address, state);
}

std::vector<MotorState> EthercatDriver::getAllMotorStates() const
{
  return impl_->getAllMotorStates();
}

bool EthercatDriver::startPdoRecording(std::string & message)
{
  return impl_->startPdoRecording(message);
}

bool EthercatDriver::stopPdoRecording(std::string & message)
{
  return impl_->stopPdoRecording(message);
}

bool EthercatDriver::isPdoRecording() const
{
  return impl_->isPdoRecording();
}

}  // namespace ethercat_driver
