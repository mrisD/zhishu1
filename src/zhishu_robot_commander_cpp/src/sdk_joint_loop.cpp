#include "zhishu_robot_commander_cpp/sdk.hpp"
#include "stability_pdo_recording.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using SteadyClock = std::chrono::steady_clock;
using namespace std::chrono_literals;

// Keep the ROS context alive after Ctrl+C long enough to cancel the active
// MoveIt action cleanly. The signal handler itself must remain async-signal-safe.
volatile std::sig_atomic_t stop_requested = 0;

void requestStop(int)
{
  stop_requested = 1;
}

bool shutdownRequested()
{
  return stop_requested != 0 || !rclcpp::ok();
}

template<typename ParameterT>
ParameterT getOrDeclareParameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const ParameterT & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<ParameterT>();
  }
  return node->declare_parameter<ParameterT>(name, default_value);
}

zhishu_robot_commander_cpp::Commander::Options commanderOptions(
  const rclcpp::Node::SharedPtr & node)
{
  zhishu_robot_commander_cpp::Commander::Options options;
  options.left_end_effector_link = getOrDeclareParameter<std::string>(
    node, "commander.left_end_effector_link", "L_hand_link");
  options.right_end_effector_link = getOrDeclareParameter<std::string>(
    node, "commander.right_end_effector_link", "R_hand_link");
  options.left_tcp_link = getOrDeclareParameter<std::string>(
    node, "commander.left_tcp_link", "left_gripper_tcp");
  options.right_tcp_link = getOrDeclareParameter<std::string>(
    node, "commander.right_tcp_link", "right_gripper_tcp");
  options.velocity_scaling = getOrDeclareParameter<double>(
    node, "commander.velocity_scaling", 0.30);
  options.acceleration_scaling = getOrDeclareParameter<double>(
    node, "commander.acceleration_scaling", 0.10);
  options.planning_time = getOrDeclareParameter<double>(
    node, "commander.planning_time", 5.0);
  options.planning_attempts = static_cast<int>(getOrDeclareParameter<std::int64_t>(
      node, "commander.planning_attempts", 5));
  options.cartesian_eef_step = getOrDeclareParameter<double>(
    node, "commander.cartesian_eef_step", 0.005);
  options.minimum_cartesian_fraction = getOrDeclareParameter<double>(
    node, "commander.minimum_cartesian_fraction", 0.95);
  return options;
}

struct TargetStatistics
{
  std::uint64_t successes{0};
  std::uint64_t failures{0};
  double total_duration_sec{0.0};
  double min_duration_sec{std::numeric_limits<double>::infinity()};
  double max_duration_sec{0.0};

  void addSuccess(double duration_sec)
  {
    ++successes;
    total_duration_sec += duration_sec;
    min_duration_sec = std::min(min_duration_sec, duration_sec);
    max_duration_sec = std::max(max_duration_sec, duration_sec);
  }

  void addFailure()
  {
    ++failures;
  }

  double averageDurationSec() const
  {
    return successes == 0U ? 0.0 : total_duration_sec / static_cast<double>(successes);
  }
};

struct TestStatistics
{
  SteadyClock::time_point test_start{SteadyClock::now()};
  std::uint64_t completed_cycles{0};
  std::uint64_t attempted_motions{0};
  std::uint64_t successful_motions{0};
  std::uint64_t failed_motions{0};
  std::array<TargetStatistics, 2> target_stats{};
  std::string stop_reason{"completed"};
  std::string failed_target{};
  std::int64_t failed_cycle{0};
  zhishu_robot_commander_cpp::Result failed_result{};
};

struct MotionAttempt
{
  zhishu_robot_commander_cpp::Result result;
  double duration_sec{0.0};
  bool interrupted{false};
};

struct MotionTarget
{
  std::string name;
  std::vector<double> joints;
};

std::vector<double> degreesToRadians(const std::vector<double> & degrees)
{
  constexpr double radians_per_degree = 3.14159265358979323846 / 180.0;
  auto radians = degrees;
  for (auto & angle : radians) {
    angle *= radians_per_degree;
  }
  return radians;
}

double elapsedSeconds(const SteadyClock::time_point & start)
{
  return std::chrono::duration<double>(SteadyClock::now() - start).count();
}

bool interruptibleSleep(double seconds)
{
  if (seconds <= 0.0) {
    return !shutdownRequested();
  }

  const auto deadline = SteadyClock::now() +
    std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<double>(seconds));
  while (!shutdownRequested() && SteadyClock::now() < deadline) {
    const auto remaining = deadline - SteadyClock::now();
    const auto chunk = std::min(
      remaining,
      std::chrono::duration_cast<SteadyClock::duration>(std::chrono::milliseconds(50)));
    if (chunk > SteadyClock::duration::zero()) {
      std::this_thread::sleep_for(chunk);
    }
  }
  return !shutdownRequested();
}

MotionAttempt executeMotion(
  const rclcpp::Node::SharedPtr & node,
  zhishu_robot_commander_cpp::RobotSdk & robot,
  const MotionTarget & target,
  std::int64_t cycle,
  std::int64_t loop_count)
{
  if (loop_count == 0) {
    RCLCPP_INFO(
      node->get_logger(),
      "[STABILITY][MOVE_START] cycle=%ld/infinite target=%s",
      static_cast<long>(cycle), target.name.c_str());
  } else {
    RCLCPP_INFO(
      node->get_logger(),
      "[STABILITY][MOVE_START] cycle=%ld/%ld target=%s",
      static_cast<long>(cycle), static_cast<long>(loop_count), target.name.c_str());
  }

  const auto start = SteadyClock::now();
  auto motion = robot.moveJointsAsync(
    zhishu_robot_commander_cpp::ArmGroup::kBoth, target.joints);

  while (!motion.done() && !shutdownRequested()) {
    motion.waitFor(50ms);
  }

  if (shutdownRequested()) {
    RCLCPP_WARN(
      node->get_logger(),
      "[STABILITY][SAFE_STOP_REQUESTED] cycle=%ld target=%s; cancelling active motion",
      static_cast<long>(cycle), target.name.c_str());

    if (!motion.done()) {
      motion.cancel();
      const auto cancel_deadline = SteadyClock::now() + 5s;
      while (!motion.done() && SteadyClock::now() < cancel_deadline) {
        motion.waitFor(50ms);
      }
    }

    const double duration_sec =
      std::chrono::duration<double>(SteadyClock::now() - start).count();
    if (motion.done()) {
      RCLCPP_WARN(
        node->get_logger(),
        "[STABILITY][SAFE_STOP_COMPLETE] cycle=%ld target=%s; "
        "trajectory cancellation acknowledged and controller left active",
        static_cast<long>(cycle), target.name.c_str());
      return MotionAttempt{motion.result(), duration_sec, true};
    }

    RCLCPP_ERROR(
      node->get_logger(),
      "[STABILITY][SAFE_STOP_TIMEOUT] cycle=%ld target=%s; "
      "cancellation was not acknowledged within 5 seconds",
      static_cast<long>(cycle), target.name.c_str());
    return MotionAttempt{
      zhishu_robot_commander_cpp::Result::failure(
        zhishu_robot_commander_cpp::ErrorCode::kTimeout,
        "timed out waiting for safe motion cancellation"),
      duration_sec,
      true};
  }

  const auto result = motion.result();
  const double duration_sec =
    std::chrono::duration<double>(SteadyClock::now() - start).count();

  if (result) {
    RCLCPP_INFO(
      node->get_logger(),
      "[STABILITY][MOVE_OK] cycle=%ld target=%s duration=%.3f s",
      static_cast<long>(cycle), target.name.c_str(), duration_sec);
  } else if (rclcpp::ok()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[STABILITY][MOVE_FAIL] cycle=%ld target=%s duration=%.3f s code=%s native=%d message=%s",
      static_cast<long>(cycle), target.name.c_str(), duration_sec,
      zhishu_robot_commander_cpp::toString(result.code), result.native_code,
      result.message.c_str());
  } else {
    RCLCPP_WARN(
      node->get_logger(),
      "[STABILITY][MOVE_INTERRUPTED] cycle=%ld target=%s duration=%.3f s",
      static_cast<long>(cycle), target.name.c_str(), duration_sec);
  }

  return MotionAttempt{result, duration_sec, false};
}

void printTestSummary(
  const rclcpp::Node::SharedPtr & node,
  const TestStatistics & statistics,
  const std::array<MotionTarget, 2> & targets,
  std::int64_t loop_count,
  double interval_sec,
  double velocity_scaling,
  double acceleration_scaling)
{
  const double elapsed_sec = elapsedSeconds(statistics.test_start);
  const double elapsed_hours = elapsed_sec / 3600.0;
  const std::string requested_cycles =
    loop_count == 0 ? "infinite" : std::to_string(loop_count);
  RCLCPP_INFO(
    node->get_logger(),
    "[STABILITY][SUMMARY] reason=%s runtime=%.1f s (%.3f h) completed_cycles=%lu "
    "requested_cycles=%s attempted_motions=%lu success=%lu failure=%lu interval=%.3f s "
    "velocity_scaling=%.3f acceleration_scaling=%.3f",
    statistics.stop_reason.c_str(), elapsed_sec, elapsed_hours,
    static_cast<unsigned long>(statistics.completed_cycles),
    requested_cycles.c_str(),
    static_cast<unsigned long>(statistics.attempted_motions),
    static_cast<unsigned long>(statistics.successful_motions),
    static_cast<unsigned long>(statistics.failed_motions), interval_sec,
    velocity_scaling, acceleration_scaling);

  for (std::size_t index = 0; index < targets.size(); ++index) {
    const auto & target_stats = statistics.target_stats[index];
    const double min_sec = target_stats.successes == 0U ? 0.0 : target_stats.min_duration_sec;
    RCLCPP_INFO(
      node->get_logger(),
      "[STABILITY][SUMMARY_TARGET] target=%s success=%lu failure=%lu avg=%.3f s min=%.3f s max=%.3f s",
      targets[index].name.c_str(), static_cast<unsigned long>(target_stats.successes),
      static_cast<unsigned long>(target_stats.failures), target_stats.averageDurationSec(),
      min_sec, target_stats.max_duration_sec);
  }

  if (statistics.failed_motions > 0U) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[STABILITY][SUMMARY_FAILURE] cycle=%ld target=%s code=%s native=%d message=%s",
      static_cast<long>(statistics.failed_cycle), statistics.failed_target.c_str(),
      zhishu_robot_commander_cpp::toString(statistics.failed_result.code),
      statistics.failed_result.native_code, statistics.failed_result.message.c_str());
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  stop_requested = 0;
  rclcpp::init(
    argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  if (std::signal(SIGINT, requestStop) == SIG_ERR ||
    std::signal(SIGTERM, requestStop) == SIG_ERR)
  {
    rclcpp::shutdown();
    return 1;
  }
  auto node = std::make_shared<rclcpp::Node>(
    "zhishu_robot_joint_ethercat_loop",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  int exit_code = 0;
  constexpr std::int64_t default_loop_count = 0;
  constexpr const char * sequence_description =
    "prepare_all_zero(unrecorded)->[configured_joint_pose->all_zero_return]";
  const std::vector<double> all_zero(14, 0.0);
  const auto left_pose_degrees = getOrDeclareParameter<std::vector<double>>(
    node, "left_target_joints",
    {-15.0, -13.0, -24.0, 20.0, 32.0, -27.0, 19.0});
  const auto right_pose_degrees = getOrDeclareParameter<std::vector<double>>(
    node, "right_target_joints",
    {15.0, 13.0, 24.0, 20.0, -32.0, 0.0, 19.0});
  std::vector<double> both_pose = degreesToRadians(left_pose_degrees);
  const auto right_pose_radians = degreesToRadians(right_pose_degrees);
  both_pose.insert(both_pose.end(), right_pose_radians.begin(), right_pose_radians.end());
  const MotionTarget preparation_target{"prepare_all_zero", all_zero};
  const std::array<MotionTarget, 2> targets{{
    {"configured_joint_pose", both_pose},
    {"all_zero_return", all_zero},
  }};
  TestStatistics statistics;
  std::unique_ptr<zhishu_robot_commander_cpp::StabilityPdoRecording> pdo_recording;

  try {
    const auto loop_count = getOrDeclareParameter<std::int64_t>(
      node, "loop_count", default_loop_count);
    const auto interval_sec = getOrDeclareParameter<double>(node, "interval_sec", 1.5);
    const auto record_pdo = getOrDeclareParameter<bool>(
      node, "record_pdo_diagnostics", true);
    const auto use_fake_hardware = getOrDeclareParameter<bool>(
      node, "use_fake_hardware", false);
    const auto pdo_record_cycles = getOrDeclareParameter<std::int64_t>(
      node, "pdo_record_cycles", 2);
    const auto pdo_service_timeout_sec = getOrDeclareParameter<double>(
      node, "pdo_service_timeout_sec", 10.0);
    const auto options = commanderOptions(node);

    if (loop_count < 0) {
      RCLCPP_ERROR(
        node->get_logger(), "loop_count must be >= 0 (0 means loop forever), got %ld",
        static_cast<long>(loop_count));
      statistics.stop_reason = "invalid_parameter";
      exit_code = 2;
    } else if (interval_sec < 0.0) {
      RCLCPP_ERROR(
        node->get_logger(), "interval_sec must be >= 0.0, got %.3f", interval_sec);
      statistics.stop_reason = "invalid_parameter";
      exit_code = 2;
    } else if (record_pdo && !use_fake_hardware &&
      (pdo_record_cycles <= 0 || pdo_service_timeout_sec <= 0.0))
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "pdo_record_cycles and pdo_service_timeout_sec must be positive when PDO "
        "recording is enabled; got %ld and %.3f",
        static_cast<long>(pdo_record_cycles), pdo_service_timeout_sec);
      statistics.stop_reason = "invalid_parameter";
      exit_code = 2;
    } else if (left_pose_degrees.size() != 7U || right_pose_degrees.size() != 7U) {
      RCLCPP_ERROR(
        node->get_logger(),
        "left_target_joints and right_target_joints must each contain 7 values; got %zu and %zu",
        left_pose_degrees.size(), right_pose_degrees.size());
      statistics.stop_reason = "invalid_parameter";
      exit_code = 2;
    } else {
      zhishu_robot_commander_cpp::RobotSdk robot(node, options);

      const auto state = robot.readState();
      if (!state.result) {
        RCLCPP_ERROR(
          node->get_logger(), "robot state is not ready [%s]: %s",
          zhishu_robot_commander_cpp::toString(state.result.code),
          state.result.message.c_str());
        statistics.stop_reason = "robot_not_ready";
        exit_code = 1;
      } else {
        RCLCPP_INFO(
          node->get_logger(),
          "[STABILITY][PREP_START] moving both arms to all-zero before PDO recording");
        const auto preparation = executeMotion(
          node, robot, preparation_target, 0, 1);
        bool preparation_ok = false;
        if (preparation.interrupted) {
          statistics.stop_reason = "user_shutdown";
          exit_code = 0;
        } else if (!preparation.result) {
          statistics.failed_motions = 1;
          statistics.failed_target = preparation_target.name;
          statistics.failed_cycle = 0;
          statistics.failed_result = preparation.result;
          statistics.stop_reason = "initial_position_failure";
          robot.cancelActiveMotion();
          exit_code = 1;
          RCLCPP_ERROR(
            node->get_logger(),
            "[STABILITY][PREP_FAIL] all-zero preparation failed; PDO recording will not start");
        } else {
          RCLCPP_INFO(
            node->get_logger(),
            "[STABILITY][PREP_OK] both arms are at all-zero; settling for %.3f s before "
            "PDO recording",
            interval_sec);
          preparation_ok = interruptibleSleep(interval_sec);
          if (!preparation_ok) {
            statistics.stop_reason = "user_shutdown";
          }
        }

        if (preparation_ok) {
          if (record_pdo && !use_fake_hardware) {
            const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(pdo_service_timeout_sec));
            pdo_recording =
              std::make_unique<zhishu_robot_commander_cpp::StabilityPdoRecording>(
            node, timeout);
            if (!pdo_recording->start()) {
              statistics.stop_reason = "pdo_recording_start_failure";
              throw std::runtime_error(
                    "both PDO recorders must start before the real-hardware stability test");
            }
          }

          statistics.test_start = SteadyClock::now();
          RCLCPP_INFO(
          node->get_logger(),
          "[STABILITY][TEST_START] sequence=%s loop_count=%ld%s interval_sec=%.3f "
          "velocity_scaling=%.3f acceleration_scaling=%.3f",
          sequence_description,
          static_cast<long>(loop_count), loop_count == 0 ? " (infinite)" : "",
          interval_sec, options.velocity_scaling, options.acceleration_scaling);

          std::int64_t cycle = 1;
          while (!shutdownRequested() && (loop_count == 0 || cycle <= loop_count)) {
            const auto cycle_start = SteadyClock::now();
            if (loop_count == 0) {
              RCLCPP_INFO(
              node->get_logger(), "[STABILITY][CYCLE_START] cycle=%ld/infinite",
              static_cast<long>(cycle));
            } else {
              RCLCPP_INFO(
              node->get_logger(), "[STABILITY][CYCLE_START] cycle=%ld/%ld",
              static_cast<long>(cycle), static_cast<long>(loop_count));
            }

            bool cycle_ok = true;
            for (std::size_t target_index = 0; target_index < targets.size(); ++target_index) {
              if (shutdownRequested()) {
                cycle_ok = false;
                statistics.stop_reason = "user_shutdown";
                break;
              }

              ++statistics.attempted_motions;
              const auto attempt = executeMotion(
              node, robot, targets[target_index], cycle, loop_count);

              if (attempt.interrupted) {
                cycle_ok = false;
                statistics.stop_reason = "user_shutdown";
                exit_code = 0;
                break;
              }

              if (!attempt.result) {
                cycle_ok = false;
                if (!shutdownRequested()) {
                  ++statistics.failed_motions;
                  statistics.target_stats[target_index].addFailure();
                  statistics.stop_reason = "motion_failure";
                  statistics.failed_target = targets[target_index].name;
                  statistics.failed_cycle = cycle;
                  statistics.failed_result = attempt.result;
                  robot.cancelActiveMotion();
                  exit_code = 1;
                } else {
                  statistics.stop_reason = "user_shutdown";
                  exit_code = 0;
                }
                break;
              }

              ++statistics.successful_motions;
              statistics.target_stats[target_index].addSuccess(attempt.duration_sec);

              const bool has_next_target_in_cycle = target_index + 1 < targets.size();
              const bool has_next_cycle = loop_count == 0 || cycle < loop_count;
              const bool has_next_action = has_next_target_in_cycle || has_next_cycle;
              if (has_next_action && interval_sec > 0.0) {
                RCLCPP_INFO(
                node->get_logger(),
                "[STABILITY][INTERVAL] waiting %.3f s before next motion", interval_sec);
                if (!interruptibleSleep(interval_sec)) {
                  cycle_ok = false;
                  statistics.stop_reason = "user_shutdown";
                  break;
                }
              }
            }

            if (!cycle_ok) {
              break;
            }

            ++statistics.completed_cycles;
            const double cycle_duration_sec =
              std::chrono::duration<double>(SteadyClock::now() - cycle_start).count();
            RCLCPP_INFO(
            node->get_logger(),
            "[STABILITY][CYCLE_OK] cycle=%ld duration=%.3f s total_runtime=%.1f s "
            "success=%lu failure=%lu",
            static_cast<long>(cycle), cycle_duration_sec,
            elapsedSeconds(statistics.test_start),
            static_cast<unsigned long>(statistics.successful_motions),
            static_cast<unsigned long>(statistics.failed_motions));

            if (pdo_recording && pdo_recording->active() &&
              statistics.completed_cycles >= static_cast<std::uint64_t>(pdo_record_cycles))
            {
              if (!pdo_recording->stop()) {
                statistics.stop_reason = "pdo_recording_stop_failure";
                exit_code = 1;
                break;
              }
            }
            ++cycle;
          }

          if (shutdownRequested() && statistics.stop_reason == "completed") {
            statistics.stop_reason = "user_shutdown";
          }
          if (statistics.stop_reason == "completed" && loop_count > 0 &&
            statistics.completed_cycles == static_cast<std::uint64_t>(loop_count))
          {
            RCLCPP_INFO(
            node->get_logger(), "[STABILITY][TEST_OK] all %ld cycles completed",
            static_cast<long>(loop_count));
          }

          printTestSummary(
          node, statistics, targets, loop_count, interval_sec,
          options.velocity_scaling, options.acceleration_scaling);
        }
      }
    }
  } catch (const std::exception & exception) {
    if (statistics.stop_reason == "completed") {
      statistics.stop_reason = "exception";
    }
    RCLCPP_FATAL(node->get_logger(), "joint EtherCAT loop test failed: %s", exception.what());
    exit_code = 1;
  }

  pdo_recording.reset();
  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return exit_code;
}
