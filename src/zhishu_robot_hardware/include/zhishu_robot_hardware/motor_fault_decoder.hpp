#ifndef ZHISHU_ROBOT_HARDWARE__MOTOR_FAULT_DECODER_HPP_
#define ZHISHU_ROBOT_HARDWARE__MOTOR_FAULT_DECODER_HPP_

#include <array>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace zhishu_robot_hardware
{

struct MotorFaultBit
{
  uint16_t mask;
  uint8_t bit;
  const char * name;
  const char * description;
};

inline const std::array<MotorFaultBit, 16> & motor_fault_definitions()
{
  static const std::array<MotorFaultBit, 16> definitions = {{
    {0x0001U, 0U, "FAULT_MOTOR_OVER_TEMPERATURE", "电机线圈过温"},
    {0x0002U, 1U, "FAULT_DRIVER_CHIP", "驱动芯片故障"},
    {0x0004U, 2U, "FAULT_UNDER_VOLTAGE", "欠压"},
    {0x0008U, 3U, "FAULT_OVER_VOLTAGE", "过压"},
    {0x0010U, 4U, "FAULT_OVER_CURRENT", "过流"},
    {0x0020U, 5U, "FAULT_PHASE_LOSS", "缺相"},
    {0x0040U, 6U, "FAULT_BOARD_OVER_TEMPERATURE", "驱动器温度过高"},
    {0x0080U, 7U, "FAULT_OVER_LOAD", "过载"},
    {0x0100U, 8U, "FAULT_MOTOR_STUCK", "电机堵转"},
    {0x0200U, 9U, "FAULT_ENCODER_MOTOR", "电机端编码器异常"},
    {0x0400U, 10U, "FAULT_ENCODER_LOAD", "负载端编码器异常"},
    {0x0800U, 11U, "FAULT_POSITION_FEEDBACK", "位置反馈超限"},
    {0x1000U, 12U, "FAULT_VELOCITY_FEEDBACK", "速度反馈超限"},
    {0x2000U, 13U, "FAULT_POSITION_ERROR", "位置误差超限"},
    {0x4000U, 14U, "FAULT_VELOCITY_ERROR", "速度误差超限"},
    {0x8000U, 15U, "FAULT_OTHERS", "其他故障（包括电气角度未标定等）"},
  }};
  return definitions;
}

// The vendor defines 0x603F as a 16-bit bit mask, so every set bit must be
// reported. Do not decode it as a single enumerated error value.
inline std::string format_motor_fault_mask(uint16_t error_code)
{
  if (error_code == 0U) {
    return "none";
  }

  std::ostringstream stream;
  bool first = true;
  for (const auto & fault : motor_fault_definitions()) {
    if ((error_code & fault.mask) == 0U) {
      continue;
    }
    if (!first) {
      stream << "; ";
    }
    first = false;
    stream << "bit" << static_cast<unsigned int>(fault.bit)
           << "(0x" << std::uppercase << std::hex << std::setw(4)
           << std::setfill('0') << static_cast<unsigned int>(fault.mask)
           << std::dec << ") " << fault.name << ": " << fault.description;
  }
  return stream.str();
}

}  // namespace zhishu_robot_hardware

#endif  // ZHISHU_ROBOT_HARDWARE__MOTOR_FAULT_DECODER_HPP_
