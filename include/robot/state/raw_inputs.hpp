#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "robot/config/robot_config.hpp"
#include "robot/core/frame.hpp"

namespace robot {

constexpr std::size_t kMaxTrackingWheels = 3;

struct ScalarSample {
  double value{};
  TimeUs sample_time_us{};
  bool api_ok{};
  std::uint32_t device_status{};
};

struct MotorSample {
  std::uint8_t smart_port{};
  ScalarSample position_rad;
  ScalarSample velocity_radps;
  ScalarSample current_A;
  ScalarSample temperature_C;
  ScalarSample applied_voltage_V;
  std::uint32_t faults{};
  bool faults_api_ok{};
};

template <std::size_t N>
struct MotorSideRaw {
  std::array<MotorSample, N> motor{};
};

struct TrackingWheelRaw {
  ScalarSample position_rad;
  ScalarSample velocity_radps;
  bool configured{};
};

struct ImuRaw {
  ScalarSample rotation_rad;
  ScalarSample yaw_rate_radps;
  bool calibrating{};
  bool status_api_ok{};
};

struct RawDriveInputs {
  FrameHeader h;
  TimeUs acquisition_end_us{};
  MotorSideRaw<kMotorsPerSide> left;
  MotorSideRaw<kMotorsPerSide> right;
  std::array<MotorSample, kLiftMotorCount> lift{};
  std::array<TrackingWheelRaw, kMaxTrackingWheels> tracking{};
  ImuRaw imu;
  ScalarSample battery_V;
};

}  // namespace robot
