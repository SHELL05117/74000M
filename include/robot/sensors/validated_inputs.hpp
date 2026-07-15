#pragma once

#include <array>
#include <cstdint>

#include "robot/config/robot_config.hpp"
#include "robot/core/fault.hpp"
#include "robot/core/frame.hpp"
#include "robot/core/quality.hpp"
#include "robot/state/raw_inputs.hpp"

namespace robot {

enum SensorReject : std::uint32_t {
  kSensorAccepted = 0,
  kSensorApiError = 1u << 0,
  kSensorNonfinite = 1u << 1,
  kSensorFutureTimestamp = 1u << 2,
  kSensorStale = 1u << 3,
  kSensorNonmonotonic = 1u << 4,
  kSensorOutOfRange = 1u << 5,
  kSensorImplausibleRate = 1u << 6,
  kSensorFrozen = 1u << 7,
  kSensorDeviceStatus = 1u << 8,
  kSensorWarmup = 1u << 9,
  kSensorCalibrating = 1u << 10,
  kSensorNotConfigured = 1u << 11,
  kSensorFrameRejected = 1u << 12,
};

struct CheckedScalar {
  double value{};
  TimeUs sample_time_us{};
  Quality quality{Quality::Invalid};
  std::uint32_t reject_bits{};
};

struct ValidatedMotor {
  std::uint8_t smart_port{};
  CheckedScalar position_rad;
  CheckedScalar velocity_radps;
  CheckedScalar current_A;
  CheckedScalar temperature_C;
  CheckedScalar applied_voltage_V;
  Quality kinematic_quality{Quality::Invalid};
  Quality health_quality{Quality::Invalid};
  std::uint32_t device_faults{};
  std::uint32_t reject_bits{};
};

struct ValidatedTrackingWheel {
  CheckedScalar position_rad;
  CheckedScalar velocity_radps;
  bool configured{};
};

struct ValidatedImu {
  CheckedScalar rotation_rad;
  CheckedScalar yaw_rate_radps;
  bool calibrating{};
};

struct ValidatedInputs {
  FrameHeader h{};
  TimeUs acquisition_end_us{};
  std::array<ValidatedMotor, kMotorsPerSide> left{};
  std::array<ValidatedMotor, kMotorsPerSide> right{};
  std::array<ValidatedTrackingWheel, kMaxTrackingWheels> tracking{};
  ValidatedImu imu{};
  CheckedScalar battery_V{};
  FaultBits fault_bits{};
  std::uint32_t frame_reject_bits{};
};

}  // namespace robot
