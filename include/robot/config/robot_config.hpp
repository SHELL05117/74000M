#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "robot/config/robot_identity.hpp"
#include "robot/core/frame.hpp"
#include "robot/ui/registry_ids.hpp"

namespace robot {

constexpr std::size_t kMotorsPerSide = 3;

struct MotorPortConfig {
  std::int8_t smart_port{};
  bool reversed{};
  std::uint16_t cartridge_rpm{};
  // Motor output-shaft revolutions required for one drive-wheel revolution.
  // This is per motor because mixed V5 cartridges can use different external
  // gear trains while producing the same wheel speed.
  double motor_rev_per_wheel_rev{1.0};
};

// The supplied wiring notation uses true=forward. The HAL stores whether the
// motor is reversed, so the two flags intentionally have opposite polarity.
constexpr MotorPortConfig motorFromForwardFlag(
    std::int8_t smart_port, std::uint16_t cartridge_rpm,
    bool forward_positive,
    double motor_rev_per_wheel_rev = 1.0) noexcept {
  return {smart_port, !forward_positive, cartridge_rpm,
          motor_rev_per_wheel_rev};
}

struct ImuConfig {
  bool installed{};
  std::uint8_t smart_port{};
};

struct RotationSensorConfig {
  bool installed{};
  std::uint8_t smart_port{};
  bool reversed{};
  double offset_m{};
};

struct HardwareConfig {
  std::array<MotorPortConfig, kMotorsPerSide> left{};
  std::array<MotorPortConfig, kMotorsPerSide> right{};
  ImuConfig imu{};
  RotationSensorConfig parallel_rotation{};
  RotationSensorConfig lateral_rotation{};
};

struct GeometryConfig {
  double nominal_drive_wheel_diameter_m{};
  double nominal_track_width_m{};
};

struct CalibrationConfig {
  double left_m_per_motor_rad{};
  double right_m_per_motor_rad{};
  double effective_track_width_m{};
};

struct RuntimeConfig {
  double nominal_period_s{0.010};
  double min_math_dt_s{0.001};
  double max_math_dt_s{0.050};
  TimeUs request_ttl_us{40000};
  TimeUs output_ttl_us{40000};
};

struct ElectricalConfig {
  double max_command_voltage_V{12.0};
};

struct RobotCapabilities {
  bool hardware_output{};
  bool driver_control{};
  bool pose_good{};
  bool autonomous_chassis_velocity{};
  bool autonomous_motion{};
  bool competition_routes{};
};

struct RobotConfig {
  RobotIdentity identity{};
  HardwareConfig hardware{};
  GeometryConfig geometry{};
  CalibrationConfig calibration{};
  RuntimeConfig runtime{};
  ElectricalConfig electrical{};
  VerificationLevel hardware_verification{VerificationLevel::Unverified};
  RobotCapabilities capabilities{};
  RouteId selected_route{RouteIds::kDoNothing};
};

enum ConfigFault : std::uint32_t {
  kConfigOk = 0,
  kBadIdentity = 1u << 0,
  kWrongRobotId = 1u << 1,
  kBadSchema = 1u << 2,
  kHardwareUnverified = 1u << 3,
  kBadPort = 1u << 4,
  kDuplicatePort = 1u << 5,
  kBadCartridge = 1u << 6,
  kBadGeometry = 1u << 7,
  kBadCalibration = 1u << 8,
  kBadRuntime = 1u << 9,
  kBadVoltageLimit = 1u << 10,
  kCapabilityViolation = 1u << 11,
  kBadRoute = 1u << 12,
  kBadTransmission = 1u << 13,
};

struct ConfigCheck {
  bool structurally_valid{};
  bool output_unlock_allowed{};
  std::uint32_t fault_bits{};
};

inline bool knownCartridge(std::uint16_t rpm) noexcept {
  return rpm == 100 || rpm == 200 || rpm == 600;
}

inline bool finitePositive(double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}

inline double nominalWheelRpm(const MotorPortConfig& motor) noexcept {
  return finitePositive(motor.motor_rev_per_wheel_rev)
             ? static_cast<double>(motor.cartridge_rpm) /
                   motor.motor_rev_per_wheel_rev
             : 0.0;
}

inline bool capabilityChainValid(const RobotCapabilities& capabilities) {
  if (capabilities.driver_control && !capabilities.hardware_output) return false;
  if (capabilities.pose_good && !capabilities.hardware_output) return false;
  if (capabilities.autonomous_chassis_velocity &&
      (!capabilities.hardware_output || !capabilities.pose_good)) {
    return false;
  }
  if (capabilities.autonomous_motion &&
      !capabilities.autonomous_chassis_velocity) {
    return false;
  }
  if (capabilities.competition_routes && !capabilities.autonomous_motion)
    return false;
  return true;
}

inline ConfigCheck validateConfig(const RobotConfig& config,
                                  const char* expected_robot_id,
                                  std::uint32_t expected_schema) {
  ConfigCheck result{};
  std::array<bool, 22> used_ports{};

  if (!boundedNonEmptyString(config.identity.team_number, 8) ||
      !boundedNonEmptyString(config.identity.robot_id, 16) ||
      !boundedNonEmptyString(config.identity.robot_name, 20) ||
      !boundedNonEmptyString(config.identity.software_version, 16)) {
    result.fault_bits |= kBadIdentity;
  }
  if (!boundedStringEqual(config.identity.robot_id, expected_robot_id, 16))
    result.fault_bits |= kWrongRobotId;
  if (config.identity.config_schema == 0 ||
      config.identity.config_schema != expected_schema)
    result.fault_bits |= kBadSchema;

  auto claim_port = [&](std::uint8_t port) {
    if (port < 1 || port > 21) {
      result.fault_bits |= kBadPort;
      return;
    }
    if (used_ports[port]) result.fault_bits |= kDuplicatePort;
    used_ports[port] = true;
  };

  for (const auto& motor : config.hardware.left) {
    claim_port(static_cast<std::uint8_t>(motor.smart_port));
    if (!knownCartridge(motor.cartridge_rpm))
      result.fault_bits |= kBadCartridge;
    if (!finitePositive(motor.motor_rev_per_wheel_rev))
      result.fault_bits |= kBadTransmission;
  }
  for (const auto& motor : config.hardware.right) {
    claim_port(static_cast<std::uint8_t>(motor.smart_port));
    if (!knownCartridge(motor.cartridge_rpm))
      result.fault_bits |= kBadCartridge;
    if (!finitePositive(motor.motor_rev_per_wheel_rev))
      result.fault_bits |= kBadTransmission;
  }
  if (config.hardware.imu.installed)
    claim_port(config.hardware.imu.smart_port);
  if (config.hardware.parallel_rotation.installed)
    claim_port(config.hardware.parallel_rotation.smart_port);
  if (config.hardware.lateral_rotation.installed)
    claim_port(config.hardware.lateral_rotation.smart_port);

  if (!finitePositive(config.geometry.nominal_drive_wheel_diameter_m) ||
      !finitePositive(config.geometry.nominal_track_width_m)) {
    result.fault_bits |= kBadGeometry;
  }
  const bool calibration_required =
      config.capabilities.pose_good ||
      config.capabilities.autonomous_chassis_velocity ||
      config.capabilities.autonomous_motion ||
      config.capabilities.competition_routes;
  if (calibration_required &&
      (!finitePositive(config.calibration.left_m_per_motor_rad) ||
       !finitePositive(config.calibration.right_m_per_motor_rad) ||
       !finitePositive(config.calibration.effective_track_width_m))) {
    result.fault_bits |= kBadCalibration;
  }
  if (!finitePositive(config.runtime.nominal_period_s) ||
      !finitePositive(config.runtime.min_math_dt_s) ||
      !finitePositive(config.runtime.max_math_dt_s) ||
      config.runtime.min_math_dt_s > config.runtime.nominal_period_s ||
      config.runtime.max_math_dt_s < config.runtime.nominal_period_s ||
      config.runtime.request_ttl_us == 0 || config.runtime.output_ttl_us == 0) {
    result.fault_bits |= kBadRuntime;
  }
  if (!finitePositive(config.electrical.max_command_voltage_V) ||
      config.electrical.max_command_voltage_V > 12.0) {
    result.fault_bits |= kBadVoltageLimit;
  }

  const bool hardware_verified = atLeast(
      config.hardware_verification, VerificationLevel::HILValidated);
  if (!hardware_verified) result.fault_bits |= kHardwareUnverified;
  if (!capabilityChainValid(config.capabilities) ||
      (config.capabilities.hardware_output && !hardware_verified)) {
    result.fault_bits |= kCapabilityViolation;
  }
  if (!config.capabilities.competition_routes &&
      config.selected_route != RouteIds::kDoNothing) {
    result.fault_bits |= kBadRoute;
  }

  constexpr std::uint32_t kNonStructural = kHardwareUnverified;
  result.structurally_valid =
      (result.fault_bits & ~kNonStructural) == kConfigOk;
  result.output_unlock_allowed =
      result.structurally_valid && hardware_verified &&
      config.capabilities.hardware_output;
  return result;
}

inline RobotConfig make1690XCommissioningConfig() {
  RobotConfig config{};
  constexpr double kRatio6MotorRevPerWheelRev = 48.0 / 36.0;
  constexpr double kRatio18MotorRevPerWheelRev =
      (12.0 / 36.0) * (48.0 / 36.0);

  config.identity = {"1690X", "1690X", "1690X SAMPLE", "commission", 0,
                     2, 0};
  config.hardware.left = {{
      motorFromForwardFlag(11, 600, false,
                           kRatio6MotorRevPerWheelRev),
      motorFromForwardFlag(12, 200, true,
                           kRatio18MotorRevPerWheelRev),
      motorFromForwardFlag(13, 600, true,
                           kRatio6MotorRevPerWheelRev),
  }};
  config.hardware.right = {{
      motorFromForwardFlag(1, 600, true,
                           kRatio6MotorRevPerWheelRev),
      motorFromForwardFlag(2, 200, false,
                           kRatio18MotorRevPerWheelRev),
      motorFromForwardFlag(3, 600, false,
                           kRatio6MotorRevPerWheelRev),
  }};
  config.hardware.imu = {false, 0};
  config.geometry = {0.06985, 0.1524};
  // CAD/nominal values are not promoted to fitted odometry calibration.
  config.calibration = {};
  // Initial commissioning build is physically capped at 4 V.
  config.electrical.max_command_voltage_V = 4.0;
  config.hardware_verification = VerificationLevel::Implemented;
  config.selected_route = RouteIds::kDoNothing;
  return config;
}

// Kept as a compatibility entry point for existing callers. It now returns
// the identified, capability-locked 1690X commissioning profile.
inline RobotConfig makeOfflineRobotConfig() {
  return make1690XCommissioningConfig();
}

}  // namespace robot
