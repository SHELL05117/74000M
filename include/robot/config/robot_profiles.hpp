#pragma once

#include <cstdint>

#include "robot/config/robot_config.hpp"

namespace robot {

enum class RobotProfile : std::uint8_t {
  k492X,
  k492Z,
};

inline constexpr std::uint32_t kRobotConfigSchema = 4;
inline constexpr double kRatio6MotorRevPerWheelRev = 48.0 / 36.0;
inline constexpr double kRatio18MotorRevPerWheelRev =
    (12.0 / 36.0) * (48.0 / 36.0);

inline RobotConfig makeSharedRobotConfig(RobotIdentity identity,
                                         HardwareConfig hardware) {
  RobotConfig config{};
  config.identity = identity;
  config.hardware = hardware;
  config.hardware.imu = {false, 0};
  config.geometry = {0.06985, 0.1524};
  // CAD/nominal values are not promoted to fitted odometry calibration.
  config.calibration = {};
  // Operator-authorized full-voltage commissioning ceiling. Input/output
  // slew and all commissioning safety gates remain active.
  config.electrical.max_command_voltage_V = 12.0;
  config.hardware_verification = VerificationLevel::Implemented;
  config.selected_route = RouteIds::kDoNothing;
  return config;
}

inline HardwareConfig make492XHardwareConfig() {
  HardwareConfig hardware{};
  hardware.left = {{
      motorFromReversedFlag(2, 600, true,
                            kRatio6MotorRevPerWheelRev),   // Front: reverse
      motorFromReversedFlag(3, 200, true,
                            kRatio18MotorRevPerWheelRev),  // Middle: reverse
      motorFromReversedFlag(1, 600, false,
                            kRatio6MotorRevPerWheelRev),   // Rear: forward
  }};
  hardware.right = {{
      motorFromReversedFlag(14, 600, false,
                            kRatio6MotorRevPerWheelRev),   // Front: forward
      motorFromReversedFlag(13, 200, false,
                            kRatio18MotorRevPerWheelRev),  // Middle: forward
      motorFromReversedFlag(11, 600, true,
                            kRatio6MotorRevPerWheelRev),   // Rear: reverse
  }};
  hardware.lift = {
      true,
      {{motorFromReversedFlag(19, 200, true),
        motorFromReversedFlag(18, 200, false)}},
      0.0,
      units::degreesToRadians(830.0)};
  return hardware;
}

inline HardwareConfig make492ZHardwareConfig() {
  HardwareConfig hardware{};
  hardware.left = {{
      motorFromReversedFlag(3, 600, true,
                            kRatio6MotorRevPerWheelRev),   // Front: reverse
      motorFromReversedFlag(2, 200, true,
                            kRatio18MotorRevPerWheelRev),  // Middle: reverse
      motorFromReversedFlag(1, 600, false,
                            kRatio6MotorRevPerWheelRev),   // Rear: forward
  }};
  hardware.right = {{
      motorFromReversedFlag(13, 600, false,
                            kRatio6MotorRevPerWheelRev),   // Front: forward
      motorFromReversedFlag(12, 200, false,
                            kRatio18MotorRevPerWheelRev),  // Middle: forward
      motorFromReversedFlag(11, 600, true,
                            kRatio6MotorRevPerWheelRev),   // Rear: reverse
  }};
  hardware.lift = {
      true,
      {{motorFromReversedFlag(14, 200, true),
        motorFromReversedFlag(5, 200, false)}},
      0.0,
      units::degreesToRadians(830.0)};
  return hardware;
}

inline RobotConfig make492XRobotConfig() {
  return makeSharedRobotConfig(
      {"492X", "492X", "492X ROBOT", "commission", 0,
       kRobotConfigSchema, 0},
      make492XHardwareConfig());
}

inline RobotConfig make492ZRobotConfig() {
  return makeSharedRobotConfig(
      {"492Z", "492Z", "492Z ROBOT", "commission", 0,
       kRobotConfigSchema, 0},
      make492ZHardwareConfig());
}

#if defined(ROBOT_PROFILE_492X) && defined(ROBOT_PROFILE_492Z)
#error "Select exactly one robot profile"
#elif defined(ROBOT_PROFILE_492X)
inline constexpr RobotProfile kSelectedRobotProfile = RobotProfile::k492X;
#elif defined(ROBOT_PROFILE_492Z)
inline constexpr RobotProfile kSelectedRobotProfile = RobotProfile::k492Z;
#else
#error "Define ROBOT_PROFILE_492X or ROBOT_PROFILE_492Z"
#endif

inline constexpr const char* selectedRobotId() noexcept {
  return kSelectedRobotProfile == RobotProfile::k492X ? "492X" : "492Z";
}

inline RobotConfig makeSelectedRobotConfig() {
  return kSelectedRobotProfile == RobotProfile::k492X
             ? make492XRobotConfig()
             : make492ZRobotConfig();
}

// Compatibility name for PC fixtures that need the build-selected profile.
inline RobotConfig makeOfflineRobotConfig() {
  return makeSelectedRobotConfig();
}

}  // namespace robot
