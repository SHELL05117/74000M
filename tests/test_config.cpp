#include "robot/config/robot_config.hpp"
#include "test_framework.hpp"

#include <limits>

namespace {

robot::RobotConfig validConfig() {
  robot::RobotConfig config{};
  config.identity = {"74000", "74000M-A", "74000M", "test", 1, 1, 1};
  config.hardware.left = {{{1, false, 200}, {2, false, 200}, {3, false, 200}}};
  config.hardware.right =
      {{{4, false, 200}, {5, false, 200}, {6, false, 200}}};
  config.hardware.imu = {true, 7};
  config.geometry = {0.08255, 0.30};
  config.calibration = {0.013, 0.013, 0.30};
  config.hardware_verification = robot::VerificationLevel::HILValidated;
  return config;
}

}  // namespace

ROBOT_TEST("1690X commissioning profile is structurally valid but locked") {
  const auto config = robot::makeOfflineRobotConfig();
  const auto check = robot::validateConfig(config, "1690X", 3);
  ROBOT_REQUIRE(check.structurally_valid);
  ROBOT_REQUIRE(!check.output_unlock_allowed);
  ROBOT_REQUIRE((check.fault_bits & robot::kHardwareUnverified) != 0);
  ROBOT_REQUIRE((check.fault_bits & robot::kBadPort) == 0);
  ROBOT_REQUIRE(
      robot::boundedStringEqual(config.identity.team_number, "1690X", 8));
  ROBOT_REQUIRE(
      robot::boundedStringEqual(config.identity.robot_id, "1690X", 16));
  ROBOT_REQUIRE(!config.hardware.imu.installed);
  ROBOT_REQUIRE(config.hardware.lift.installed);
}

ROBOT_TEST("offline profile preserves supplied six-motor drivetrain facts") {
  const auto config = robot::makeOfflineRobotConfig();

  ROBOT_REQUIRE(config.hardware.left[0].smart_port == 3);
  ROBOT_REQUIRE(config.hardware.left[0].cartridge_rpm == 600);
  ROBOT_REQUIRE(config.hardware.left[0].reversed);
  ROBOT_REQUIRE_NEAR(config.hardware.left[0].motor_rev_per_wheel_rev,
                     4.0 / 3.0, 1e-12);
  ROBOT_REQUIRE(config.hardware.left[1].smart_port == 2);
  ROBOT_REQUIRE(config.hardware.left[1].cartridge_rpm == 200);
  ROBOT_REQUIRE(config.hardware.left[1].reversed);
  ROBOT_REQUIRE_NEAR(config.hardware.left[1].motor_rev_per_wheel_rev,
                     4.0 / 9.0, 1e-12);
  ROBOT_REQUIRE(config.hardware.left[2].smart_port == 1);
  ROBOT_REQUIRE(config.hardware.left[2].cartridge_rpm == 600);
  ROBOT_REQUIRE(!config.hardware.left[2].reversed);

  ROBOT_REQUIRE(config.hardware.right[0].smart_port == 13);
  ROBOT_REQUIRE(config.hardware.right[0].cartridge_rpm == 600);
  ROBOT_REQUIRE(!config.hardware.right[0].reversed);
  ROBOT_REQUIRE(config.hardware.right[1].smart_port == 12);
  ROBOT_REQUIRE(config.hardware.right[1].cartridge_rpm == 200);
  ROBOT_REQUIRE(!config.hardware.right[1].reversed);
  ROBOT_REQUIRE(config.hardware.right[2].smart_port == 11);
  ROBOT_REQUIRE(config.hardware.right[2].cartridge_rpm == 600);
  ROBOT_REQUIRE(config.hardware.right[2].reversed);

  ROBOT_REQUIRE(config.hardware.lift.motors[0].smart_port == 14);
  ROBOT_REQUIRE(config.hardware.lift.motors[0].cartridge_rpm == 200);
  ROBOT_REQUIRE(config.hardware.lift.motors[0].reversed);
  ROBOT_REQUIRE(config.hardware.lift.motors[1].smart_port == 5);
  ROBOT_REQUIRE(config.hardware.lift.motors[1].cartridge_rpm == 200);
  ROBOT_REQUIRE(!config.hardware.lift.motors[1].reversed);
  ROBOT_REQUIRE_NEAR(config.hardware.lift.minimum_position_rad, 0.0, 0.0);
  ROBOT_REQUIRE_NEAR(config.hardware.lift.maximum_position_rad,
                     robot::units::degreesToRadians(830.0), 1e-12);

  for (const auto& motor : config.hardware.left)
    ROBOT_REQUIRE_NEAR(robot::nominalWheelRpm(motor), 450.0, 1e-9);
  for (const auto& motor : config.hardware.right)
    ROBOT_REQUIRE_NEAR(robot::nominalWheelRpm(motor), 450.0, 1e-9);

  ROBOT_REQUIRE(!config.capabilities.hardware_output);
  ROBOT_REQUIRE(config.hardware_verification ==
                robot::VerificationLevel::Implemented);
  ROBOT_REQUIRE_NEAR(config.electrical.max_command_voltage_V, 12.0, 1e-12);
}

ROBOT_TEST("missing odometry calibration is allowed only while pose is locked") {
  auto config = robot::make1690XCommissioningConfig();
  auto locked = robot::validateConfig(config, "1690X", 3);
  ROBOT_REQUIRE(locked.structurally_valid);
  config.capabilities.hardware_output = true;
  config.capabilities.pose_good = true;
  config.hardware_verification = robot::VerificationLevel::HILValidated;
  const auto unlocked = robot::validateConfig(config, "1690X", 3);
  ROBOT_REQUIRE((unlocked.fault_bits & robot::kBadCalibration) != 0);
}

ROBOT_TEST("valid but capability-locked configuration cannot output") {
  const auto config = validConfig();
  const auto check = robot::validateConfig(config, "74000M-A", 1);
  ROBOT_REQUIRE(check.structurally_valid);
  ROBOT_REQUIRE(!check.output_unlock_allowed);
  ROBOT_REQUIRE(check.fault_bits == robot::kConfigOk);
}

ROBOT_TEST("wrong robot identity schema and duplicate ports are rejected") {
  auto config = validConfig();
  config.identity.config_schema = 2;
  config.hardware.right[0].smart_port = 1;
  const auto check = robot::validateConfig(config, "74000M-B", 1);
  ROBOT_REQUIRE(!check.structurally_valid);
  ROBOT_REQUIRE((check.fault_bits & robot::kWrongRobotId) != 0);
  ROBOT_REQUIRE((check.fault_bits & robot::kBadSchema) != 0);
  ROBOT_REQUIRE((check.fault_bits & robot::kDuplicatePort) != 0);
  ROBOT_REQUIRE(!check.output_unlock_allowed);
}

ROBOT_TEST("nonfinite geometry is rejected before output unlock") {
  auto config = validConfig();
  config.geometry.nominal_track_width_m =
      std::numeric_limits<double>::quiet_NaN();
  config.capabilities.hardware_output = true;
  const auto check = robot::validateConfig(config, "74000M-A", 1);
  ROBOT_REQUIRE((check.fault_bits & robot::kBadGeometry) != 0);
  ROBOT_REQUIRE(!check.output_unlock_allowed);
}

ROBOT_TEST("capability dependencies and DoNothing fallback are enforced") {
  auto config = validConfig();
  config.capabilities.driver_control = true;
  config.selected_route = 0x0101;
  const auto check = robot::validateConfig(config, "74000M-A", 1);
  ROBOT_REQUIRE((check.fault_bits & robot::kCapabilityViolation) != 0);
  ROBOT_REQUIRE((check.fault_bits & robot::kBadRoute) != 0);
}

ROBOT_TEST("hardware output unlock requires both evidence and explicit capability") {
  auto config = validConfig();
  config.capabilities.hardware_output = true;
  const auto check = robot::validateConfig(config, "74000M-A", 1);
  ROBOT_REQUIRE(check.structurally_valid);
  ROBOT_REQUIRE(check.output_unlock_allowed);

  config.hardware_verification = robot::VerificationLevel::SimValidated;
  const auto simulated = robot::validateConfig(config, "74000M-A", 1);
  ROBOT_REQUIRE(!simulated.output_unlock_allowed);
  ROBOT_REQUIRE((simulated.fault_bits & robot::kHardwareUnverified) != 0);
}
