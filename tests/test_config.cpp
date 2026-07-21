#include "robot/config/robot_config.hpp"
#include "robot/config/robot_profiles.hpp"
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

ROBOT_TEST("492X and 492Z profiles are structurally valid but locked") {
  const auto check_profile = [](const robot::RobotConfig& config,
                                const char* expected_id) {
    const auto check = robot::validateConfig(
        config, expected_id, robot::kRobotConfigSchema);
    ROBOT_REQUIRE(check.structurally_valid);
    ROBOT_REQUIRE(!check.output_unlock_allowed);
    ROBOT_REQUIRE((check.fault_bits & robot::kHardwareUnverified) != 0);
    ROBOT_REQUIRE((check.fault_bits & robot::kBadPort) == 0);
    ROBOT_REQUIRE(robot::boundedStringEqual(config.identity.team_number,
                                             expected_id, 8));
    ROBOT_REQUIRE(robot::boundedStringEqual(config.identity.robot_id,
                                             expected_id, 16));
    ROBOT_REQUIRE(!config.hardware.imu.installed);
    ROBOT_REQUIRE(config.hardware.lift.installed);
  };

  check_profile(robot::make492XRobotConfig(), "492X");
  check_profile(robot::make492ZRobotConfig(), "492Z");
}

ROBOT_TEST("492Z profile preserves its supplied Smart Port map") {
  const auto config = robot::make492ZRobotConfig();

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

ROBOT_TEST("492X differs from 492Z only by identity and Smart Ports") {
  const auto x = robot::make492XRobotConfig();
  const auto z = robot::make492ZRobotConfig();

  const std::array<int, 8> x_ports{{2, 3, 1, 14, 13, 11, 19, 18}};
  const std::array<int, 8> z_ports{{3, 2, 1, 13, 12, 11, 14, 5}};
  const auto collect_ports = [](const robot::RobotConfig& config) {
    return std::array<int, 8>{{
        config.hardware.left[0].smart_port,
        config.hardware.left[1].smart_port,
        config.hardware.left[2].smart_port,
        config.hardware.right[0].smart_port,
        config.hardware.right[1].smart_port,
        config.hardware.right[2].smart_port,
        config.hardware.lift.motors[0].smart_port,
        config.hardware.lift.motors[1].smart_port,
    }};
  };
  ROBOT_REQUIRE(collect_ports(x) == x_ports);
  ROBOT_REQUIRE(collect_ports(z) == z_ports);

  for (std::size_t i = 0; i < robot::kMotorsPerSide; ++i) {
    ROBOT_REQUIRE(x.hardware.left[i].reversed ==
                  z.hardware.left[i].reversed);
    ROBOT_REQUIRE(x.hardware.left[i].cartridge_rpm ==
                  z.hardware.left[i].cartridge_rpm);
    ROBOT_REQUIRE_NEAR(x.hardware.left[i].motor_rev_per_wheel_rev,
                       z.hardware.left[i].motor_rev_per_wheel_rev, 0.0);
    ROBOT_REQUIRE(x.hardware.right[i].reversed ==
                  z.hardware.right[i].reversed);
    ROBOT_REQUIRE(x.hardware.right[i].cartridge_rpm ==
                  z.hardware.right[i].cartridge_rpm);
    ROBOT_REQUIRE_NEAR(x.hardware.right[i].motor_rev_per_wheel_rev,
                       z.hardware.right[i].motor_rev_per_wheel_rev, 0.0);
  }
  ROBOT_REQUIRE_NEAR(x.geometry.nominal_drive_wheel_diameter_m,
                     z.geometry.nominal_drive_wheel_diameter_m, 0.0);
  ROBOT_REQUIRE_NEAR(x.geometry.nominal_track_width_m,
                     z.geometry.nominal_track_width_m, 0.0);
  ROBOT_REQUIRE_NEAR(x.hardware.lift.maximum_position_rad,
                     z.hardware.lift.maximum_position_rad, 0.0);
  for (std::size_t i = 0; i < robot::kLiftMotorCount; ++i) {
    ROBOT_REQUIRE(x.hardware.lift.motors[i].reversed ==
                  z.hardware.lift.motors[i].reversed);
    ROBOT_REQUIRE(x.hardware.lift.motors[i].cartridge_rpm ==
                  z.hardware.lift.motors[i].cartridge_rpm);
  }
  ROBOT_REQUIRE(x.hardware.imu.installed == z.hardware.imu.installed);
  ROBOT_REQUIRE(x.hardware.parallel_rotation.installed ==
                z.hardware.parallel_rotation.installed);
  ROBOT_REQUIRE(x.hardware.lateral_rotation.installed ==
                z.hardware.lateral_rotation.installed);
  ROBOT_REQUIRE_NEAR(x.runtime.nominal_period_s,
                     z.runtime.nominal_period_s, 0.0);
  ROBOT_REQUIRE(x.runtime.request_ttl_us == z.runtime.request_ttl_us);
  ROBOT_REQUIRE(x.runtime.output_ttl_us == z.runtime.output_ttl_us);
  ROBOT_REQUIRE_NEAR(x.electrical.max_command_voltage_V,
                     z.electrical.max_command_voltage_V, 0.0);
  ROBOT_REQUIRE(x.hardware_verification == z.hardware_verification);
  ROBOT_REQUIRE(x.capabilities.hardware_output ==
                z.capabilities.hardware_output);
  ROBOT_REQUIRE(x.capabilities.driver_control ==
                z.capabilities.driver_control);
  ROBOT_REQUIRE(x.capabilities.pose_good == z.capabilities.pose_good);
  ROBOT_REQUIRE(x.capabilities.autonomous_chassis_velocity ==
                z.capabilities.autonomous_chassis_velocity);
  ROBOT_REQUIRE(x.capabilities.autonomous_motion ==
                z.capabilities.autonomous_motion);
  ROBOT_REQUIRE(x.capabilities.competition_routes ==
                z.capabilities.competition_routes);
  ROBOT_REQUIRE(x.selected_route == z.selected_route);
}

ROBOT_TEST("build-selected profile binds identity and hardware atomically") {
  const auto selected = robot::makeSelectedRobotConfig();
  ROBOT_REQUIRE(robot::boundedStringEqual(selected.identity.robot_id,
                                           robot::selectedRobotId(), 16));
  const auto check = robot::validateConfig(
      selected, robot::selectedRobotId(), robot::kRobotConfigSchema);
  ROBOT_REQUIRE(check.structurally_valid);
}

ROBOT_TEST("missing odometry calibration is allowed only while pose is locked") {
  auto config = robot::make492ZRobotConfig();
  auto locked = robot::validateConfig(
      config, "492Z", robot::kRobotConfigSchema);
  ROBOT_REQUIRE(locked.structurally_valid);
  config.capabilities.hardware_output = true;
  config.capabilities.pose_good = true;
  config.hardware_verification = robot::VerificationLevel::HILValidated;
  const auto unlocked = robot::validateConfig(
      config, "492Z", robot::kRobotConfigSchema);
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
