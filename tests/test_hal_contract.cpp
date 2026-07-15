#include "robot/platform/hardware_self_test.hpp"
#include "robot/state/raw_inputs.hpp"
#include "test_framework.hpp"

#include <type_traits>

namespace {

robot::HardwareConfig hardwareConfig() {
  robot::HardwareConfig hardware{};
  hardware.left = {{{1, false, 200}, {2, false, 200}, {3, false, 200}}};
  hardware.right = {{{4, true, 200}, {5, true, 200}, {6, true, 200}}};
  hardware.imu = {true, 7};
  return hardware;
}

robot::RawDriveInputs validRaw(const robot::HardwareConfig& hardware,
                               robot::TimeUs now_us) {
  robot::RawDriveInputs raw{};
  raw.h.time_us = now_us;
  auto fill_motor = [now_us](robot::MotorSample& sample,
                             const robot::MotorPortConfig& config) {
    sample.smart_port = static_cast<std::uint8_t>(config.smart_port);
    sample.position_rad = {0.0, now_us, true, 0};
    sample.velocity_radps = {0.0, now_us, true, 0};
    sample.current_A = {0.0, now_us, true, 0};
    sample.temperature_C = {25.0, now_us, true, 0};
    sample.applied_voltage_V = {0.0, now_us, true, 0};
    sample.faults_api_ok = true;
  };
  for (std::size_t i = 0; i < robot::kMotorsPerSide; ++i) {
    fill_motor(raw.left.motor[i], hardware.left[i]);
    fill_motor(raw.right.motor[i], hardware.right[i]);
  }
  raw.imu.status_api_ok = hardware.imu.installed;
  raw.imu.rotation_rad = {0.0, now_us, hardware.imu.installed, 0};
  raw.imu.yaw_rate_radps = {0.0, now_us, hardware.imu.installed, 0};
  raw.battery_V = {12.0, now_us, true, 0};
  raw.acquisition_end_us = now_us;
  return raw;
}

}  // namespace

ROBOT_TEST("raw input keeps every motor identity before aggregation") {
  robot::RawDriveInputs raw{};
  for (std::size_t i = 0; i < robot::kMotorsPerSide; ++i) {
    raw.left.motor[i].smart_port = static_cast<std::uint8_t>(i + 1);
    raw.right.motor[i].smart_port = static_cast<std::uint8_t>(i + 4);
  }
  ROBOT_REQUIRE(raw.left.motor[2].smart_port == 3);
  ROBOT_REQUIRE(raw.right.motor[0].smart_port == 4);
  ROBOT_REQUIRE(std::is_trivially_copyable_v<robot::RawDriveInputs>);
}

ROBOT_TEST("startup self check cannot complete before three seconds") {
  const auto hardware = hardwareConfig();
  robot::StartupSelfCheck check({1000, 2000});
  check.begin(0, hardware, true, true, true);
  ROBOT_REQUIRE(!check.tick(2999999, validRaw(hardware, 2999999)).complete);
  const auto status = check.tick(3000000, validRaw(hardware, 3000000));
  ROBOT_REQUIRE(status.complete);
  ROBOT_REQUIRE(status.healthy);
  ROBOT_REQUIRE(status.elapsed_us == robot::kMinimumStartupSelfCheckUs);
}

ROBOT_TEST("startup self check reports the sensorless commissioning IMU") {
  auto hardware = hardwareConfig();
  hardware.imu = {false, 0};
  robot::StartupSelfCheck check;
  check.begin(0, hardware, true, true, false);
  const auto status =
      check.tick(robot::kMinimumStartupSelfCheckUs,
                 validRaw(hardware, robot::kMinimumStartupSelfCheckUs));
  ROBOT_REQUIRE(status.complete);
  ROBOT_REQUIRE(!status.healthy);
  ROBOT_REQUIRE((status.fault_bits & robot::kStartupImuMissing) != 0);
}

ROBOT_TEST("startup self check waits for IMU then reports invalid data") {
  const auto hardware = hardwareConfig();
  robot::StartupSelfCheck check;
  check.begin(0, hardware, true, true, true);
  auto raw = validRaw(hardware, robot::kMinimumStartupSelfCheckUs);
  raw.imu.calibrating = true;
  raw.imu.rotation_rad.api_ok = false;
  raw.imu.yaw_rate_radps.api_ok = false;
  ROBOT_REQUIRE(
      !check.tick(robot::kMinimumStartupSelfCheckUs, raw).complete);
  raw.h.time_us = 5000000;
  const auto status = check.tick(5000000, raw);
  ROBOT_REQUIRE(status.complete);
  ROBOT_REQUIRE((status.fault_bits & robot::kStartupImuInvalid) != 0);
}

ROBOT_TEST("startup self check identifies a missing drivetrain motor") {
  const auto hardware = hardwareConfig();
  robot::StartupSelfCheck check;
  check.begin(0, hardware, true, true, true);
  auto raw = validRaw(hardware, 5000000);
  raw.left.motor[1].position_rad.api_ok = false;
  raw.left.motor[1].velocity_radps.api_ok = false;
  raw.left.motor[1].current_A.api_ok = false;
  raw.left.motor[1].temperature_C.api_ok = false;
  raw.left.motor[1].applied_voltage_V.api_ok = false;
  raw.left.motor[1].faults_api_ok = false;
  const auto status = check.tick(5000000, raw);
  ROBOT_REQUIRE(status.complete);
  ROBOT_REQUIRE((status.fault_bits & robot::kStartupMotorMissing) != 0);
}

ROBOT_TEST("direction test cannot start without disabled bench authorization") {
  robot::DirectionSelfTest test;
  const robot::DirectionSelfTestLimits limits{1.0, 50000, 0.01};
  ROBOT_REQUIRE(!test.begin(hardwareConfig(), limits, false, true));
  ROBOT_REQUIRE(test.state() == robot::DirectionTestState::Locked);
  ROBOT_REQUIRE(!test.begin(hardwareConfig(), limits, true, false));
  ROBOT_REQUIRE(test.state() == robot::DirectionTestState::Locked);
}

ROBOT_TEST("direction test emits one bounded intent and requires positive feedback") {
  robot::DirectionSelfTest test;
  const robot::DirectionSelfTestLimits limits{1.0, 50000, 0.01};
  ROBOT_REQUIRE(test.begin(hardwareConfig(), limits, true, true));
  robot::DirectionPulseIntent intent{};
  ROBOT_REQUIRE(test.nextIntent(intent));
  ROBOT_REQUIRE(intent.smart_port == 1);
  ROBOT_REQUIRE(intent.voltage_V == 1.0);
  ROBOT_REQUIRE(test.observe(1, 0.02, true, false));
  ROBOT_REQUIRE(test.state() == robot::DirectionTestState::Ready);
}

ROBOT_TEST("direction test locks failure to the affected port") {
  robot::DirectionSelfTest test;
  ROBOT_REQUIRE(test.begin(hardwareConfig(), {1.0, 50000, 0.01}, true, true));
  robot::DirectionPulseIntent intent{};
  ROBOT_REQUIRE(test.nextIntent(intent));
  ROBOT_REQUIRE(!test.observe(intent.smart_port, -0.02, true, false));
  ROBOT_REQUIRE(test.state() == robot::DirectionTestState::Failed);
  ROBOT_REQUIRE(test.failedPort() == 1);
}
