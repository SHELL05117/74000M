#include "robot/sensors/filters.hpp"
#include "robot/sensors/sensor_validator.hpp"
#include "test_framework.hpp"

#include <cmath>
#include <limits>

namespace {

robot::ScalarValidationConfig scalarConfig(double minimum = -1000.0,
                                           double maximum = 1000.0) {
  return {minimum, maximum, 10000.0, 20000, 1e-12, 3000, 1};
}

robot::SensorValidatorConfig sensorConfig() {
  robot::SensorValidatorConfig config{};
  config.motor_position_rad = scalarConfig();
  config.motor_velocity_radps = scalarConfig();
  config.motor_current_A = scalarConfig(0.0, 20.0);
  config.motor_temperature_C = scalarConfig(-50.0, 150.0);
  config.motor_applied_voltage_V = scalarConfig(-12.1, 12.1);
  config.tracking_position_rad = scalarConfig();
  config.tracking_velocity_radps = scalarConfig();
  config.imu_rotation_rad = scalarConfig();
  config.imu_yaw_rate_radps = scalarConfig();
  config.battery_V = scalarConfig(0.1, 20.0);
  config.max_acquisition_age_us = 20000;
  return config;
}

robot::ScalarSample sample(double value, robot::TimeUs time_us) {
  return {value, time_us, true, 0};
}

robot::RawDriveInputs rawFrame(robot::TimeUs time_us,
                               std::uint32_t sequence,
                               std::uint32_t epoch = 1) {
  robot::RawDriveInputs raw{};
  raw.h = {time_us, sequence, epoch};
  raw.acquisition_end_us = time_us;
  for (std::size_t i = 0; i < robot::kMotorsPerSide; ++i) {
    auto fill = [&](robot::MotorSample& motor, std::uint8_t port) {
      motor.smart_port = port;
      motor.position_rad = sample(0.01 * sequence, time_us);
      motor.velocity_radps = sample(1.0, time_us);
      motor.current_A = sample(0.5, time_us);
      motor.temperature_C = sample(30.0, time_us);
      motor.applied_voltage_V = sample(2.0, time_us);
      motor.faults_api_ok = true;
    };
    fill(raw.left.motor[i], static_cast<std::uint8_t>(i + 1));
    fill(raw.right.motor[i], static_cast<std::uint8_t>(i + 4));
  }
  raw.imu.rotation_rad = sample(0.01 * sequence, time_us);
  raw.imu.yaw_rate_radps = sample(0.1, time_us);
  raw.imu.status_api_ok = true;
  raw.battery_V = sample(12.0, time_us);
  return raw;
}

}  // namespace

ROBOT_TEST("scalar validator warms up then accepts a healthy sequence") {
  robot::ScalarValidator validator(scalarConfig());
  auto checked = validator.update(sample(0.0, 1000), 1000, false);
  ROBOT_REQUIRE(checked.quality == robot::Quality::Degraded);
  ROBOT_REQUIRE((checked.reject_bits & robot::kSensorWarmup) != 0);
  checked = validator.update(sample(0.001, 2000), 2000, false);
  ROBOT_REQUIRE(checked.quality == robot::Quality::Good);
}

ROBOT_TEST("bad scalar samples do not advance the accepted baseline") {
  auto config = scalarConfig();
  config.max_abs_rate_per_s = 10.0;
  robot::ScalarValidator validator(config);
  validator.update(sample(0.0, 1000), 1000, false);
  validator.update(sample(0.001, 2000), 2000, false);
  auto checked = validator.update(sample(1.0, 3000), 3000, false);
  ROBOT_REQUIRE(checked.quality == robot::Quality::Invalid);
  ROBOT_REQUIRE((checked.reject_bits & robot::kSensorImplausibleRate) != 0);
  checked = validator.update(sample(0.002, 4000), 4000, false);
  ROBOT_REQUIRE(checked.quality == robot::Quality::Degraded);
  ROBOT_REQUIRE_NEAR(checked.value, 0.002, 1e-12);
}

ROBOT_TEST("scalar validator detects freeze only when motion is expected") {
  robot::ScalarValidator validator(scalarConfig());
  validator.update(sample(1.0, 1000), 1000, true);
  validator.update(sample(1.0, 2000), 2000, true);
  validator.update(sample(1.0, 3000), 3000, true);
  const auto frozen = validator.update(sample(1.0, 4000), 4000, true);
  ROBOT_REQUIRE(frozen.quality == robot::Quality::Invalid);
  ROBOT_REQUIRE((frozen.reject_bits & robot::kSensorFrozen) != 0);

  validator.reset();
  validator.update(sample(1.0, 1000), 1000, false);
  validator.update(sample(1.0, 5000), 5000, false);
  const auto stationary = validator.update(sample(1.0, 9000), 9000, false);
  ROBOT_REQUIRE(stationary.quality == robot::Quality::Good);
}

ROBOT_TEST("scalar validator rejects stale future and nonfinite samples") {
  robot::ScalarValidator validator(scalarConfig());
  auto checked = validator.update(sample(1.0, 1000), 22000, false);
  ROBOT_REQUIRE((checked.reject_bits & robot::kSensorStale) != 0);
  checked = validator.update(sample(1.0, 30000), 20000, false);
  ROBOT_REQUIRE((checked.reject_bits & robot::kSensorFutureTimestamp) != 0);
  checked = validator.update(
      sample(std::numeric_limits<double>::quiet_NaN(), 20000), 20000, false);
  ROBOT_REQUIRE((checked.reject_bits & robot::kSensorNonfinite) != 0);
}

ROBOT_TEST("median low pass and window slope have explicit resettable state") {
  robot::Median3 median;
  double output{};
  ROBOT_REQUIRE(median.update(1.0, output));
  ROBOT_REQUIRE(median.update(100.0, output));
  ROBOT_REQUIRE(median.update(2.0, output));
  ROBOT_REQUIRE_NEAR(output, 2.0, 1e-12);

  robot::OnePoleLowPass low_pass(8.0);
  ROBOT_REQUIRE(low_pass.update(0.0, 0.010, output));
  ROBOT_REQUIRE(low_pass.update(1.0, 0.020, output));
  ROBOT_REQUIRE(output > 0.0 && output < 1.0);
  low_pass.reset();
  ROBOT_REQUIRE(low_pass.update(5.0, 0.010, output));
  ROBOT_REQUIRE_NEAR(output, 5.0, 1e-12);

  robot::WindowedSlope<5> slope;
  for (std::uint32_t i = 0; i < 5; ++i)
    slope.update(2.0 * static_cast<double>(i), i * 1000000, output);
  ROBOT_REQUIRE_NEAR(output, 2.0, 1e-12);
}

ROBOT_TEST("sensor validator publishes one same-frame checked snapshot") {
  robot::SensorValidator validator(sensorConfig());
  const robot::MotionExpectation moving{true, true, false, true};
  const auto first = validator.update(rawFrame(10000, 1), 10000, moving);
  ROBOT_REQUIRE(first.left[0].kinematic_quality == robot::Quality::Degraded);
  const auto second = validator.update(rawFrame(20000, 2), 20000, moving);
  ROBOT_REQUIRE(second.h.sequence == 2);
  ROBOT_REQUIRE(second.left[0].smart_port == 1);
  ROBOT_REQUIRE(second.right[2].smart_port == 6);
  ROBOT_REQUIRE(second.left[0].kinematic_quality == robot::Quality::Good);
  ROBOT_REQUIRE(second.left[0].velocity_radps.quality == robot::Quality::Good);
  ROBOT_REQUIRE(second.imu.rotation_rad.quality == robot::Quality::Good);
}

ROBOT_TEST("sensor validator isolates a bad motor and rejects duplicate frames") {
  robot::SensorValidator validator(sensorConfig());
  validator.update(rawFrame(10000, 1), 10000);
  auto raw = rawFrame(20000, 2);
  raw.left.motor[1].position_rad.value =
      std::numeric_limits<double>::quiet_NaN();
  const auto bad = validator.update(raw, 20000);
  ROBOT_REQUIRE(bad.left[1].kinematic_quality == robot::Quality::Invalid);
  ROBOT_REQUIRE(bad.left[0].kinematic_quality == robot::Quality::Good);
  ROBOT_REQUIRE(robot::hasFault(bad.fault_bits, robot::Fault::SensorInvalid));

  const auto duplicate = validator.update(rawFrame(20000, 2), 20000);
  ROBOT_REQUIRE((duplicate.frame_reject_bits & robot::kSensorFrameRejected) !=
                0);
}

ROBOT_TEST("imu calibration cannot publish a valid heading") {
  robot::SensorValidator validator(sensorConfig());
  auto raw = rawFrame(10000, 1);
  raw.imu.calibrating = true;
  const auto checked = validator.update(raw, 10000);
  ROBOT_REQUIRE(checked.imu.rotation_rad.quality == robot::Quality::Invalid);
  ROBOT_REQUIRE((checked.imu.rotation_rad.reject_bits &
                 robot::kSensorCalibrating) != 0);
}
