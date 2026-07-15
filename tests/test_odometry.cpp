#include "robot/odometry/odometry.hpp"
#include "test_framework.hpp"

#include <cmath>

namespace {

robot::CheckedScalar checked(double value,
                             robot::Quality quality = robot::Quality::Good,
                             robot::TimeUs time_us = 0) {
  return {value, time_us, quality, 0};
}

robot::OdometryConfig driveConfig() {
  robot::OdometryConfig config{};
  config.layout = robot::OdomLayout::DriveEncodersImu;
  config.left_drive_m_per_motor_rad = 1.0;
  config.right_drive_m_per_motor_rad = 1.0;
  config.effective_track_width_m = 0.5;
  config.max_dt_s = 0.05;
  config.min_geometry_determinant = 1e-6;
  config.heading_disagreement_rad = 0.2;
  config.slip_confirm_samples = 2;
  return config;
}

robot::OdometryConfig twoTrackingConfig() {
  auto config = driveConfig();
  config.layout = robot::OdomLayout::TwoTrackingImu;
  config.tracking_count = 2;
  config.tracking[0] = {0.0, 0.2, 0.0, 1.0};
  config.tracking[1] = {0.1, 0.0, robot::units::kPi / 2.0, 1.0};
  return config;
}

robot::OdometryConfig threeTrackingConfig() {
  auto config = twoTrackingConfig();
  config.layout = robot::OdomLayout::ThreeTrackingImu;
  config.tracking_count = 3;
  config.tracking[1] = {0.0, -0.2, 0.0, 1.0};
  config.tracking[2] = {0.1, 0.0, robot::units::kPi / 2.0, 1.0};
  return config;
}

robot::OdomObservation driveObservation(std::uint32_t sequence, double left,
                                        double right, double imu,
                                        std::uint32_t epoch = 1) {
  robot::OdomObservation observation{};
  observation.h = {sequence * 10000u, sequence, epoch};
  observation.left_distance_m = checked(left);
  observation.right_distance_m = checked(right);
  observation.imu_rotation_rad = checked(imu);
  observation.imu_rate_radps = checked(0.0);
  return observation;
}

}  // namespace

ROBOT_TEST("drive encoder odometry integrates straight motion with SE2") {
  robot::Odometry odometry(driveConfig());
  auto estimate = odometry.update(driveObservation(1, 0.0, 0.0, 0.0), 0.01);
  ROBOT_REQUIRE(estimate.translation_quality == robot::Quality::Invalid);
  estimate = odometry.update(driveObservation(2, 1.0, 1.0, 0.0), 0.01);
  ROBOT_REQUIRE_NEAR(estimate.pose.x_m, 1.0, 1e-12);
  ROBOT_REQUIRE_NEAR(estimate.pose.y_m, 0.0, 1e-12);
  ROBOT_REQUIRE(estimate.translation_quality == robot::Quality::Good);
  ROBOT_REQUIRE(estimate.heading_quality == robot::Quality::Good);
}

ROBOT_TEST("positive differential and IMU angle produce a left turn") {
  robot::Odometry odometry(driveConfig());
  odometry.update(driveObservation(1, 0.0, 0.0, 0.0), 0.01);
  const auto estimate =
      odometry.update(driveObservation(2, -0.25, 0.25, 1.0), 0.01);
  ROBOT_REQUIRE_NEAR(estimate.pose.theta_rad, 1.0, 1e-12);
  ROBOT_REQUIRE_NEAR(estimate.pose.x_m, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(estimate.pose.y_m, 0.0, 1e-12);
}

ROBOT_TEST("IMU only advances heading and keeps translation invalid") {
  robot::Odometry odometry(driveConfig());
  auto first = driveObservation(1, 0.0, 0.0, 0.0);
  first.left_distance_m.quality = robot::Quality::Invalid;
  first.right_distance_m.quality = robot::Quality::Invalid;
  odometry.update(first, 0.01);
  auto second = driveObservation(2, 0.0, 0.0, 0.5);
  second.left_distance_m.quality = robot::Quality::Invalid;
  second.right_distance_m.quality = robot::Quality::Invalid;
  const auto estimate = odometry.update(second, 0.01);
  ROBOT_REQUIRE_NEAR(estimate.pose.theta_rad, 0.5, 1e-12);
  ROBOT_REQUIRE_NEAR(estimate.pose.x_m, 0.0, 1e-12);
  ROBOT_REQUIRE(estimate.heading_quality == robot::Quality::Good);
  ROBOT_REQUIRE(estimate.translation_quality == robot::Quality::Invalid);
}

ROBOT_TEST("two tracking wheels remove rotation with configured offsets") {
  robot::Odometry odometry(twoTrackingConfig());
  robot::OdomObservation first{};
  first.h = {10000, 1, 1};
  first.tracking_distance_m[0] = checked(0.0);
  first.tracking_distance_m[1] = checked(0.0);
  first.imu_rotation_rad = checked(0.0);
  odometry.update(first, 0.01);

  auto second = first;
  second.h = {20000, 2, 1};
  second.tracking_distance_m[0] = checked(-0.2);
  second.tracking_distance_m[1] = checked(0.1);
  second.imu_rotation_rad = checked(1.0);
  const auto estimate = odometry.update(second, 0.01);
  ROBOT_REQUIRE_NEAR(estimate.pose.x_m, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(estimate.pose.y_m, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(estimate.pose.theta_rad, 1.0, 1e-12);
  ROBOT_REQUIRE(estimate.translation_quality == robot::Quality::Good);
}

ROBOT_TEST("three tracking wheels provide continuous degraded heading fallback") {
  robot::Odometry odometry(threeTrackingConfig());
  robot::OdomObservation first{};
  first.h = {10000, 1, 1};
  for (auto& wheel : first.tracking_distance_m) wheel = checked(0.0);
  first.imu_rotation_rad = checked(0.0, robot::Quality::Invalid);
  odometry.update(first, 0.01);

  auto second = first;
  second.h = {20000, 2, 1};
  second.tracking_distance_m[0] = checked(-0.2);
  second.tracking_distance_m[1] = checked(0.2);
  second.tracking_distance_m[2] = checked(0.1);
  const auto estimate = odometry.update(second, 0.01);
  ROBOT_REQUIRE_NEAR(estimate.pose.theta_rad, 1.0, 1e-12);
  ROBOT_REQUIRE_NEAR(estimate.pose.x_m, 0.0, 1e-12);
  ROBOT_REQUIRE(estimate.heading_quality == robot::Quality::Degraded);
  ROBOT_REQUIRE(estimate.translation_quality == robot::Quality::Degraded);
}

ROBOT_TEST("pose reset captures every baseline at one update boundary") {
  robot::Odometry odometry(driveConfig());
  odometry.update(driveObservation(1, 5.0, 5.0, 2.0), 0.01);
  ROBOT_REQUIRE(odometry.requestReset({{1.0, 2.0, 3.0}, 7}));
  auto estimate =
      odometry.update(driveObservation(2, 5.0, 5.0, 2.0), 0.01);
  ROBOT_REQUIRE_NEAR(estimate.pose.x_m, 1.0, 1e-12);
  ROBOT_REQUIRE_NEAR(estimate.pose.y_m, 2.0, 1e-12);
  ROBOT_REQUIRE_NEAR(estimate.pose.theta_rad, 3.0, 1e-12);
  ROBOT_REQUIRE(estimate.reset_generation == 1);

  estimate = odometry.update(driveObservation(3, 5.0, 5.0, 2.0), 0.01);
  ROBOT_REQUIRE_NEAR(estimate.pose.x_m, 1.0, 1e-12);
  ROBOT_REQUIRE_NEAR(estimate.pose.y_m, 2.0, 1e-12);
  ROBOT_REQUIRE_NEAR(estimate.pose.theta_rad, 3.0, 1e-12);
  ROBOT_REQUIRE(estimate.reset_generation == 1);
}

ROBOT_TEST("pose reset does not invent sensor quality") {
  robot::Odometry odometry(driveConfig());
  robot::OdomObservation invalid{};
  invalid.h = {10000, 1, 1};
  odometry.update(invalid, 0.01);
  ROBOT_REQUIRE(odometry.requestReset({{1.0, 2.0, 3.0}, 8}));
  invalid.h = {20000, 2, 1};
  const auto estimate = odometry.update(invalid, 0.01);
  ROBOT_REQUIRE_NEAR(estimate.pose.x_m, 1.0, 1e-12);
  ROBOT_REQUIRE(estimate.translation_quality == robot::Quality::Invalid);
  ROBOT_REQUIRE(estimate.heading_quality == robot::Quality::Invalid);
}

ROBOT_TEST("odometry rejects unobservable tracking geometry") {
  auto config = twoTrackingConfig();
  config.tracking[1].phi_rad = 0.0;
  robot::Odometry odometry(config);
  ROBOT_REQUIRE(!odometry.valid());
  const auto estimate = odometry.update({}, 0.01);
  ROBOT_REQUIRE((estimate.fault_bits & robot::kOdomBadConfig) != 0);
}

ROBOT_TEST("persistent IMU wheel disagreement invalidates translation") {
  robot::Odometry odometry(driveConfig());
  odometry.update(driveObservation(1, 0.0, 0.0, 0.0), 0.01);
  auto estimate =
      odometry.update(driveObservation(2, -0.25, 0.25, 0.0), 0.01);
  ROBOT_REQUIRE(estimate.slip == robot::SlipState::Suspected);
  ROBOT_REQUIRE(estimate.translation_quality == robot::Quality::Degraded);
  estimate =
      odometry.update(driveObservation(3, -0.5, 0.5, 0.0), 0.01);
  ROBOT_REQUIRE(estimate.slip == robot::SlipState::Confirmed);
  ROBOT_REQUIRE(estimate.translation_quality == robot::Quality::Invalid);
}

ROBOT_TEST("odometry rejects sequence regression without changing pose") {
  robot::Odometry odometry(driveConfig());
  odometry.update(driveObservation(1, 0.0, 0.0, 0.0), 0.01);
  const auto moved =
      odometry.update(driveObservation(2, 1.0, 1.0, 0.0), 0.01);
  const auto duplicate =
      odometry.update(driveObservation(2, 2.0, 2.0, 0.0), 0.01);
  ROBOT_REQUIRE((duplicate.fault_bits & robot::kOdomTimingInvalid) != 0);
  ROBOT_REQUIRE_NEAR(duplicate.pose.x_m, moved.pose.x_m, 1e-12);
}
