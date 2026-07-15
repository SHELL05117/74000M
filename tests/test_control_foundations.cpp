#include "robot/control/engineering_pid.hpp"
#include "robot/control/feedforward.hpp"
#include "robot/control/motion_profile.hpp"
#include "robot/control/termination.hpp"
#include "test_framework.hpp"

#include <cmath>
#include <limits>

namespace {

robot::PidConfig pidConfig() {
  return {2.0, 1.0, 0.5, -10.0, 10.0, -2.0, 2.0,
          1.0, 20.0, 0.01, 0.001, 0.1};
}

robot::MotionTerminationConfig terminationConfig() {
  return {0.01, 0.02, 200, 1000, 0.7, 0.03, 300};
}

}  // namespace

ROBOT_TEST("engineering PID rejects invalid dt and nonfinite inputs") {
  robot::EngineeringPid pid(pidConfig());
  ROBOT_REQUIRE(pid.valid());
  auto result = pid.update(1.0, 0.0, 0.0);
  ROBOT_REQUIRE(!result.valid);
  ROBOT_REQUIRE((result.status & robot::kPidInvalidInput) != 0);
  result = pid.update(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.01);
  ROBOT_REQUIRE(!result.valid);

  auto invalid = pidConfig();
  invalid.output_min = invalid.output_max;
  robot::EngineeringPid bad(invalid);
  ROBOT_REQUIRE(!bad.valid());
  ROBOT_REQUIRE((bad.update(0.0, 0.0, 0.01).status &
                 robot::kPidInvalidConfig) != 0);
}

ROBOT_TEST("engineering PID uses derivative on measurement without kick") {
  robot::EngineeringPid pid(pidConfig());
  auto first = pid.update(0.0, 0.0, 0.01);
  ROBOT_REQUIRE(first.valid);
  ROBOT_REQUIRE_NEAR(first.derivative, 0.0, 0.0);
  const auto setpoint_step = pid.update(5.0, 0.0, 0.01);
  ROBOT_REQUIRE(setpoint_step.valid);
  ROBOT_REQUIRE_NEAR(setpoint_step.derivative, 0.0, 1e-12);
  const auto measured_motion = pid.update(5.0, 0.1, 0.01);
  ROBOT_REQUIRE(measured_motion.derivative < 0.0);
  ROBOT_REQUIRE(std::abs(measured_motion.derivative) < 5.0);
}

ROBOT_TEST("engineering PID integrates measured variable dt and resets history") {
  auto config = pidConfig();
  config.kp = 0.0;
  config.kd = 0.0;
  config.ki = 2.0;
  robot::EngineeringPid pid(config);
  auto result = pid.update(0.5, 0.0, 0.01);
  ROBOT_REQUIRE_NEAR(result.integral, 0.01, 1e-12);
  result = pid.update(0.5, 0.0, 0.02);
  ROBOT_REQUIRE_NEAR(result.integral, 0.03, 1e-12);
  pid.reset();
  ROBOT_REQUIRE_NEAR(pid.integralState(), 0.0, 0.0);
  result = pid.update(0.5, 0.0, 0.01);
  ROBOT_REQUIRE_NEAR(result.integral, 0.01, 1e-12);
  ROBOT_REQUIRE_NEAR(result.derivative, 0.0, 0.0);
}

ROBOT_TEST("engineering PID holds integral when saturation pushes outward") {
  auto config = pidConfig();
  config.kp = 100.0;
  config.ki = 10.0;
  config.kd = 0.0;
  config.output_min = -1.0;
  config.output_max = 1.0;
  robot::EngineeringPid pid(config);
  const auto result = pid.update(0.5, 0.0, 0.01);
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE_NEAR(result.output, 1.0, 0.0);
  ROBOT_REQUIRE_NEAR(result.integral, 0.0, 0.0);
  ROBOT_REQUIRE((result.status & robot::kPidIntegralHeldForSaturation) != 0);
  ROBOT_REQUIRE((result.status & robot::kPidOutputSaturatedHigh) != 0);
}

ROBOT_TEST("engineering PID applies integral zone clamp and deadband") {
  auto config = pidConfig();
  config.kp = 0.0;
  config.kd = 0.0;
  config.ki = 100.0;
  config.integral_min = -0.2;
  config.integral_max = 0.2;
  config.integral_zone = 0.5;
  robot::EngineeringPid pid(config);
  auto result = pid.update(1.0, 0.0, 0.01);
  ROBOT_REQUIRE_NEAR(result.integral, 0.0, 0.0);
  ROBOT_REQUIRE((result.status & robot::kPidIntegralOutsideZone) != 0);
  for (int i = 0; i < 10; ++i) result = pid.update(0.1, 0.0, 0.01);
  ROBOT_REQUIRE_NEAR(result.integral, 0.2, 1e-12);
  result = pid.update(0.005, 0.0, 0.01);
  ROBOT_REQUIRE((result.status & robot::kPidDeadband) != 0);
}

ROBOT_TEST("directional and side-specific feedforward preserve signs") {
  const robot::DifferentialFeedforwardConfig config{
      {0.4, 0.6, 2.0, 0.3}, {0.5, 0.7, 2.1, 0.4}};
  const auto forward = robot::calculateDifferentialFeedforward(
      config, 1.0, 2.0, 1.0, 2.0);
  ROBOT_REQUIRE(forward.valid);
  ROBOT_REQUIRE_NEAR(forward.left_V, 3.0, 1e-12);
  ROBOT_REQUIRE_NEAR(forward.right_V, 3.4, 1e-12);
  const auto reverse = robot::calculateDifferentialFeedforward(
      config, -1.0, -2.0, -1.0, -2.0);
  ROBOT_REQUIRE(reverse.valid);
  ROBOT_REQUIRE_NEAR(reverse.left_V, -3.2, 1e-12);
  ROBOT_REQUIRE_NEAR(reverse.right_V, -3.6, 1e-12);
  const auto start = robot::calculateFeedforward(config.left, 0.0, 1.0);
  ROBOT_REQUIRE_NEAR(start.voltage_V, 0.7, 1e-12);
}

ROBOT_TEST("feedforward rejects invalid coefficients and NaN") {
  robot::DirectionalFeedforwardGains gains{0.1, 0.1, 1.0, 0.1};
  ROBOT_REQUIRE(!robot::calculateFeedforward(
                     gains, std::numeric_limits<double>::infinity(), 0.0)
                     .valid);
  gains.kV_Vs_per_unit = -1.0;
  ROBOT_REQUIRE(!robot::calculateFeedforward(gains, 1.0, 0.0).valid);
}

ROBOT_TEST("trapezoid profile degenerates to triangle and reaches endpoint") {
  robot::TrapezoidProfile profile;
  ROBOT_REQUIRE(profile.configure(1.0, {2.0, 1.0}));
  ROBOT_REQUIRE(profile.triangular());
  ROBOT_REQUIRE_NEAR(profile.peakVelocity(), 1.0, 1e-12);
  ROBOT_REQUIRE_NEAR(profile.totalTime(), 2.0, 1e-12);
  const auto midpoint = profile.sample(1.0);
  ROBOT_REQUIRE(midpoint.valid);
  ROBOT_REQUIRE_NEAR(midpoint.position, 0.5, 1e-12);
  const auto endpoint = profile.sample(profile.totalTime());
  ROBOT_REQUIRE_NEAR(endpoint.position, 1.0, 1e-12);
  ROBOT_REQUIRE_NEAR(endpoint.velocity, 0.0, 0.0);
}

ROBOT_TEST("trapezoid profile supports cruise reverse and nonzero start") {
  robot::TrapezoidProfile long_profile;
  ROBOT_REQUIRE(long_profile.configure(10.0, {2.0, 1.0}));
  ROBOT_REQUIRE(!long_profile.triangular());
  ROBOT_REQUIRE_NEAR(long_profile.totalTime(), 7.0, 1e-12);
  robot::TrapezoidProfile reverse;
  ROBOT_REQUIRE(reverse.configure(-2.0, {2.0, 2.0}));
  const auto reverse_end = reverse.sample(reverse.totalTime() + 1.0);
  ROBOT_REQUIRE_NEAR(reverse_end.position, -2.0, 1e-12);
  ROBOT_REQUIRE_NEAR(reverse_end.velocity, 0.0, 0.0);
  robot::TrapezoidProfile blended;
  ROBOT_REQUIRE(blended.configure(3.0, {2.0, 1.0}, 1.0));
  ROBOT_REQUIRE_NEAR(blended.sample(0.0).velocity, 1.0, 1e-12);
  ROBOT_REQUIRE_NEAR(blended.sample(blended.totalTime()).position, 3.0, 1e-12);
}

ROBOT_TEST("trapezoid profile rejects infeasible or nonfinite boundaries") {
  robot::TrapezoidProfile profile;
  ROBOT_REQUIRE(!profile.configure(0.1, {2.0, 1.0}, 2.0));
  ROBOT_REQUIRE(!profile.configure(
      std::numeric_limits<double>::quiet_NaN(), {2.0, 1.0}));
  ROBOT_REQUIRE(!profile.sample(0.0).valid);
}

ROBOT_TEST("quintic S curve honors endpoint and kinematic limits") {
  const robot::SCurveProfileConfig limits{1.2, 2.0, 8.0};
  robot::QuinticSCurveProfile profile;
  ROBOT_REQUIRE(profile.configure(2.0, limits));
  const auto start = profile.sample(0.0);
  const auto end = profile.sample(profile.totalTime());
  ROBOT_REQUIRE_NEAR(start.position, 0.0, 0.0);
  ROBOT_REQUIRE_NEAR(start.velocity, 0.0, 0.0);
  ROBOT_REQUIRE_NEAR(start.acceleration, 0.0, 0.0);
  ROBOT_REQUIRE_NEAR(end.position, 2.0, 1e-12);
  ROBOT_REQUIRE_NEAR(end.velocity, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(end.acceleration, 0.0, 1e-12);
  for (int i = 0; i <= 100; ++i) {
    const auto sample =
        profile.sample(profile.totalTime() * static_cast<double>(i) / 100.0);
    ROBOT_REQUIRE(sample.valid);
    ROBOT_REQUIRE(std::abs(sample.velocity) <= limits.max_velocity + 1e-9);
    ROBOT_REQUIRE(std::abs(sample.acceleration) <=
                  limits.max_acceleration + 1e-9);
    ROBOT_REQUIRE(std::abs(sample.jerk) <= limits.max_jerk + 1e-9);
  }
}

ROBOT_TEST("quintic S curve handles reverse zero and invalid limits") {
  robot::QuinticSCurveProfile profile;
  ROBOT_REQUIRE(profile.configure(-1.0, {2.0, 3.0, 10.0}));
  ROBOT_REQUIRE_NEAR(profile.sample(profile.totalTime()).position, -1.0,
                     1e-12);
  ROBOT_REQUIRE(profile.configure(0.0, {2.0, 3.0, 10.0}));
  ROBOT_REQUIRE_NEAR(profile.totalTime(), 0.0, 0.0);
  ROBOT_REQUIRE_NEAR(profile.sample(1.0).position, 0.0, 0.0);
  ROBOT_REQUIRE(!profile.configure(1.0, {0.0, 3.0, 10.0}));
}

ROBOT_TEST("termination monitor requires error and velocity settle bands") {
  robot::MotionTerminationMonitor monitor(terminationConfig());
  ROBOT_REQUIRE(monitor.start(0));
  ROBOT_REQUIRE(monitor.update(0, 1.0, 0.0, 0.1, true) ==
                robot::MotionTerminationState::Running);
  ROBOT_REQUIRE(monitor.update(100, 0.005, 0.01, 0.1, true) ==
                robot::MotionTerminationState::Settling);
  ROBOT_REQUIRE(monitor.update(299, 0.005, 0.01, 0.1, true) ==
                robot::MotionTerminationState::Settling);
  ROBOT_REQUIRE(monitor.update(300, 0.005, 0.01, 0.1, true) ==
                robot::MotionTerminationState::Succeeded);
  ROBOT_REQUIRE(monitor.terminal());
}

ROBOT_TEST("termination monitor detects timeout stall invalid state and reset") {
  robot::MotionTerminationMonitor timeout(terminationConfig());
  ROBOT_REQUIRE(timeout.start(0));
  ROBOT_REQUIRE(timeout.update(1000, 1.0, 1.0, 0.1, true) ==
                robot::MotionTerminationState::TimedOut);

  robot::MotionTerminationMonitor stall(terminationConfig());
  ROBOT_REQUIRE(stall.start(0));
  ROBOT_REQUIRE(stall.update(100, 1.0, 0.01, 0.9, true) ==
                robot::MotionTerminationState::Running);
  ROBOT_REQUIRE(stall.update(400, 1.0, 0.01, 0.9, true) ==
                robot::MotionTerminationState::Stalled);
  stall.reset();
  ROBOT_REQUIRE(stall.state() == robot::MotionTerminationState::Idle);
  ROBOT_REQUIRE(stall.start(500));
  ROBOT_REQUIRE(stall.update(499, 0.0, 0.0, 0.0, true) ==
                robot::MotionTerminationState::StateInvalid);
}
