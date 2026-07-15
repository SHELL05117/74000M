#include "robot/health/fault_manager.hpp"
#include "robot/health/motor_protection.hpp"
#include "test_framework.hpp"

#include <array>

namespace {

robot::CheckedScalar good(double value) {
  return {value, 0, robot::Quality::Good, 0};
}

robot::ValidatedInputs motorInputs(double temperature_C, double current_A,
                                   double velocity_radps) {
  robot::ValidatedInputs inputs{};
  for (std::size_t i = 0; i < robot::kMotorsPerSide; ++i) {
    auto fill = [&](robot::ValidatedMotor& motor) {
      motor.temperature_C = good(temperature_C);
      motor.current_A = good(current_A);
      motor.velocity_radps = good(velocity_radps);
    };
    fill(inputs.left[i]);
    fill(inputs.right[i]);
  }
  return inputs;
}

robot::MotorProtectionConfig protectionConfig() {
  return {60.0, 55.0, 3.0, 2.0, 6.0, 2.0, 0.2, 1.0, 2.5};
}

std::array<robot::FaultRuleConfig, 3> faultRules() {
  return {{{robot::Fault::ThermalDerate, robot::FaultSeverity::Derate, 1000,
            2000, 0.5, false},
           {robot::Fault::MotorOverCurrent, robot::FaultSeverity::Stop, 500,
            1000, 0.0, false},
           {robot::Fault::Stall, robot::FaultSeverity::LatchedStop, 0, 0, 0.0,
            true}}};
}

robot::ModeSnapshot disabledMode() {
  return {robot::CompetitionMode::Disabled, false, false, 3, 0, 0};
}

robot::ModeSnapshot driverMode() {
  return {robot::CompetitionMode::Driver, true, false, 2, 0, 0};
}

}  // namespace

ROBOT_TEST("motor protection applies per-motor thermal hysteresis") {
  robot::MotorProtectionMonitor monitor(protectionConfig());
  auto evidence = monitor.update(motorInputs(61.0, 1.0, 2.0), 3.0, 3.0);
  ROBOT_REQUIRE((evidence.asserted_bits &
                 robot::faultBit(robot::Fault::ThermalDerate)) != 0);
  ROBOT_REQUIRE(evidence.affected_motor_mask == 0x3Fu);

  evidence = monitor.update(motorInputs(58.0, 1.0, 2.0), 3.0, 3.0);
  ROBOT_REQUIRE((evidence.asserted_bits &
                 robot::faultBit(robot::Fault::ThermalDerate)) != 0);
  evidence = monitor.update(motorInputs(54.0, 1.0, 2.0), 3.0, 3.0);
  ROBOT_REQUIRE((evidence.asserted_bits &
                 robot::faultBit(robot::Fault::ThermalDerate)) == 0);
}

ROBOT_TEST("motor protection identifies current and stall by motor mask") {
  robot::MotorProtectionMonitor monitor(protectionConfig());
  auto inputs = motorInputs(30.0, 1.0, 2.0);
  inputs.left[1].current_A = good(3.5);
  auto evidence = monitor.update(inputs, 3.0, 3.0);
  ROBOT_REQUIRE((evidence.asserted_bits &
                 robot::faultBit(robot::Fault::MotorOverCurrent)) != 0);
  ROBOT_REQUIRE((evidence.affected_motor_mask & (1u << 1)) != 0);

  inputs = motorInputs(30.0, 1.0, 2.0);
  inputs.right[2].current_A = good(3.0);
  inputs.right[2].velocity_radps = good(0.1);
  evidence = monitor.update(inputs, 3.0, 7.0);
  ROBOT_REQUIRE((evidence.asserted_bits &
                 robot::faultBit(robot::Fault::Stall)) != 0);
  ROBOT_REQUIRE((evidence.affected_motor_mask & (1u << 5)) != 0);
}

ROBOT_TEST("missing motor telemetry is unknown rather than healthy") {
  robot::MotorProtectionMonitor monitor(protectionConfig());
  auto inputs = motorInputs(30.0, 1.0, 2.0);
  inputs.left[0].temperature_C.quality = robot::Quality::Invalid;
  auto evidence = monitor.update(inputs, 3.0, 3.0);
  ROBOT_REQUIRE((evidence.known_bits &
                 robot::faultBit(robot::Fault::ThermalDerate)) == 0);
  ROBOT_REQUIRE((evidence.known_bits &
                 robot::faultBit(robot::Fault::MotorOverCurrent)) != 0);

  inputs = motorInputs(61.0, 1.0, 2.0);
  evidence = monitor.update(inputs, 3.0, 3.0);
  inputs.left[0].temperature_C.quality = robot::Quality::Invalid;
  evidence = monitor.update(inputs, 3.0, 3.0);
  ROBOT_REQUIRE((evidence.asserted_bits &
                 robot::faultBit(robot::Fault::ThermalDerate)) != 0);
}

ROBOT_TEST("fault manager debounces entry and hysteretic recovery") {
  robot::FaultManager<3> manager(faultRules());
  const robot::FaultBits thermal =
      robot::faultBit(robot::Fault::ThermalDerate);
  robot::FaultEvidence evidence{thermal, thermal, 1};
  auto summary = manager.update(evidence, 100);
  ROBOT_REQUIRE(summary.active_bits == 0);
  summary = manager.update(evidence, 1100);
  ROBOT_REQUIRE((summary.active_bits & thermal) != 0);
  ROBOT_REQUIRE((summary.entered_bits & thermal) != 0);
  ROBOT_REQUIRE_NEAR(summary.target_derate, 0.5, 1e-12);
  ROBOT_REQUIRE(summary.safety_state == robot::SafetyState::Derated);

  evidence.asserted_bits = 0;
  summary = manager.update(evidence, 1200);
  ROBOT_REQUIRE((summary.active_bits & thermal) != 0);
  summary = manager.update(evidence, 3200);
  ROBOT_REQUIRE((summary.active_bits & thermal) == 0);
  ROBOT_REQUIRE((summary.exited_bits & thermal) != 0);
}

ROBOT_TEST("unknown evidence does not silently clear an active fault") {
  robot::FaultManager<3> manager(faultRules());
  const robot::FaultBits thermal =
      robot::faultBit(robot::Fault::ThermalDerate);
  manager.update({thermal, thermal, 1}, 0);
  manager.update({thermal, thermal, 1}, 1000);
  const auto summary = manager.update({0, 0, 0}, 100000);
  ROBOT_REQUIRE((summary.active_bits & thermal) != 0);
}

ROBOT_TEST("latched stop clears only while disabled authorized and inactive") {
  robot::FaultManager<3> manager(faultRules());
  const robot::FaultBits stall = robot::faultBit(robot::Fault::Stall);
  auto summary = manager.update({stall, stall, 0x20}, 100);
  ROBOT_REQUIRE(summary.stop_required);
  ROBOT_REQUIRE((summary.latched_bits & stall) != 0);
  ROBOT_REQUIRE(manager.clearLatched(stall, driverMode(), true) == 0);
  ROBOT_REQUIRE(manager.clearLatched(stall, disabledMode(), true) == 0);

  summary = manager.update({stall, 0, 0}, 200);
  ROBOT_REQUIRE(summary.stop_required);
  ROBOT_REQUIRE(manager.clearLatched(stall, disabledMode(), false) == 0);
  ROBOT_REQUIRE(manager.clearLatched(stall, disabledMode(), true) == stall);
  summary = manager.summary();
  ROBOT_REQUIRE(!summary.stop_required);
  ROBOT_REQUIRE((summary.latched_bits & stall) == 0);
}

ROBOT_TEST("derate restriction is immediate and recovery is rate limited") {
  robot::DerateController derate({0.5, 0.1});
  double applied{};
  ROBOT_REQUIRE(derate.update(0.4, 0.1, applied));
  ROBOT_REQUIRE_NEAR(applied, 0.4, 1e-12);
  ROBOT_REQUIRE(derate.update(1.0, 0.1, applied));
  ROBOT_REQUIRE_NEAR(applied, 0.45, 1e-12);
  ROBOT_REQUIRE(derate.update(0.2, 0.1, applied));
  ROBOT_REQUIRE_NEAR(applied, 0.2, 1e-12);
}

ROBOT_TEST("fault manager rejects time regression without losing state") {
  robot::FaultManager<3> manager(faultRules());
  const robot::FaultBits stall = robot::faultBit(robot::Fault::Stall);
  manager.update({stall, stall, 1}, 100);
  const auto regressed = manager.update({stall, 0, 0}, 99);
  ROBOT_REQUIRE(!regressed.valid);
  ROBOT_REQUIRE((manager.summary().active_bits & stall) != 0);
}
