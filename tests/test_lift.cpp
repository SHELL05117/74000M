#include "robot/drive/output_service.hpp"
#include "robot/lift/lift_control.hpp"
#include "robot/manual/commissioning_curvature.hpp"
#include "robot/platform/fake_io.hpp"
#include "test_framework.hpp"

#include <cmath>

namespace {

robot::ModeSnapshot testMode(std::uint32_t epoch = 4) {
  return {robot::CompetitionMode::Test, true, false, epoch, 0, 0};
}

robot::ControllerSnapshot controllerAt(const robot::FrameHeader& header,
                                       double right_y) {
  robot::ControllerSnapshot controller{};
  controller.h = header;
  controller.right_y = right_y;
  controller.connected = true;
  controller.api_ok = true;
  return controller;
}

std::array<robot::MotorSample, robot::kLiftMotorCount> liftAt(
    const robot::LiftHardwareConfig& hardware, robot::TimeUs time_us,
    double first_deg, double second_deg) {
  std::array<robot::MotorSample, robot::kLiftMotorCount> raw{};
  const double values[] = {robot::units::degreesToRadians(first_deg),
                           robot::units::degreesToRadians(second_deg)};
  for (std::size_t i = 0; i < raw.size(); ++i) {
    raw[i].smart_port =
        static_cast<std::uint8_t>(hardware.motors[i].smart_port);
    raw[i].position_rad = {values[i], time_us, true, 0};
  }
  return raw;
}

robot::TimingSample timingAt(const robot::FrameHeader& header) {
  robot::TimingSample timing{};
  timing.h = header;
  timing.raw_dt_s = 0.01;
  timing.math_dt_s = 0.01;
  return timing;
}

}  // namespace

ROBOT_TEST("Lift Axis2 mapping is proportional after the neutral deadband") {
  const auto robot_config = robot::make1690XCommissioningConfig();
  const auto lift_config = robot::make1690XLiftCommissioningConfig();
  robot::LiftManualMapper mapper(lift_config, robot_config.hardware.lift);
  const robot::OwnerToken owner{
      42, robot::Requirement::kDrivetrain | robot::Requirement::kLift, 1, 4};
  const robot::FrameHeader header{10000, 1, 4};
  robot::LiftRequest request{};

  ROBOT_REQUIRE(mapper.update(header, testMode(), controllerAt(header, 0.5),
                              owner, request));
  ROBOT_REQUIRE_NEAR(request.normalized_output,
                     (0.5 - lift_config.axis_shape.deadband) /
                         (1.0 - lift_config.axis_shape.deadband),
                     1e-12);

  ROBOT_REQUIRE(mapper.update(header, testMode(), controllerAt(header, -1.0),
                              owner, request));
  ROBOT_REQUIRE_NEAR(request.normalized_output, -1.0, 1e-12);
}

ROBOT_TEST("Lift gate slows before the endpoint and blocks both hard limits") {
  const auto robot_config = robot::make1690XCommissioningConfig();
  const auto lift_config = robot::make1690XLiftCommissioningConfig();
  const auto& hardware = robot_config.hardware.lift;
  robot::LiftSafetyGate gate(lift_config, hardware);
  const robot::OwnerToken owner{
      42, robot::Requirement::kDrivetrain | robot::Requirement::kLift, 1, 4};
  robot::LiftRequest request{{10000, 1, 4}, robot::RequestSource::Test,
                             owner, 1.0, 30000};

  auto result = gate.apply(&request, liftAt(hardware, 10000, 400.0, 400.0),
                           testMode(), 10000);
  ROBOT_REQUIRE(result.position_valid);
  ROBOT_REQUIRE_NEAR(result.voltage_V, 12.0, 1e-12);

  result = gate.apply(&request, liftAt(hardware, 10000, 800.0, 800.0),
                      testMode(), 10000);
  ROBOT_REQUIRE((result.limit_bits & robot::kLiftSlowdownApplied) != 0);
  ROBOT_REQUIRE_NEAR(result.applied_scale, 20.0 / 90.0, 1e-12);

  result = gate.apply(&request, liftAt(hardware, 10000, 819.0, 820.0),
                      testMode(), 10000);
  ROBOT_REQUIRE((result.limit_bits & robot::kLiftUpperLimit) != 0);
  ROBOT_REQUIRE_NEAR(result.voltage_V, 0.0, 0.0);

  request.normalized_output = -1.0;
  result = gate.apply(&request, liftAt(hardware, 10000, 0.0, 0.0),
                      testMode(), 10000);
  ROBOT_REQUIRE((result.limit_bits & robot::kLiftLowerLimit) != 0);
  ROBOT_REQUIRE_NEAR(result.voltage_V, 0.0, 0.0);
}

ROBOT_TEST("Lift gate holds on encoder disagreement or stale position") {
  const auto robot_config = robot::make1690XCommissioningConfig();
  const auto lift_config = robot::make1690XLiftCommissioningConfig();
  const auto& hardware = robot_config.hardware.lift;
  robot::LiftSafetyGate gate(lift_config, hardware);
  const robot::OwnerToken owner{
      42, robot::Requirement::kDrivetrain | robot::Requirement::kLift, 1, 4};
  const robot::LiftRequest request{{50000, 1, 4},
                                   robot::RequestSource::Test, owner, 0.8,
                                   30000};

  auto result = gate.apply(&request, liftAt(hardware, 50000, 200.0, 230.0),
                           testMode(), 50000);
  ROBOT_REQUIRE((result.limit_bits & robot::kLiftPositionDisagreement) != 0);
  ROBOT_REQUIRE(result.zero_behavior == robot::StopMode::Hold);

  result = gate.apply(&request, liftAt(hardware, 10000, 200.0, 200.0),
                      testMode(), 50000);
  ROBOT_REQUIRE((result.limit_bits & robot::kLiftBadPosition) != 0);
  ROBOT_REQUIRE_NEAR(result.voltage_V, 0.0, 0.0);
}

ROBOT_TEST("commissioning cycle and single output writer carry Lift voltage") {
  const auto robot_config = robot::make1690XCommissioningConfig();
  const auto lift_config = robot::make1690XLiftCommissioningConfig();
  robot::CommissioningControlCycle cycle(
      robot::make1690XCommissioningCurvatureConfig(), lift_config,
      robot_config.hardware.lift);
  const robot::FrameHeader header{10000, 1, 4};
  robot::RawDriveInputs raw{};
  raw.lift = liftAt(robot_config.hardware.lift, header.time_us, 300.0, 300.0);

  const robot::ActuatorFrame frame =
      cycle.update(header, testMode(), raw, controllerAt(header, 1.0),
                   timingAt(header));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 0.0);
  ROBOT_REQUIRE_NEAR(frame.right_V, 0.0, 0.0);
  ROBOT_REQUIRE_NEAR(frame.lift_V, 12.0, 1e-12);
  ROBOT_REQUIRE(frame.lift_zero_behavior == robot::StopMode::Hold);

  robot::FakeDriveIO io(robot_config.hardware, {1.0, 12.0});
  ROBOT_REQUIRE(io.initialize());
  robot::OutputService output(
      io, {robot_config.runtime.output_ttl_us,
           robot_config.electrical.max_command_voltage_V, 1e-9,
           robot::StopMode::Coast, robot::StopMode::Hold});
  const auto result = output.tick(testMode(), &frame, header.time_us);
  ROBOT_REQUIRE(result.action == robot::OutputAction::WroteVoltage);
  ROBOT_REQUIRE(result.io_ok);
  ROBOT_REQUIRE(io.writeCount() == 0);
  ROBOT_REQUIRE(io.stopCount() == 1);
  ROBOT_REQUIRE(io.liftWriteCount() == 1);
  ROBOT_REQUIRE_NEAR(io.liftCommandV(), 12.0, 1e-12);
}
