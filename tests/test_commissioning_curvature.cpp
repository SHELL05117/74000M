#include "robot/manual/commissioning_curvature.hpp"
#include "robot/config/robot_config.hpp"
#include "robot/drive/output_service.hpp"
#include "robot/platform/fake_io.hpp"
#include "test_framework.hpp"

#include <cmath>
#include <variant>

namespace {

robot::ModeSnapshot testMode(std::uint32_t epoch = 7) {
  return {robot::CompetitionMode::Test, true, false, epoch, 0, 0};
}

robot::ControllerSnapshot controllerFor(const robot::FrameHeader& header,
                                        std::uint32_t buttons = 0,
                                        double left_y = 0.0,
                                        double left_x = 0.0) {
  robot::ControllerSnapshot controller{};
  controller.h = header;
  controller.left_y = left_y;
  controller.left_x = left_x;
  controller.buttons = buttons;
  controller.connected = true;
  controller.api_ok = true;
  return controller;
}

robot::TimingSample timingFor(const robot::FrameHeader& header) {
  robot::TimingSample timing{};
  timing.h = header;
  timing.raw_dt_s = 0.01;
  timing.math_dt_s = 0.01;
  return timing;
}

robot::CommissioningCurvatureConfig directMappingConfig() {
  auto config = robot::make1690XCommissioningCurvatureConfig();
  config.throttle_shape = {0.0, 0.0, 0.0};
  config.turn_shape = {0.0, 0.0, 0.0};
  config.throttle_rise_per_s = 1000.0;
  config.throttle_fall_per_s = 1000.0;
  config.turn_rise_per_s = 1000.0;
  config.turn_fall_per_s = 1000.0;
  return config;
}

}  // namespace

ROBOT_TEST("commissioning Curvature requires ordered automatic turn gates") {
  auto config = robot::make1690XCommissioningCurvatureConfig();
  ROBOT_REQUIRE(robot::validCommissioningCurvatureConfig(config));
  config.quick_turn_exit_throttle = config.quick_turn_enter_throttle;
  ROBOT_REQUIRE(!robot::validCommissioningCurvatureConfig(config));
}

ROBOT_TEST("commissioning Curvature drives immediately and neutral coasts") {
  const auto config = robot::make1690XCommissioningCurvatureConfig();
  robot::CommissioningCurvatureMapper mapper(config);
  const robot::OwnerToken owner{42, robot::Requirement::kDrivetrain, 3, 7};
  const auto mode = testMode();

  const robot::FrameHeader start{10000, 1, 7};
  auto result = mapper.update(
      start, mode, controllerFor(start, 0, 1.0, 0.0), 0.01, owner);
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE(result.state == robot::CommissioningDriveState::Driving);
  ROBOT_REQUIRE(result.request.source == robot::RequestSource::Test);
  const auto* driving =
      std::get_if<robot::WheelVoltagePayload>(&result.request.payload);
  ROBOT_REQUIRE(driving != nullptr);
  ROBOT_REQUIRE(driving->left_V > 0.0);
  ROBOT_REQUIRE(driving->right_V > 0.0);

  const robot::FrameHeader neutral{20000, 2, 7};
  result = mapper.update(neutral, mode, controllerFor(neutral), 0.01, owner);
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE(result.state == robot::CommissioningDriveState::Coasting);
  const auto* stopped =
      std::get_if<robot::BrakePayload>(&result.request.payload);
  ROBOT_REQUIRE(stopped != nullptr);
  ROBOT_REQUIRE(stopped->mode == robot::StopMode::Coast);
}

ROBOT_TEST("commissioning Curvature scales steering with throttle") {
  const auto config = directMappingConfig();
  const robot::OwnerToken owner{42, robot::Requirement::kDrivetrain, 3, 7};
  const auto mode = testMode();

  robot::CommissioningCurvatureMapper forward_mapper(config);
  const robot::FrameHeader forward{10000, 1, 7};
  auto result = forward_mapper.update(
      forward, mode, controllerFor(forward, 0, 0.5, 0.5), 0.01, owner);
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE(!result.quick_turn_active);
  const auto* payload =
      std::get_if<robot::WheelVoltagePayload>(&result.request.payload);
  ROBOT_REQUIRE(payload != nullptr);
  ROBOT_REQUIRE_NEAR(payload->left_V, 9.0, 1e-12);
  ROBOT_REQUIRE_NEAR(payload->right_V, 3.0, 1e-12);

  robot::CommissioningCurvatureMapper reverse_mapper(config);
  const robot::FrameHeader reverse{20000, 2, 7};
  result = reverse_mapper.update(
      reverse, mode, controllerFor(reverse, 0, -0.5, 0.5), 0.01, owner);
  ROBOT_REQUIRE(result.valid);
  payload = std::get_if<robot::WheelVoltagePayload>(&result.request.payload);
  ROBOT_REQUIRE(payload != nullptr);
  ROBOT_REQUIRE_NEAR(payload->left_V, -3.0, 1e-12);
  ROBOT_REQUIRE_NEAR(payload->right_V, -9.0, 1e-12);
}

ROBOT_TEST("low commanded throttle automatically enables full Quick Turn") {
  const auto config = directMappingConfig();
  const robot::OwnerToken owner{42, robot::Requirement::kDrivetrain, 3, 7};
  const auto mode = testMode();

  robot::CommissioningCurvatureMapper quick_mapper(config);
  const robot::FrameHeader quick{10000, 1, 7};
  auto result = quick_mapper.update(
      quick, mode, controllerFor(quick, 0, 0.0, 1.0), 0.01, owner);
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE(result.quick_turn_active);
  const auto* payload =
      std::get_if<robot::WheelVoltagePayload>(&result.request.payload);
  ROBOT_REQUIRE(payload != nullptr);
  ROBOT_REQUIRE_NEAR(payload->left_V, 12.0, 1e-12);
  ROBOT_REQUIRE_NEAR(payload->right_V, -12.0, 1e-12);

  robot::CommissioningCurvatureMapper high_throttle_mapper(config);
  const robot::FrameHeader high{20000, 2, 7};
  result = high_throttle_mapper.update(
      high, mode, controllerFor(high, 0, 0.5, 0.5), 0.01, owner);
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE(!result.quick_turn_active);
  payload = std::get_if<robot::WheelVoltagePayload>(&result.request.payload);
  ROBOT_REQUIRE(payload != nullptr);
  ROBOT_REQUIRE_NEAR(payload->left_V, 9.0, 1e-12);
  ROBOT_REQUIRE_NEAR(payload->right_V, 3.0, 1e-12);
}

ROBOT_TEST("automatic Quick Turn uses throttle hysteresis") {
  const auto config = directMappingConfig();
  robot::CommissioningCurvatureMapper mapper(config);
  const robot::OwnerToken owner{42, robot::Requirement::kDrivetrain, 3, 7};
  const auto mode = testMode();

  const robot::FrameHeader enter{10000, 1, 7};
  auto result = mapper.update(
      enter, mode, controllerFor(enter, 0, 0.10, 0.5), 0.01, owner);
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE(result.quick_turn_active);

  const robot::FrameHeader hold{20000, 2, 7};
  result = mapper.update(
      hold, mode, controllerFor(hold, 0, 0.20, 0.5), 0.01, owner);
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE(result.quick_turn_active);

  const robot::FrameHeader exit{30000, 3, 7};
  result = mapper.update(
      exit, mode, controllerFor(exit, 0, 0.30, 0.5), 0.01, owner);
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE(!result.quick_turn_active);

  const robot::FrameHeader outside_enter{40000, 4, 7};
  result = mapper.update(
      outside_enter, mode,
      controllerFor(outside_enter, 0, 0.20, 0.5), 0.01, owner);
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE(!result.quick_turn_active);

  const robot::FrameHeader reenter{50000, 5, 7};
  result = mapper.update(
      reenter, mode, controllerFor(reenter, 0, 0.10, 0.5), 0.01,
      owner);
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE(result.quick_turn_active);
}

ROBOT_TEST("Coast clears automatic Quick Turn hysteresis memory") {
  const auto config = directMappingConfig();
  robot::CommissioningCurvatureMapper mapper(config);
  const robot::OwnerToken owner{42, robot::Requirement::kDrivetrain, 3, 7};
  const auto mode = testMode();

  const robot::FrameHeader enter{10000, 1, 7};
  auto result = mapper.update(
      enter, mode, controllerFor(enter, 0, 0.10, 0.5), 0.01, owner);
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE(result.quick_turn_active);

  const robot::FrameHeader coast{20000, 2, 7};
  result = mapper.update(
      coast, mode,
      controllerFor(coast, config.coast_button, 0.10, 0.5), 0.01,
      owner);
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE(result.state == robot::CommissioningDriveState::Coasting);

  const robot::FrameHeader resume{30000, 3, 7};
  result = mapper.update(
      resume, mode, controllerFor(resume, 0, 0.20, 0.5), 0.01, owner);
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE(!result.quick_turn_active);
}

ROBOT_TEST("commissioning Curvature keeps wheel ratio when saturated") {
  const auto config = directMappingConfig();
  robot::CommissioningCurvatureMapper mapper(config);
  const robot::OwnerToken owner{42, robot::Requirement::kDrivetrain, 3, 7};
  const auto mode = testMode();
  const robot::FrameHeader header{10000, 1, 7};
  const auto result = mapper.update(
      header, mode, controllerFor(header, 0, 1.0, 1.0), 0.01, owner);
  ROBOT_REQUIRE(result.valid);
  const auto* payload =
      std::get_if<robot::WheelVoltagePayload>(&result.request.payload);
  ROBOT_REQUIRE(payload != nullptr);
  ROBOT_REQUIRE_NEAR(payload->left_V, 12.0, 1e-12);
  ROBOT_REQUIRE_NEAR(payload->right_V, 0.0, 1e-12);
}

ROBOT_TEST("commissioning cycle drives through Test safety gate at twelve volts") {
  const auto config = robot::make1690XCommissioningCurvatureConfig();
  ROBOT_REQUIRE_NEAR(config.max_voltage_V, 12.0, 1e-12);
  ROBOT_REQUIRE_NEAR(config.throttle_rise_per_s, 20.0, 1e-12);
  ROBOT_REQUIRE_NEAR(config.throttle_fall_per_s, 20.0, 1e-12);
  ROBOT_REQUIRE_NEAR(config.turn_rise_per_s, 20.0, 1e-12);
  ROBOT_REQUIRE_NEAR(config.turn_fall_per_s, 20.0, 1e-12);
  ROBOT_REQUIRE_NEAR(config.output_slew.rise_V_per_s, 240.0, 1e-12);
  ROBOT_REQUIRE_NEAR(config.output_slew.fall_V_per_s, 240.0, 1e-12);
  robot::CommissioningControlCycle cycle(config);
  const auto mode = testMode();
  robot::RawDriveInputs raw{};

  const robot::FrameHeader start{10000, 1, 7};
  auto frame = cycle.update(start, mode, raw,
                            controllerFor(start, 0, 1.0, 0.0),
                            timingFor(start));
  ROBOT_REQUIRE(frame.owner == robot::RequestSource::Test);
  ROBOT_REQUIRE_NEAR(frame.left_V, 2.4, 1e-9);
  ROBOT_REQUIRE_NEAR(frame.right_V, 2.4, 1e-9);
  ROBOT_REQUIRE(std::abs(frame.left_V) <= 12.0);
  ROBOT_REQUIRE(std::abs(frame.right_V) <= 12.0);

  for (std::uint32_t i = 0; i < 3; ++i) {
    const robot::FrameHeader full{20000 + i * 10000, 2 + i, 7};
    frame = cycle.update(full, mode, raw,
                         controllerFor(full, 0, 1.0, 0.0),
                         timingFor(full));
  }
  ROBOT_REQUIRE_NEAR(frame.left_V, 9.6, 1e-9);
  ROBOT_REQUIRE_NEAR(frame.right_V, 9.6, 1e-9);

  const robot::FrameHeader full{50000, 5, 7};
  frame = cycle.update(full, mode, raw,
                       controllerFor(full, 0, 1.0, 0.0),
                       timingFor(full));
  ROBOT_REQUIRE_NEAR(frame.left_V, 12.0, 1e-9);
  ROBOT_REQUIRE_NEAR(frame.right_V, 12.0, 1e-9);
}

ROBOT_TEST("automatic Quick Turn reaches full voltage in fifty milliseconds") {
  const auto config = robot::make1690XCommissioningCurvatureConfig();
  robot::CommissioningControlCycle cycle(config);
  const auto mode = testMode();
  robot::RawDriveInputs raw{};

  robot::ActuatorFrame frame{};
  for (std::uint32_t i = 0; i < 5; ++i) {
    const robot::FrameHeader header{10000 + i * 10000, 1 + i, 7};
    frame = cycle.update(header, mode, raw,
                         controllerFor(header, 0, 0.0, 1.0),
                         timingFor(header));
    ROBOT_REQUIRE(frame.owner == robot::RequestSource::Test);
    ROBOT_REQUIRE(frame.left_V > 0.0);
    ROBOT_REQUIRE(frame.right_V < 0.0);
  }
  ROBOT_REQUIRE_NEAR(frame.left_V, 12.0, 1e-9);
  ROBOT_REQUIRE_NEAR(frame.right_V, -12.0, 1e-9);
}

ROBOT_TEST("B and every commissioning stop path use nonlatched Coast") {
  const auto config = robot::make1690XCommissioningCurvatureConfig();
  robot::CommissioningControlCycle cycle(config);
  robot::RawDriveInputs raw{};
  const auto mode = testMode();

  const robot::FrameHeader drive{10000, 1, 7};
  auto frame = cycle.update(drive, mode, raw,
                            controllerFor(drive, 0, 1.0, 0.0),
                            timingFor(drive));
  ROBOT_REQUIRE(frame.left_V > 0.0);
  ROBOT_REQUIRE(frame.right_V > 0.0);

  const robot::FrameHeader coast{20000, 2, 7};
  frame = cycle.update(coast, mode, raw,
                       controllerFor(coast, config.coast_button, 1.0, 0.0),
                       timingFor(coast));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(frame.right_V, 0.0, 1e-12);
  ROBOT_REQUIRE(frame.zero_behavior == robot::StopMode::Coast);
  ROBOT_REQUIRE(cycle.state() == robot::CommissioningDriveState::Coasting);

  const robot::FrameHeader resumed{30000, 3, 7};
  frame = cycle.update(resumed, mode, raw,
                       controllerFor(resumed, 0, 1.0, 0.0),
                       timingFor(resumed));
  ROBOT_REQUIRE(frame.left_V > 0.0);
  ROBOT_REQUIRE(frame.right_V > 0.0);
  ROBOT_REQUIRE(cycle.state() == robot::CommissioningDriveState::Driving);

  const robot::FrameHeader neutral{40000, 4, 7};
  frame = cycle.update(neutral, mode, raw, controllerFor(neutral),
                       timingFor(neutral));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(frame.right_V, 0.0, 1e-12);
  ROBOT_REQUIRE(frame.zero_behavior == robot::StopMode::Coast);

  auto field_mode = mode;
  field_mode.field_connected = true;
  const robot::FrameHeader field{50000, 5, 7};
  frame = cycle.update(field, field_mode, raw,
                       controllerFor(field, 0, 1.0, 0.0), timingFor(field));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(frame.right_V, 0.0, 1e-12);
  ROBOT_REQUIRE(frame.zero_behavior == robot::StopMode::Coast);

  const robot::FrameHeader disconnected{60000, 6, 7};
  auto disconnected_controller = controllerFor(disconnected, 0, 1.0, 0.0);
  disconnected_controller.connected = false;
  frame = cycle.update(disconnected, mode, raw, disconnected_controller,
                       timingFor(disconnected));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(frame.right_V, 0.0, 1e-12);
  ROBOT_REQUIRE(frame.zero_behavior == robot::StopMode::Coast);

  auto disabled_mode = mode;
  disabled_mode.enabled = false;
  const robot::FrameHeader disabled{70000, 7, 7};
  frame = cycle.update(disabled, disabled_mode, raw,
                       controllerFor(disabled, 0, 1.0, 0.0),
                       timingFor(disabled));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(frame.right_V, 0.0, 1e-12);
  ROBOT_REQUIRE(frame.zero_behavior == robot::StopMode::Coast);
}

ROBOT_TEST("commissioning output watchdog uses Coast for stale frames") {
  const auto robot_config = robot::make1690XCommissioningConfig();
  robot::FakeDriveIO io(robot_config.hardware, {1.0, 12.0});
  ROBOT_REQUIRE(io.initialize());
  robot::OutputService output(
      io, {robot_config.runtime.output_ttl_us,
           robot_config.electrical.max_command_voltage_V, 1e-9,
           robot::kCommissioningStopMode});
  const auto mode = testMode();
  const auto result = output.tick(mode, nullptr, 10000);
  ROBOT_REQUIRE(result.action == robot::OutputAction::Stopped);
  ROBOT_REQUIRE((result.reject_bits & robot::kOutputNoFrame) != 0);
  ROBOT_REQUIRE(io.lastStopMode() == robot::StopMode::Coast);
}
