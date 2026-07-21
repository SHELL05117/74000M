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
  auto config = robot::makeCommissioningCurvatureConfig();
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
  auto config = robot::makeCommissioningCurvatureConfig();
  ROBOT_REQUIRE(robot::validCommissioningCurvatureConfig(config));
  config.quick_turn_exit_throttle = config.quick_turn_enter_throttle;
  ROBOT_REQUIRE(!robot::validCommissioningCurvatureConfig(config));
}

ROBOT_TEST("commissioning Curvature drives immediately and neutral coasts") {
  const auto config = robot::makeCommissioningCurvatureConfig();
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
  mapper.requestOneShotCoast(coast.mode_epoch);
  result = mapper.update(
      coast, mode, controllerFor(coast, 0, 0.10, 0.5), 0.01,
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
  const auto config = robot::makeCommissioningCurvatureConfig();
  ROBOT_REQUIRE_NEAR(config.max_voltage_V, 12.0, 1e-12);
  ROBOT_REQUIRE_NEAR(config.throttle_shape.cubic_weight, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(config.turn_shape.cubic_weight, 0.15, 1e-12);
  ROBOT_REQUIRE_NEAR(config.throttle_rise_per_s, 100.0, 1e-12);
  ROBOT_REQUIRE_NEAR(config.throttle_fall_per_s, 100.0, 1e-12);
  ROBOT_REQUIRE_NEAR(config.turn_rise_per_s, 100.0, 1e-12);
  ROBOT_REQUIRE_NEAR(config.turn_fall_per_s, 100.0, 1e-12);
  ROBOT_REQUIRE_NEAR(config.output_slew.rise_V_per_s, 1200.0, 1e-12);
  ROBOT_REQUIRE_NEAR(config.output_slew.fall_V_per_s, 1200.0, 1e-12);
  robot::CommissioningControlCycle cycle(config);
  const auto mode = testMode();
  robot::RawDriveInputs raw{};

  const robot::FrameHeader start{10000, 1, 7};
  auto frame = cycle.update(start, mode, raw,
                            controllerFor(start, 0, 1.0, 0.0),
                            timingFor(start));
  ROBOT_REQUIRE(frame.owner == robot::RequestSource::Test);
  ROBOT_REQUIRE_NEAR(frame.left_V, 12.0, 1e-9);
  ROBOT_REQUIRE_NEAR(frame.right_V, 12.0, 1e-9);
  ROBOT_REQUIRE(std::abs(frame.left_V) <= 12.0);
  ROBOT_REQUIRE(std::abs(frame.right_V) <= 12.0);
}

ROBOT_TEST("commissioning log preserves mapping arbitration and final intent") {
  const auto config = directMappingConfig();
  robot::CommissioningControlCycle cycle(config);
  const auto mode = testMode();
  robot::RawDriveInputs raw{};
  const robot::FrameHeader header{10000, 1, 7};
  const auto actuator = cycle.update(
      header, mode, raw, controllerFor(header, 0, 0.10, 0.50),
      timingFor(header));
  robot::LogFrame log = robot::makeControlLogFrame(
      header, 42, mode, raw,
      controllerFor(header, 0, 0.10, 0.50), actuator,
      timingFor(header), 0, 0, header.time_us);
  cycle.populateLogFrame(log);

  ROBOT_REQUIRE(
      (log.trace.availability_bits & robot::kTraceDriverMapping) != 0);
  ROBOT_REQUIRE(
      (log.trace.availability_bits & robot::kTraceRequestCandidate) != 0);
  ROBOT_REQUIRE(
      (log.trace.availability_bits & robot::kTraceArbitration) != 0);
  ROBOT_REQUIRE(log.trace.request_candidate_present);
  ROBOT_REQUIRE(log.trace.request_selected);
  ROBOT_REQUIRE(log.trace.arbitration_reject_bits == 0);
  ROBOT_REQUIRE(log.trace.arbitration_rejected_count == 0);
  ROBOT_REQUIRE(log.trace.selected_request_sequence == header.sequence);
  ROBOT_REQUIRE(log.trace.selected_source ==
                static_cast<std::uint8_t>(robot::RequestSource::Test));
  ROBOT_REQUIRE(log.trace.quick_turn_active);
  ROBOT_REQUIRE_NEAR(log.trace.mapped_throttle, 0.10, 1e-12);
  ROBOT_REQUIRE_NEAR(log.trace.mapped_turn, 0.50, 1e-12);
  ROBOT_REQUIRE_NEAR(log.request.forward, 0.10, 1e-12);
  ROBOT_REQUIRE_NEAR(log.request.steering, 0.50, 1e-12);
  ROBOT_REQUIRE(log.request.reject_bits == 0);
  ROBOT_REQUIRE(log.timing.request_age_us == 0);
  ROBOT_REQUIRE(log.trace.stop_mode ==
                static_cast<std::uint8_t>(robot::StopMode::Coast));
  ROBOT_REQUIRE_NEAR(log.actuator.final_left_V, actuator.left_V, 1e-12);
  ROBOT_REQUIRE_NEAR(log.actuator.final_right_V, actuator.right_V, 1e-12);
  ROBOT_REQUIRE_NEAR(log.actuator.derate_applied, 1.0, 1e-12);
}

ROBOT_TEST("linear throttle preserves near-endpoint voltage") {
  const auto config = robot::makeCommissioningCurvatureConfig();
  robot::CommissioningCurvatureMapper mapper(config);
  const robot::OwnerToken owner{42, robot::Requirement::kDrivetrain, 3, 7};
  const auto mode = testMode();
  const robot::FrameHeader header{10000, 1, 7};
  constexpr double kOriginalAxis = 120.0 / 127.0;
  const auto result = mapper.update(
      header, mode, controllerFor(header, 0, kOriginalAxis, 0.0), 0.01,
      owner);
  ROBOT_REQUIRE(result.valid);
  const auto* payload =
      std::get_if<robot::WheelVoltagePayload>(&result.request.payload);
  ROBOT_REQUIRE(payload != nullptr);
  const double remapped =
      (kOriginalAxis - config.throttle_shape.deadband) /
      (1.0 - config.throttle_shape.deadband);
  ROBOT_REQUIRE_NEAR(payload->left_V, remapped * 12.0, 1e-9);
  ROBOT_REQUIRE_NEAR(payload->right_V, remapped * 12.0, 1e-9);
  ROBOT_REQUIRE(payload->left_V > 11.29);
}

ROBOT_TEST("automatic Quick Turn reaches full voltage in one nominal frame") {
  const auto config = robot::makeCommissioningCurvatureConfig();
  robot::CommissioningControlCycle cycle(config);
  const auto mode = testMode();
  robot::RawDriveInputs raw{};

  const robot::FrameHeader header{10000, 1, 7};
  const auto frame = cycle.update(header, mode, raw,
                                  controllerFor(header, 0, 0.0, 1.0),
                                  timingFor(header));
  ROBOT_REQUIRE(frame.owner == robot::RequestSource::Test);
  ROBOT_REQUIRE_NEAR(frame.left_V, 12.0, 1e-9);
  ROBOT_REQUIRE_NEAR(frame.right_V, -12.0, 1e-9);
}

ROBOT_TEST("Left produces one Coast frame and held Left does not latch") {
  const auto config = robot::makeCommissioningCurvatureConfig();
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
  cycle.acceptGlobalEvent(
      {coast, 1, robot::kGlobalCoastOnce});
  frame = cycle.update(coast, mode, raw,
                       controllerFor(coast, robot::kButtonLeft, 1.0, 0.0),
                       timingFor(coast));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(frame.right_V, 0.0, 1e-12);
  ROBOT_REQUIRE(frame.zero_behavior == robot::StopMode::Coast);
  ROBOT_REQUIRE(cycle.state() == robot::CommissioningDriveState::Coasting);

  const robot::FrameHeader held{30000, 3, 7};
  cycle.acceptGlobalEvent({held, 1, robot::kGlobalControlNoEvent});
  frame = cycle.update(held, mode, raw,
                       controllerFor(held, robot::kButtonLeft, 1.0, 0.0),
                       timingFor(held));
  ROBOT_REQUIRE(frame.left_V > 0.0);
  ROBOT_REQUIRE(frame.right_V > 0.0);
  ROBOT_REQUIRE(cycle.state() == robot::CommissioningDriveState::Driving);

  const robot::FrameHeader resumed{40000, 4, 7};
  frame = cycle.update(resumed, mode, raw,
                       controllerFor(resumed, 0, 1.0, 0.0),
                       timingFor(resumed));
  ROBOT_REQUIRE(frame.left_V > 0.0);
  ROBOT_REQUIRE(frame.right_V > 0.0);
  ROBOT_REQUIRE(cycle.state() == robot::CommissioningDriveState::Driving);

  const robot::FrameHeader neutral{50000, 5, 7};
  frame = cycle.update(neutral, mode, raw, controllerFor(neutral),
                       timingFor(neutral));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(frame.right_V, 0.0, 1e-12);
  ROBOT_REQUIRE(frame.zero_behavior == robot::StopMode::Coast);

  auto field_mode = mode;
  field_mode.field_connected = true;
  const robot::FrameHeader field{60000, 6, 7};
  frame = cycle.update(field, field_mode, raw,
                       controllerFor(field, 0, 1.0, 0.0), timingFor(field));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(frame.right_V, 0.0, 1e-12);
  ROBOT_REQUIRE(frame.zero_behavior == robot::StopMode::Coast);

  const robot::FrameHeader disconnected{70000, 7, 7};
  auto disconnected_controller = controllerFor(disconnected, 0, 1.0, 0.0);
  disconnected_controller.connected = false;
  frame = cycle.update(disconnected, mode, raw, disconnected_controller,
                       timingFor(disconnected));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(frame.right_V, 0.0, 1e-12);
  ROBOT_REQUIRE(frame.zero_behavior == robot::StopMode::Coast);

  auto disabled_mode = mode;
  disabled_mode.enabled = false;
  const robot::FrameHeader disabled{80000, 8, 7};
  frame = cycle.update(disabled, disabled_mode, raw,
                       controllerFor(disabled, 0, 1.0, 0.0),
                       timingFor(disabled));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(frame.right_V, 0.0, 1e-12);
  ROBOT_REQUIRE(frame.zero_behavior == robot::StopMode::Coast);
}

ROBOT_TEST("global Left detector emits once and requires release after invalid input") {
  robot::GlobalControlEventDetector detector;
  const robot::FrameHeader released{10000, 1, 7};
  auto event = detector.observe(released, controllerFor(released));
  ROBOT_REQUIRE(event.event_bits == robot::kGlobalControlNoEvent);

  const robot::FrameHeader pressed{20000, 2, 7};
  event = detector.observe(
      pressed, controllerFor(pressed, robot::kButtonLeft));
  ROBOT_REQUIRE(event.event_bits == robot::kGlobalCoastOnce);
  ROBOT_REQUIRE(event.event_sequence == 1);

  const robot::FrameHeader held{30000, 3, 7};
  event = detector.observe(
      held, controllerFor(held, robot::kButtonLeft));
  ROBOT_REQUIRE(event.event_bits == robot::kGlobalControlNoEvent);

  auto invalid = controllerFor({40000, 4, 7}, robot::kButtonLeft);
  invalid.connected = false;
  invalid.api_ok = false;
  event = detector.observe(invalid.h, invalid);
  ROBOT_REQUIRE(event.event_bits == robot::kGlobalControlNoEvent);

  const robot::FrameHeader reconnect_held{50000, 5, 7};
  event = detector.observe(
      reconnect_held,
      controllerFor(reconnect_held, robot::kButtonLeft));
  ROBOT_REQUIRE(event.event_bits == robot::kGlobalControlNoEvent);
  const robot::FrameHeader rearm{60000, 6, 7};
  detector.observe(rearm, controllerFor(rearm));
  const robot::FrameHeader press_again{70000, 7, 7};
  event = detector.observe(
      press_again,
      controllerFor(press_again, robot::kButtonLeft));
  ROBOT_REQUIRE(event.event_bits == robot::kGlobalCoastOnce);
  ROBOT_REQUIRE(event.event_sequence == 2);
}

ROBOT_TEST("commissioning output watchdog uses Coast for stale frames") {
  const auto robot_config = robot::makeSelectedRobotConfig();
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
