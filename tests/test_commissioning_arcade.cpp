#include "robot/manual/commissioning_arcade.hpp"
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

}  // namespace

ROBOT_TEST("commissioning Arcade requires neutral hold and chord release") {
  const auto config = robot::make1690XCommissioningArcadeConfig();
  robot::CommissioningArcadeMapper mapper(config);
  const robot::OwnerToken owner{42, robot::Requirement::kDrivetrain, 3, 7};
  const auto mode = testMode();

  const robot::FrameHeader start{10000, 1, 7};
  auto result = mapper.update(start, mode,
                              controllerFor(start, config.arm_chord), 0.01,
                              owner);
  ROBOT_REQUIRE(!result.valid);
  ROBOT_REQUIRE(result.state ==
                robot::CommissioningDriveState::HoldingArmChord);

  const robot::FrameHeader held{1010000, 2, 7};
  result = mapper.update(held, mode,
                         controllerFor(held, config.arm_chord), 0.01, owner);
  ROBOT_REQUIRE(!result.valid);
  ROBOT_REQUIRE(result.state ==
                robot::CommissioningDriveState::AwaitingChordRelease);

  const robot::FrameHeader released{1020000, 3, 7};
  result = mapper.update(released, mode, controllerFor(released), 0.01, owner);
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE(result.state == robot::CommissioningDriveState::Armed);
  ROBOT_REQUIRE(result.request.source == robot::RequestSource::Test);
  const auto* stopped =
      std::get_if<robot::WheelVoltagePayload>(&result.request.payload);
  ROBOT_REQUIRE(stopped != nullptr);
  ROBOT_REQUIRE_NEAR(stopped->left_V, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(stopped->right_V, 0.0, 1e-12);
}

ROBOT_TEST("commissioning cycle drives through Test safety gate at twelve volts") {
  const auto config = robot::make1690XCommissioningArcadeConfig();
  ROBOT_REQUIRE_NEAR(config.max_voltage_V, 12.0, 1e-12);
  ROBOT_REQUIRE_NEAR(config.output_slew.rise_V_per_s, 36.0, 1e-12);
  ROBOT_REQUIRE_NEAR(config.output_slew.fall_V_per_s, 72.0, 1e-12);
  robot::CommissioningControlCycle cycle(config);
  const auto mode = testMode();
  robot::RawDriveInputs raw{};

  const robot::FrameHeader start{10000, 1, 7};
  auto frame = cycle.update(start, mode, raw,
                            controllerFor(start, config.arm_chord),
                            timingFor(start));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(frame.right_V, 0.0, 1e-12);

  const robot::FrameHeader held{1010000, 2, 7};
  frame = cycle.update(held, mode, raw,
                       controllerFor(held, config.arm_chord),
                       timingFor(held));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 1e-12);

  const robot::FrameHeader released{1020000, 3, 7};
  frame = cycle.update(released, mode, raw, controllerFor(released),
                       timingFor(released));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 1e-12);

  const robot::FrameHeader drive{1030000, 4, 7};
  frame = cycle.update(drive, mode, raw,
                       controllerFor(drive, 0, 1.0, 0.0),
                       timingFor(drive));
  ROBOT_REQUIRE(frame.owner == robot::RequestSource::Test);
  ROBOT_REQUIRE(frame.left_V > 0.0);
  ROBOT_REQUIRE(frame.right_V > 0.0);
  ROBOT_REQUIRE(std::abs(frame.left_V) <= 12.0);
  ROBOT_REQUIRE(std::abs(frame.right_V) <= 12.0);

  for (std::uint32_t i = 0; i < 40; ++i) {
    const robot::FrameHeader full{1040000 + i * 10000, 5 + i, 7};
    frame = cycle.update(full, mode, raw,
                         controllerFor(full, 0, 1.0, 0.0),
                         timingFor(full));
  }
  ROBOT_REQUIRE_NEAR(frame.left_V, 12.0, 1e-9);
  ROBOT_REQUIRE_NEAR(frame.right_V, 12.0, 1e-9);
}

ROBOT_TEST("B and field connection lock commissioning output") {
  const auto config = robot::make1690XCommissioningArcadeConfig();
  robot::CommissioningControlCycle cycle(config);
  robot::RawDriveInputs raw{};
  const auto mode = testMode();

  const robot::FrameHeader start{10000, 1, 7};
  cycle.update(start, mode, raw, controllerFor(start, config.arm_chord),
               timingFor(start));
  const robot::FrameHeader held{1010000, 2, 7};
  cycle.update(held, mode, raw, controllerFor(held, config.arm_chord),
               timingFor(held));
  const robot::FrameHeader released{1020000, 3, 7};
  cycle.update(released, mode, raw, controllerFor(released),
               timingFor(released));

  const robot::FrameHeader emergency{1030000, 4, 7};
  auto frame = cycle.update(
      emergency, mode, raw,
      controllerFor(emergency, config.emergency_stop_button, 1.0, 0.0),
      timingFor(emergency));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(frame.right_V, 0.0, 1e-12);
  ROBOT_REQUIRE(cycle.state() ==
                robot::CommissioningDriveState::LatchedStop);

  auto field_mode = mode;
  field_mode.field_connected = true;
  const robot::FrameHeader field{1040000, 5, 7};
  frame = cycle.update(field, field_mode, raw,
                       controllerFor(field, 0, 1.0, 0.0), timingFor(field));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(frame.right_V, 0.0, 1e-12);
}
