#include "robot/manual/manual_drive.hpp"
#include "test_framework.hpp"

#include <cmath>

namespace {

robot::ManualDriveConfig manualConfig() {
  robot::ManualDriveConfig config{};
  config.throttle_axis = robot::ControllerAxis::RightY;
  config.turn_axis = robot::ControllerAxis::RightX;
  config.throttle_shape = {0.0, 0.05, 0.0};
  config.turn_shape = {0.0, 0.05, 0.0};
  config.throttle_rise_per_s = 100.0;
  config.throttle_fall_per_s = 100.0;
  config.turn_rise_per_s = 100.0;
  config.turn_fall_per_s = 100.0;
  config.max_dt_s = 0.05;
  config.curvature_gain = 1.0;
  config.quick_turn_gain = 1.0;
  config.quick_turn_max_throttle = 0.2;
  config.quick_turn_max_speed_mps = 0.3;
  config.quick_turn_button = robot::kButtonR1;
  config.state_ttl_us = 20000;
  config.request_ttl_us = 30000;
  config.heading = {true,
                    0.02,
                    0.04,
                    0.2,
                    0.5,
                    0.3,
                    0.02,
                    1.0,
                    0.0,
                    0.3,
                    robot::Quality::Good,
                    robot::Quality::Good};
  return config;
}

robot::ModeSnapshot driverMode(std::uint32_t epoch = 1) {
  return {robot::CompetitionMode::Driver, true, false, epoch, 0, 0};
}

robot::ControllerSnapshot controllerAt(const robot::FrameHeader& header,
                                       double throttle, double turn,
                                       std::uint32_t buttons = 0) {
  robot::ControllerSnapshot controller{};
  controller.h = header;
  controller.right_y = throttle;
  controller.right_x = turn;
  controller.buttons = buttons;
  controller.connected = true;
  controller.api_ok = true;
  return controller;
}

robot::RobotState stateAt(const robot::FrameHeader& header, double speed,
                          double heading = 1.0) {
  robot::RobotState state{};
  state.h = header;
  state.pose.theta_rad = heading;
  state.body_velocity.vx_mps = speed;
  state.translation_quality = robot::Quality::Good;
  state.heading_quality = robot::Quality::Good;
  return state;
}

robot::OwnerToken owner(std::uint32_t epoch = 1) {
  return {11, robot::Requirement::kDrivetrain, 4, epoch};
}

}  // namespace

ROBOT_TEST("remapped cubic input curve is odd monotonic and full scale") {
  const robot::AxisShapeConfig config{0.0, 0.1, 0.6};
  double previous = -1.0;
  for (int i = -100; i <= 100; ++i) {
    const double input = static_cast<double>(i) / 100.0;
    const double output = robot::shapeCenteredAxis(input, config);
    ROBOT_REQUIRE(output >= previous - 1e-12);
    ROBOT_REQUIRE_NEAR(output,
                       -robot::shapeCenteredAxis(-input, config), 1e-12);
    previous = output;
  }
  ROBOT_REQUIRE_NEAR(robot::shapeCenteredAxis(-1.0, config), -1.0, 1e-12);
  ROBOT_REQUIRE_NEAR(robot::shapeCenteredAxis(1.0, config), 1.0, 1e-12);
  ROBOT_REQUIRE_NEAR(robot::shapeCenteredAxis(0.1, config), 0.0, 1e-12);
}

ROBOT_TEST("axis slew crosses a reversal through zero and resets") {
  robot::AsymmetricAxisSlew slew(1.0, 2.0, 0.1);
  double output{};
  ROBOT_REQUIRE(slew.update(1.0, 0.1, output));
  ROBOT_REQUIRE_NEAR(output, 0.1, 1e-12);
  ROBOT_REQUIRE(slew.update(-1.0, 0.1, output));
  ROBOT_REQUIRE_NEAR(output, 0.0, 1e-12);
  slew.reset();
  ROBOT_REQUIRE(slew.update(-1.0, 0.1, output));
  ROBOT_REQUIRE_NEAR(output, -0.1, 1e-12);
}

ROBOT_TEST("manual drive publishes a fresh curvature request with lease") {
  robot::ManualDrive manual(manualConfig());
  const robot::FrameHeader header{10000, 1, 1};
  const auto result = manual.update(header, driverMode(),
                                    controllerAt(header, 0.6, 0.3),
                                    stateAt(header, 0.2), 0.01, owner());
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE(result.request.source == robot::RequestSource::Driver);
  ROBOT_REQUIRE(robot::sameLease(result.request.owner, owner()));
  ROBOT_REQUIRE(result.request.ttl_us == 30000);
  const auto* payload =
      std::get_if<robot::DriverCurvaturePayload>(&result.request.payload);
  ROBOT_REQUIRE(payload != nullptr);
  ROBOT_REQUIRE(payload->steering_mode ==
                robot::DriverSteeringMode::Curvature);
  ROBOT_REQUIRE(payload->allocation ==
                robot::AllocationPolicy::RatioPreserving);
}

ROBOT_TEST("quick turn is explicit and speed gated") {
  robot::ManualDrive manual(manualConfig());
  const robot::FrameHeader header{10000, 1, 1};
  auto result = manual.update(
      header, driverMode(),
      controllerAt(header, 0.1, 0.8, robot::kButtonR1),
      stateAt(header, 0.1), 0.01, owner());
  const auto* payload =
      std::get_if<robot::DriverCurvaturePayload>(&result.request.payload);
  ROBOT_REQUIRE(result.valid && payload != nullptr);
  ROBOT_REQUIRE(payload->steering_mode ==
                robot::DriverSteeringMode::QuickTurn);

  const robot::FrameHeader fast_header{20000, 2, 1};
  result = manual.update(
      fast_header, driverMode(),
      controllerAt(fast_header, 0.1, 0.8, robot::kButtonR1),
      stateAt(fast_header, 0.5), 0.01, owner());
  payload = std::get_if<robot::DriverCurvaturePayload>(&result.request.payload);
  ROBOT_REQUIRE(payload != nullptr);
  ROBOT_REQUIRE(payload->steering_mode ==
                robot::DriverSteeringMode::Curvature);
}

ROBOT_TEST("heading assist locks then releases on raw turn in one frame") {
  robot::ManualDrive manual(manualConfig());
  for (std::uint32_t sequence = 1; sequence <= 3; ++sequence) {
    const robot::FrameHeader header{sequence * 10000u, sequence, 1};
    const auto result = manual.update(
        header, driverMode(), controllerAt(header, 0.6, 0.0),
        stateAt(header, 0.6, 1.0), 0.01, owner());
    ROBOT_REQUIRE(result.valid);
  }

  const robot::FrameHeader correction_header{40000, 4, 1};
  auto result = manual.update(
      correction_header, driverMode(),
      controllerAt(correction_header, 0.6, 0.0),
      stateAt(correction_header, 0.6, 0.9), 0.01, owner());
  auto* payload =
      std::get_if<robot::DriverCurvaturePayload>(&result.request.payload);
  ROBOT_REQUIRE(payload != nullptr);
  ROBOT_REQUIRE(payload->steering_mode ==
                robot::DriverSteeringMode::HeadingAssist);
  ROBOT_REQUIRE(payload->allocation == robot::AllocationPolicy::PreserveTurn);
  ROBOT_REQUIRE_NEAR(payload->steering, 0.1, 1e-12);

  const robot::FrameHeader release_header{50000, 5, 1};
  result = manual.update(release_header, driverMode(),
                         controllerAt(release_header, 0.6, 0.05),
                         stateAt(release_header, 0.6, 0.9), 0.01,
                         owner());
  payload = std::get_if<robot::DriverCurvaturePayload>(&result.request.payload);
  ROBOT_REQUIRE(payload != nullptr);
  ROBOT_REQUIRE(payload->steering_mode ==
                robot::DriverSteeringMode::Curvature);
}

ROBOT_TEST("disconnect returns no request and clears remembered control state") {
  auto config = manualConfig();
  config.throttle_rise_per_s = 1.0;
  robot::ManualDrive manual(config);
  const robot::FrameHeader first_header{10000, 1, 1};
  auto result = manual.update(first_header, driverMode(),
                              controllerAt(first_header, 1.0, 0.0),
                              stateAt(first_header, 0.0), 0.01, owner());
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE_NEAR(result.diagnostics.slewed_throttle, 0.01, 1e-12);

  const robot::FrameHeader disconnected_header{20000, 2, 1};
  auto disconnected = controllerAt(disconnected_header, 1.0, 0.0);
  disconnected.connected = false;
  result = manual.update(disconnected_header, driverMode(), disconnected,
                         stateAt(disconnected_header, 0.0), 0.01, owner());
  ROBOT_REQUIRE(!result.valid);
  ROBOT_REQUIRE((result.diagnostics.reject_bits &
                 robot::kManualControllerDisconnected) != 0);

  const robot::FrameHeader reconnect_header{30000, 3, 1};
  result = manual.update(reconnect_header, driverMode(),
                         controllerAt(reconnect_header, 1.0, 0.0),
                         stateAt(reconnect_header, 0.0), 0.01, owner());
  ROBOT_REQUIRE(result.valid);
  ROBOT_REQUIRE_NEAR(result.diagnostics.slewed_throttle, 0.01, 1e-12);
}

ROBOT_TEST("wrong epoch or lease cannot produce a manual request") {
  robot::ManualDrive manual(manualConfig());
  const robot::FrameHeader header{10000, 1, 2};
  const auto result = manual.update(
      header, driverMode(2), controllerAt(header, 0.5, 0.0),
      stateAt(header, 0.0), 0.01, owner(1));
  ROBOT_REQUIRE(!result.valid);
  ROBOT_REQUIRE((result.diagnostics.reject_bits & robot::kManualOwnerInvalid) !=
                0);
}
