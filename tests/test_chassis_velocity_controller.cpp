#include "robot/commands/drive_request_arbiter.hpp"
#include "robot/control/chassis_velocity_controller.hpp"
#include "robot/drive/output_service.hpp"
#include "robot/drive/safety_gate.hpp"
#include "robot/platform/fake_io.hpp"
#include "test_framework.hpp"

#include <array>
#include <limits>

namespace {

robot::PidConfig wheelPid(double kp = 1.0) {
  return {kp, 0.0, 0.0, -6.0, 6.0, -1.0, 1.0, 10.0,
          0.0, 0.0, 0.001, 0.2};
}

robot::ChassisVelocityControllerConfig controllerConfig() {
  return {0.4,
          3.0,
          2.0,
          6.0,
          0.5,
          30000,
          20000,
          true,
          wheelPid(),
          wheelPid(),
          {{0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 1.0, 0.0}}};
}

robot::ModeSnapshot autonomousMode(std::uint32_t epoch = 7) {
  return {robot::CompetitionMode::AutonomousInterface, true, false, epoch, 0,
          0};
}

robot::RobotState goodState(robot::TimeUs time_us = 10000,
                            std::uint32_t epoch = 7) {
  robot::RobotState state{};
  state.h = {time_us, 1, epoch};
  state.competition = autonomousMode(epoch);
  state.translation_quality = robot::Quality::Good;
  state.heading_quality = robot::Quality::Good;
  state.output_derate = 1.0;
  return state;
}

robot::DriveRequest chassisRequest(double vx, double omega,
                                   robot::TimeUs time_us = 10000,
                                   std::uint32_t epoch = 7) {
  robot::DriveRequest request{};
  request.h = {time_us, 1, epoch};
  request.source = robot::RequestSource::FutureAutonomy;
  request.owner = {4, robot::Requirement::kDrivetrain, 9, epoch};
  request.ttl_us = 30000;
  request.payload = robot::ChassisVelocityPayload{vx, omega};
  return request;
}

robot::ChassisVelocityControlInput controlInput(const robot::RobotState& state,
                                                robot::TimeUs now_us = 10000) {
  robot::DriveCapabilities capabilities{};
  capabilities.autonomous_chassis_velocity = true;
  return {{now_us, 2, state.h.mode_epoch}, autonomousMode(state.h.mode_epoch),
          &state, now_us, 0.01, capabilities};
}

robot::HardwareConfig fakeHardware() {
  robot::HardwareConfig hardware{};
  hardware.left = {{{1, false, 200}, {2, false, 200}, {3, false, 200}}};
  hardware.right = {{{4, true, 200}, {5, true, 200}, {6, true, 200}}};
  hardware.imu = {true, 7};
  return hardware;
}

class ExactLease final : public robot::LeaseAuthority {
 public:
  explicit ExactLease(robot::OwnerToken owner) : owner_(owner) {}
  bool owns(const robot::OwnerToken& token) const noexcept override {
    return robot::sameLease(owner_, token);
  }

 private:
  robot::OwnerToken owner_{};
};

}  // namespace

ROBOT_TEST("chassis velocity controller performs inverse kinematics FF and PID") {
  robot::ChassisVelocityController controller(controllerConfig());
  ROBOT_REQUIRE(controller.valid());
  const auto state = goodState();
  const auto result = controller.update(chassisRequest(1.0, 1.0),
                                        controlInput(state));
  ROBOT_REQUIRE(result.has_request);
  ROBOT_REQUIRE(result.reject_bits == robot::kChassisVelocityAccepted);
  ROBOT_REQUIRE_NEAR(result.target.left_mps, 0.8, 1e-12);
  ROBOT_REQUIRE_NEAR(result.target.right_mps, 1.2, 1e-12);
  const auto* voltage =
      std::get_if<robot::WheelVoltagePayload>(&result.request.payload);
  ROBOT_REQUIRE(voltage != nullptr);
  ROBOT_REQUIRE_NEAR(voltage->left_V, 1.6, 1e-12);
  ROBOT_REQUIRE_NEAR(voltage->right_V, 2.4, 1e-12);
  ROBOT_REQUIRE(result.request.source == robot::RequestSource::FutureAutonomy);
}

ROBOT_TEST("autonomous wheel voltage remains source scoped through safety gate") {
  const auto state = goodState();
  auto chassis = chassisRequest(0.5, 0.0);
  ExactLease authority(chassis.owner);
  robot::DriveCapabilities capabilities{};
  capabilities.autonomous_chassis_velocity = true;
  std::array<robot::DriveRequestCandidate, 1> candidates{{{chassis, true}}};
  robot::DriveRequestArbiter arbiter({40000});
  const auto selected = arbiter.select(candidates, autonomousMode(), 10000,
                                       capabilities, authority);
  ROBOT_REQUIRE(selected.has_selection);
  robot::ChassisVelocityController controller(controllerConfig());
  const auto controlled =
      controller.update(selected.selected, controlInput(state));
  ROBOT_REQUIRE(controlled.has_request);

  robot::SafetyGate gate({12.0, 40000, {1000.0, 1000.0, 0.05},
                          robot::StopMode::Brake, robot::StopMode::Brake,
                          robot::StopMode::Brake});
  const robot::SafetyGateInput gate_input{{10000, 3, 7}, autonomousMode(),
                                           10000, 0.01, 1.0, capabilities};
  const auto frame = gate.apply(&controlled.request, gate_input);
  ROBOT_REQUIRE(frame.owner == robot::RequestSource::FutureAutonomy);
  ROBOT_REQUIRE(frame.left_V > 0.0 && frame.right_V > 0.0);

  robot::FakeDriveIO io(fakeHardware(), {5.0, 8.0});
  ROBOT_REQUIRE(io.initialize());
  robot::OutputService output(
      io, {20000, 12.0, 1e-9, robot::StopMode::Brake});
  ROBOT_REQUIRE(output.tick(autonomousMode(), &frame, 10000).action ==
                robot::OutputAction::WroteVoltage);

  auto wrong_source = controlled.request;
  wrong_source.source = robot::RequestSource::Driver;
  ROBOT_REQUIRE(!robot::supportedPayload(wrong_source.payload, capabilities,
                                         wrong_source.source));
  const auto rejected = gate.apply(&wrong_source, gate_input);
  ROBOT_REQUIRE((rejected.applied_limits & robot::kAppliedInvalidRequestStop) !=
                0);
  ROBOT_REQUIRE_NEAR(rejected.left_V, 0.0, 0.0);
}

ROBOT_TEST("chassis velocity controller rejects locked stale and invalid state") {
  robot::ChassisVelocityController controller(controllerConfig());
  auto state = goodState();
  auto input = controlInput(state);
  input.capabilities.autonomous_chassis_velocity = false;
  auto result = controller.update(chassisRequest(1.0, 0.0), input);
  ROBOT_REQUIRE(!result.has_request);
  ROBOT_REQUIRE((result.reject_bits &
                 robot::kChassisVelocityCapabilityLocked) != 0);

  input = controlInput(state, 40001);
  result = controller.update(chassisRequest(1.0, 0.0, 40000), input);
  ROBOT_REQUIRE(!result.has_request);
  ROBOT_REQUIRE((result.reject_bits & robot::kChassisVelocityStateStale) != 0);

  state = goodState();
  state.translation_quality = robot::Quality::Invalid;
  input = controlInput(state);
  result = controller.update(chassisRequest(1.0, 0.0), input);
  ROBOT_REQUIRE(!result.has_request);
  ROBOT_REQUIRE((result.reject_bits & robot::kChassisVelocityStateInvalid) !=
                0);
}

ROBOT_TEST("degraded velocity state scales both chassis targets") {
  robot::ChassisVelocityController controller(controllerConfig());
  auto state = goodState();
  state.translation_quality = robot::Quality::Degraded;
  const auto result = controller.update(chassisRequest(1.0, 1.0),
                                        controlInput(state));
  ROBOT_REQUIRE(result.has_request);
  ROBOT_REQUIRE((result.applied_limits &
                 robot::kChassisVelocityDegradedScale) != 0);
  ROBOT_REQUIRE_NEAR(result.target.left_mps, 0.4, 1e-12);
  ROBOT_REQUIRE_NEAR(result.target.right_mps, 0.6, 1e-12);
}

ROBOT_TEST("velocity target acceleration is bounded for feedforward") {
  auto config = controllerConfig();
  config.feedforward.left.kA_Vs2_per_unit = 1.0;
  config.feedforward.right.kA_Vs2_per_unit = 1.0;
  robot::ChassisVelocityController controller(config);
  auto state = goodState();
  auto input = controlInput(state);
  ROBOT_REQUIRE(controller.update(chassisRequest(0.0, 0.0), input).has_request);
  state.h = {20000, 2, 7};
  input = controlInput(state, 20000);
  input.dt_s = 0.01;
  const auto ramp =
      controller.update(chassisRequest(1.0, 0.0, 20000), input);
  ROBOT_REQUIRE(ramp.has_request);
  ROBOT_REQUIRE((ramp.applied_limits &
                 robot::kChassisVelocityLeftAccelerationLimit) != 0);
  ROBOT_REQUIRE((ramp.applied_limits &
                 robot::kChassisVelocityRightAccelerationLimit) != 0);
  ROBOT_REQUIRE_NEAR(ramp.feedforward.left_V, 3.0, 1e-12);
  ROBOT_REQUIRE_NEAR(ramp.feedforward.right_V, 3.0, 1e-12);
}

ROBOT_TEST("wheel PID saturation does not wind up autonomous controller") {
  auto config = controllerConfig();
  config.left_pid = wheelPid(100.0);
  config.right_pid = wheelPid(100.0);
  robot::ChassisVelocityController controller(config);
  const auto state = goodState();
  const auto result = controller.update(chassisRequest(2.0, 0.0),
                                        controlInput(state));
  ROBOT_REQUIRE(result.has_request);
  ROBOT_REQUIRE_NEAR(result.left_pid.output, 6.0, 0.0);
  ROBOT_REQUIRE_NEAR(result.right_pid.output, 6.0, 0.0);
  ROBOT_REQUIRE_NEAR(result.left_pid.integral, 0.0, 0.0);
  ROBOT_REQUIRE((result.left_pid.status &
                 robot::kPidIntegralHeldForSaturation) != 0);
}

ROBOT_TEST("chassis velocity controller rejects target and time boundaries") {
  robot::ChassisVelocityController controller(controllerConfig());
  const auto state = goodState();
  auto result = controller.update(chassisRequest(4.0, 0.0),
                                  controlInput(state));
  ROBOT_REQUIRE(!result.has_request);
  ROBOT_REQUIRE((result.reject_bits &
                 robot::kChassisVelocityTargetOutOfRange) != 0);
  auto request = chassisRequest(1.0, 0.0);
  std::get<robot::ChassisVelocityPayload>(request.payload).omega_radps =
      std::numeric_limits<double>::quiet_NaN();
  result = controller.update(request, controlInput(state));
  ROBOT_REQUIRE(!result.has_request);
  ROBOT_REQUIRE((result.reject_bits & robot::kChassisVelocityBadRequest) != 0);
}
