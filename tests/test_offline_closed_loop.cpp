#include "robot/autonomy/safety_supervisor.hpp"
#include "robot/autonomy/trajectory_tracker.hpp"
#include "robot/commands/drive_request_arbiter.hpp"
#include "robot/control/chassis_velocity_controller.hpp"
#include "robot/drive/output_service.hpp"
#include "robot/drive/safety_gate.hpp"
#include "robot/platform/fake_io.hpp"
#include "test_framework.hpp"

#include <array>
#include <cmath>
#include <variant>

namespace {

robot::HardwareConfig fakeHardware() {
  robot::HardwareConfig hardware{};
  hardware.left = {{{1, false, 200}, {2, false, 200}, {3, false, 200}}};
  hardware.right = {{{4, true, 200}, {5, true, 200}, {6, true, 200}}};
  hardware.imu = {true, 7};
  return hardware;
}

robot::PidConfig wheelPid() {
  return {2.0, 0.0, 0.0, -12.0, 12.0, -1.0, 1.0, 10.0,
          0.0, 0.0, 0.001, 0.05};
}

class ExactAuthority final : public robot::LeaseAuthority {
 public:
  explicit ExactAuthority(robot::OwnerToken owner) : owner_(owner) {}
  bool owns(const robot::OwnerToken& token) const noexcept override {
    return robot::sameLease(owner_, token);
  }

 private:
  robot::OwnerToken owner_{};
};

double averagePosition(const robot::MotorSideRaw<robot::kMotorsPerSide>& side) {
  double sum{};
  for (const auto& motor : side.motor) sum += motor.position_rad.value;
  return sum / static_cast<double>(robot::kMotorsPerSide);
}

double averageVelocity(const robot::MotorSideRaw<robot::kMotorsPerSide>& side) {
  double sum{};
  for (const auto& motor : side.motor) sum += motor.velocity_radps.value;
  return sum / static_cast<double>(robot::kMotorsPerSide);
}

}  // namespace

ROBOT_TEST("offline full autonomy chain closes through FakeDriveIO") {
  constexpr double kTrackWidthM = 0.4;
  constexpr double kMetresPerMotorRad = 0.05;
  constexpr double kDtS = 0.01;
  constexpr robot::TimeUs kDtUs = 10000;
  const robot::ModeSnapshot mode{
      robot::CompetitionMode::AutonomousInterface, true, false, 7, 0, 0};
  const robot::OwnerToken owner{40, robot::Requirement::kDrivetrain, 7, 7};
  robot::DriveCapabilities capabilities{};
  capabilities.autonomous_chassis_velocity = true;

  std::array<robot::PathWaypoint, 2> waypoints{{
      {{0.0, 0.0, 0.0}, 0.0, robot::TravelDirection::Forward},
      {{1.0, 0.0, 0.0}, 0.0, robot::TravelDirection::Forward},
  }};
  const robot::DifferentialFeedforwardConfig feedforward{
      {0.0, 0.0, 10.0, 0.0}, {0.0, 0.0, 10.0, 0.0}};
  const robot::TrajectoryConstraints trajectory_constraints{
      0.5, 1.0, 1.0, 1.5, 2.0, 2.5, 12.0, kTrackWidthM, 0.025,
      0.0, 0.0, feedforward};
  robot::FixedTrajectoryGenerator<2, 128> generator;
  robot::FixedTrajectory<128> trajectory;
  ROBOT_REQUIRE(generator.generate(waypoints, 2, trajectory_constraints,
                                   trajectory) ==
                robot::TrajectoryGenerationStatus::Success);

  const robot::TrajectoryTrackerConfig tracker_config{
      1.5,
      3.0,
      2.5,
      0.2,
      0.7,
      1.5,
      2.0,
      0.5,
      1.0,
      0.6,
      0.5,
      30000,
      30000,
      200000,
      true,
      {0.03, 0.04, 150000, 6000000, 2.0, 0.01, 500000}};
  robot::TrajectoryTracker<128> tracker(tracker_config);

  const std::array<robot::FailurePolicyEntry, 7> policy_entries{{
      {robot::AutonomousOutcome::Timeout, robot::FailureAction::EndRoute, 0,
       robot::FailureAction::EndRoute},
      {robot::AutonomousOutcome::Stalled, robot::FailureAction::EndRoute, 0,
       robot::FailureAction::EndRoute},
      {robot::AutonomousOutcome::StateInvalid,
       robot::FailureAction::EndRoute, 0, robot::FailureAction::EndRoute},
      {robot::AutonomousOutcome::Interrupted,
       robot::FailureAction::EndRoute, 0, robot::FailureAction::EndRoute},
      {robot::AutonomousOutcome::Collision,
       robot::FailureAction::EmergencyStop, 0,
       robot::FailureAction::EmergencyStop},
      {robot::AutonomousOutcome::Deviation,
       robot::FailureAction::EndRoute, 0, robot::FailureAction::EndRoute},
      {robot::AutonomousOutcome::MechanismConflict,
       robot::FailureAction::EndRoute, 0, robot::FailureAction::EndRoute},
  }};
  const robot::FixedFailurePolicy<7> policy(policy_entries);
  robot::AutonomySafetySupervisor<7> supervisor(
      {0.5, 0.5, 200000, 30000, 30000,
       robot::DegradedAutonomyPolicy::ContinueScaled},
      policy);

  robot::ChassisVelocityController velocity_controller(
      {kTrackWidthM, 1.0, 5.0, 12.0, 0.5, 30000, 30000, true,
       wheelPid(), wheelPid(), feedforward});
  robot::DriveRequestArbiter arbiter({40000});
  const ExactAuthority authority(owner);
  robot::SafetyGate safety_gate(
      {12.0, 40000, {1000.0, 1000.0, 0.05}, robot::StopMode::Brake,
       robot::StopMode::Brake, robot::StopMode::Brake});
  robot::FakeDriveIO drive(fakeHardware(), {2.0, 8.0});
  ROBOT_REQUIRE(drive.initialize());
  robot::OutputService output(
      drive, {30000, 12.0, 1e-9, robot::StopMode::Brake});

  robot::RobotState state{};
  state.h = {0, 1, 7};
  state.competition = mode;
  state.translation_quality = robot::Quality::Good;
  state.heading_quality = robot::Quality::Good;
  state.output_derate = 1.0;
  robot::TrajectoryTrackerInput tracker_start{{0, 1, 7}, mode, &state, owner,
                                                capabilities,
                                                robot::Quality::Good, 0};
  ROBOT_REQUIRE(tracker.start(trajectory, tracker_start));
  ROBOT_REQUIRE(supervisor.startRoute(0x101, mode, 0));
  ROBOT_REQUIRE(supervisor.beginCommand(owner, mode));

  double last_left_distance{};
  double last_right_distance{};
  double max_command_voltage{};
  bool succeeded = false;
  for (std::uint32_t step = 0; step < 600; ++step) {
    const robot::TimeUs now_us = static_cast<robot::TimeUs>(step) * kDtUs;
    const robot::FrameHeader header{now_us, step + 1, 7};
    const auto raw = drive.readAll(header);
    const double left_distance =
        averagePosition(raw.left) * kMetresPerMotorRad;
    const double right_distance =
        averagePosition(raw.right) * kMetresPerMotorRad;
    if (step > 0) {
      const double dl = left_distance - last_left_distance;
      const double dr = right_distance - last_right_distance;
      state.pose = robot::integrateBodyTwist(
          state.pose, {0.5 * (dl + dr), 0.0, (dr - dl) / kTrackWidthM});
    }
    last_left_distance = left_distance;
    last_right_distance = right_distance;
    state.h = header;
    state.left_distance_m = left_distance;
    state.right_distance_m = right_distance;
    state.left_velocity_mps =
        averageVelocity(raw.left) * kMetresPerMotorRad;
    state.right_velocity_mps =
        averageVelocity(raw.right) * kMetresPerMotorRad;
    state.body_velocity.vx_mps =
        0.5 * (state.left_velocity_mps + state.right_velocity_mps);
    state.body_velocity.omega_radps =
        (state.right_velocity_mps - state.left_velocity_mps) / kTrackWidthM;

    const robot::TrajectoryTrackerInput tracker_input{
        header, mode, &state, owner, capabilities, robot::Quality::Good,
        now_us};
    const auto tracked = tracker.update(tracker_input);
    robot::AutonomySafetyInput safety_input{};
    safety_input.h = header;
    safety_input.mode = mode;
    safety_input.state = &state;
    safety_input.owner = owner;
    safety_input.slip_quality = robot::Quality::Good;
    safety_input.deviation_m = std::hypot(
        tracked.body_position_error.x_m, tracked.body_position_error.y_m);
    if (tracked.state == robot::TrajectoryTrackerState::Success) {
      safety_input.command_terminal = true;
      safety_input.command_outcome = robot::AutonomousOutcome::Success;
      succeeded = true;
    } else if (tracked.state == robot::TrajectoryTrackerState::Timeout) {
      safety_input.command_terminal = true;
      safety_input.command_outcome = robot::AutonomousOutcome::Timeout;
    } else if (tracked.state == robot::TrajectoryTrackerState::Stalled) {
      safety_input.command_terminal = true;
      safety_input.command_outcome = robot::AutonomousOutcome::Stalled;
    } else if (tracked.state == robot::TrajectoryTrackerState::StateInvalid) {
      safety_input.command_terminal = true;
      safety_input.command_outcome = robot::AutonomousOutcome::StateInvalid;
    } else if (tracked.state == robot::TrajectoryTrackerState::DeviationAbort) {
      safety_input.command_terminal = true;
      safety_input.command_outcome = robot::AutonomousOutcome::Deviation;
    }
    const auto safety_decision = supervisor.evaluate(safety_input);

    robot::DriveRequest candidate{};
    if (safety_decision.has_safety_request) {
      candidate = safety_decision.safety_request;
    } else if (std::holds_alternative<robot::ChassisVelocityPayload>(
                   tracked.request.payload)) {
      ROBOT_REQUIRE(supervisor.filterChassisRequest(
          safety_decision, tracked.request, candidate));
    } else {
      candidate = tracked.request;
    }
    std::array<robot::DriveRequestCandidate, 1> candidates{{
        {candidate, tracked.has_request || safety_decision.has_safety_request},
    }};
    const auto selected =
        arbiter.select(candidates, mode, now_us, capabilities, authority);
    ROBOT_REQUIRE(selected.has_selection);

    robot::DriveRequest voltage_request = selected.selected;
    if (std::holds_alternative<robot::ChassisVelocityPayload>(
            selected.selected.payload)) {
      const robot::ChassisVelocityControlInput controller_input{
          header, mode, &state, now_us, kDtS, capabilities};
      const auto controlled =
          velocity_controller.update(selected.selected, controller_input);
      ROBOT_REQUIRE(controlled.has_request);
      voltage_request = controlled.request;
    }
    const robot::SafetyGateInput gate_input{
        header, mode, now_us, kDtS, 1.0, capabilities};
    const auto frame = safety_gate.apply(&voltage_request, gate_input);
    max_command_voltage =
        std::max(max_command_voltage,
                 std::max(std::abs(frame.left_V), std::abs(frame.right_V)));
    const auto output_result = output.tick(mode, &frame, now_us);
    ROBOT_REQUIRE(output_result.io_ok);
    drive.advance(kDtS);
    if (succeeded) break;
  }

  ROBOT_REQUIRE(succeeded);
  ROBOT_REQUIRE(!supervisor.latched());
  ROBOT_REQUIRE_NEAR(state.pose.x_m, 1.0, 0.03);
  ROBOT_REQUIRE_NEAR(state.pose.y_m, 0.0, 0.01);
  ROBOT_REQUIRE_NEAR(state.pose.theta_rad, 0.0, 0.01);
  ROBOT_REQUIRE(max_command_voltage > 1.0);
  ROBOT_REQUIRE(max_command_voltage <= 12.0);
  ROBOT_REQUIRE(drive.writeCount() > 10);
}
