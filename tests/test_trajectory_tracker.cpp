#include "robot/autonomy/trajectory_tracker.hpp"
#include "test_framework.hpp"

#include <array>
#include <cmath>
#include <variant>

namespace {

robot::TrajectoryConstraints generationConstraints() {
  return {1.0,
          2.0,
          2.0,
          2.0,
          2.0,
          3.0,
          12.0,
          0.4,
          0.05,
          0.0,
          0.0,
          {{0.1, 0.1, 0.5, 0.05}, {0.1, 0.1, 0.5, 0.05}}};
}

robot::TrajectoryTrackerConfig trackerConfig() {
  return {2.0,
          3.0,
          3.0,
          0.2,
          1.2,
          2.0,
          2.0,
          0.75,
          1.0,
          0.5,
          0.4,
          50000,
          30000,
          200000,
          true,
          {0.03, 0.05, 100000, 10000000, 2.0, 0.01, 500000}};
}

template <robot::TravelDirection Direction>
robot::FixedTrajectory<128> lineTrajectory() {
  const double end_x = Direction == robot::TravelDirection::Forward ? 2.0 : -2.0;
  std::array<robot::PathWaypoint, 2> waypoints{{
      {{0.0, 0.0, 0.0}, 0.0, Direction},
      {{end_x, 0.0, 0.0}, 0.0, Direction},
  }};
  robot::FixedTrajectoryGenerator<2, 128> generator;
  robot::FixedTrajectory<128> trajectory;
  const auto status = generator.generate(waypoints, 2, generationConstraints(),
                                         trajectory);
  ROBOT_REQUIRE(status == robot::TrajectoryGenerationStatus::Success);
  return trajectory;
}

robot::RobotState trackerState(const robot::TrajectorySample& target,
                               robot::TimeUs time_us,
                               std::uint32_t sequence = 1) {
  robot::RobotState state{};
  state.h = {time_us, sequence, 7};
  state.competition = {robot::CompetitionMode::AutonomousInterface, true,
                       false, 7, 0, 0};
  state.pose = target.pose;
  state.body_velocity.vx_mps = target.linear_velocity_mps;
  state.body_velocity.omega_radps = target.angular_velocity_radps;
  state.translation_quality = robot::Quality::Good;
  state.heading_quality = robot::Quality::Good;
  return state;
}

robot::TrajectoryTrackerInput trackerInput(robot::RobotState& state,
                                           robot::TimeUs now_us,
                                           robot::Quality slip =
                                               robot::Quality::Good,
                                           std::uint32_t sequence = 1) {
  state.h.time_us = now_us;
  state.h.sequence = sequence;
  robot::DriveCapabilities capabilities{};
  capabilities.autonomous_chassis_velocity = true;
  return {{now_us, sequence, 7},
          {robot::CompetitionMode::AutonomousInterface, true, false, 7, 0, 0},
          &state,
          {21, robot::Requirement::kDrivetrain, 5, 7},
          capabilities,
          slip,
          now_us};
}

const robot::ChassisVelocityPayload* velocity(
    const robot::TrajectoryTrackerResult& result) {
  return std::get_if<robot::ChassisVelocityPayload>(&result.request.payload);
}

robot::TimeUs toUs(double time_s) {
  return static_cast<robot::TimeUs>(std::ceil(time_s * 1e6));
}

}  // namespace

ROBOT_TEST("trajectory tracker reproduces exact straight reference in body frame") {
  const auto trajectory = lineTrajectory<robot::TravelDirection::Forward>();
  robot::TrajectoryTracker<128> tracker(trackerConfig());
  auto state = trackerState(trajectory[0], 0);
  ROBOT_REQUIRE(tracker.start(trajectory, trackerInput(state, 0)));
  const double sample_time = 0.5 * trajectory.totalTime();
  const auto target = trajectory.sampleAt(sample_time);
  const auto now = toUs(sample_time);
  state = trackerState(target, now, 2);
  const auto result = tracker.update(trackerInput(state, now, robot::Quality::Good,
                                                  2));
  ROBOT_REQUIRE(result.has_request);
  ROBOT_REQUIRE(result.state == robot::TrajectoryTrackerState::Tracking);
  ROBOT_REQUIRE(velocity(result) != nullptr);
  ROBOT_REQUIRE_NEAR(velocity(result)->vx_mps, target.linear_velocity_mps,
                     1e-5);
  ROBOT_REQUIRE_NEAR(velocity(result)->omega_radps, 0.0, 1e-9);
  ROBOT_REQUIRE(result.progress > 0.0 && result.progress < 1.0);
}

ROBOT_TEST("trajectory tracker preserves reverse chassis velocity") {
  const auto trajectory = lineTrajectory<robot::TravelDirection::Reverse>();
  robot::TrajectoryTracker<128> tracker(trackerConfig());
  auto state = trackerState(trajectory[0], 0);
  ROBOT_REQUIRE(tracker.start(trajectory, trackerInput(state, 0)));
  const double sample_time = 0.5 * trajectory.totalTime();
  const auto target = trajectory.sampleAt(sample_time);
  const auto now = toUs(sample_time);
  state = trackerState(target, now, 2);
  const auto result = tracker.update(trackerInput(state, now));
  ROBOT_REQUIRE(velocity(result) != nullptr);
  ROBOT_REQUIRE(velocity(result)->vx_mps < 0.0);
}

ROBOT_TEST("world error becomes signed body correction with curvature limits") {
  const auto trajectory = lineTrajectory<robot::TravelDirection::Forward>();
  auto config = trackerConfig();
  config.lateral_error_gain_per_m = 20.0;
  config.max_curvature_per_m = 0.5;
  robot::TrajectoryTracker<128> tracker(config);
  auto state = trackerState(trajectory[0], 0);
  ROBOT_REQUIRE(tracker.start(trajectory, trackerInput(state, 0)));
  const double sample_time = 0.5 * trajectory.totalTime();
  const auto target = trajectory.sampleAt(sample_time);
  const auto now = toUs(sample_time);
  state = trackerState(target, now, 2);
  state.pose.y_m -= 0.4;
  const auto result = tracker.update(trackerInput(state, now));
  ROBOT_REQUIRE(velocity(result) != nullptr);
  ROBOT_REQUIRE(result.body_position_error.y_m > 0.0);
  ROBOT_REQUIRE(velocity(result)->omega_radps > 0.0);
  ROBOT_REQUIRE((result.applied_limit_bits & robot::kTrackerCurvatureClamp) !=
                0);
  ROBOT_REQUIRE(std::abs(velocity(result)->omega_radps) <=
                0.5 * std::abs(velocity(result)->vx_mps) + 1e-12);
}

ROBOT_TEST("degraded state and slip scale trajectory speed then abort") {
  const auto trajectory = lineTrajectory<robot::TravelDirection::Forward>();
  const double sample_time = 0.5 * trajectory.totalTime();
  const auto target = trajectory.sampleAt(sample_time);
  const auto now = toUs(sample_time);

  robot::TrajectoryTracker<128> baseline(trackerConfig());
  auto baseline_state = trackerState(trajectory[0], 0);
  ROBOT_REQUIRE(baseline.start(trajectory, trackerInput(baseline_state, 0)));
  baseline_state = trackerState(target, now, 2);
  const auto normal = baseline.update(trackerInput(baseline_state, now));

  robot::TrajectoryTracker<128> degraded(trackerConfig());
  auto degraded_state = trackerState(trajectory[0], 0);
  ROBOT_REQUIRE(degraded.start(trajectory, trackerInput(degraded_state, 0)));
  degraded_state = trackerState(target, now, 2);
  degraded_state.translation_quality = robot::Quality::Degraded;
  auto limited = degraded.update(
      trackerInput(degraded_state, now, robot::Quality::Degraded, 2));
  ROBOT_REQUIRE(limited.state == robot::TrajectoryTrackerState::Degraded);
  ROBOT_REQUIRE((limited.applied_limit_bits &
                 robot::kTrackerStateDegradedScale) != 0);
  ROBOT_REQUIRE((limited.applied_limit_bits & robot::kTrackerSlipScale) != 0);
  ROBOT_REQUIRE_NEAR(velocity(limited)->vx_mps,
                     velocity(normal)->vx_mps * 0.5 * 0.4, 1e-5);

  const auto abort_time = now + trackerConfig().slip_abort_time_us;
  degraded_state = trackerState(
      trajectory.sampleAt(sample_time + 0.2), abort_time, 3);
  limited = degraded.update(trackerInput(degraded_state, abort_time,
                                         robot::Quality::Degraded, 3));
  ROBOT_REQUIRE(limited.state == robot::TrajectoryTrackerState::SlipAbort);
  ROBOT_REQUIRE(std::holds_alternative<robot::BrakePayload>(
      limited.request.payload));
}

ROBOT_TEST("tracker invalid quality stale state and deviation all brake") {
  const auto trajectory = lineTrajectory<robot::TravelDirection::Forward>();
  robot::TrajectoryTracker<128> invalid_tracker(trackerConfig());
  auto state = trackerState(trajectory[0], 0);
  ROBOT_REQUIRE(invalid_tracker.start(trajectory, trackerInput(state, 0)));
  state.translation_quality = robot::Quality::Invalid;
  auto result = invalid_tracker.update(trackerInput(state, 1000));
  ROBOT_REQUIRE(result.state == robot::TrajectoryTrackerState::StateInvalid);
  ROBOT_REQUIRE(std::holds_alternative<robot::BrakePayload>(
      result.request.payload));

  robot::TrajectoryTracker<128> stale_tracker(trackerConfig());
  state = trackerState(trajectory[0], 0);
  ROBOT_REQUIRE(stale_tracker.start(trajectory, trackerInput(state, 0)));
  auto stale_input = trackerInput(state, 60000);
  state.h.time_us = 0;
  result = stale_tracker.update(stale_input);
  ROBOT_REQUIRE(result.state == robot::TrajectoryTrackerState::StateInvalid);

  robot::TrajectoryTracker<128> deviation_tracker(trackerConfig());
  state = trackerState(trajectory[0], 0);
  ROBOT_REQUIRE(deviation_tracker.start(trajectory, trackerInput(state, 0)));
  state.pose.y_m = 1.0;
  result = deviation_tracker.update(trackerInput(state, 10000));
  ROBOT_REQUIRE(result.state == robot::TrajectoryTrackerState::DeviationAbort);
  ROBOT_REQUIRE(std::holds_alternative<robot::BrakePayload>(
      result.request.payload));
}

ROBOT_TEST("trajectory endpoint requires settle and timeout is deterministic") {
  const auto trajectory = lineTrajectory<robot::TravelDirection::Forward>();
  robot::TrajectoryTracker<128> tracker(trackerConfig());
  auto state = trackerState(trajectory[0], 0);
  ROBOT_REQUIRE(tracker.start(trajectory, trackerInput(state, 0)));
  const auto end_time = toUs(trajectory.totalTime());
  state = trackerState(trajectory[trajectory.size() - 1], end_time, 2);
  state.body_velocity = {};
  auto result = tracker.update(trackerInput(state, end_time,
                                            robot::Quality::Good, 2));
  ROBOT_REQUIRE(result.state == robot::TrajectoryTrackerState::Settling);
  result = tracker.update(trackerInput(state, end_time + 100000,
                                       robot::Quality::Good, 3));
  ROBOT_REQUIRE(result.state == robot::TrajectoryTrackerState::Success);
  ROBOT_REQUIRE(std::holds_alternative<robot::BrakePayload>(
      result.request.payload));

  robot::TrajectoryTracker<128> timeout(trackerConfig());
  state = trackerState(trajectory[0], 0);
  ROBOT_REQUIRE(timeout.start(trajectory, trackerInput(state, 0)));
  state = trackerState(trajectory[trajectory.size() - 1], 10000000, 4);
  state.body_velocity = {};
  result = timeout.update(trackerInput(state, 10000000,
                                       robot::Quality::Good, 4));
  ROBOT_REQUIRE(result.state == robot::TrajectoryTrackerState::Timeout);
}

ROBOT_TEST("trajectory replay yields deterministic velocity requests") {
  const auto trajectory = lineTrajectory<robot::TravelDirection::Forward>();
  robot::TrajectoryTracker<128> first(trackerConfig());
  robot::TrajectoryTracker<128> second(trackerConfig());
  auto state_a = trackerState(trajectory[0], 0);
  auto state_b = state_a;
  ROBOT_REQUIRE(first.start(trajectory, trackerInput(state_a, 0)));
  ROBOT_REQUIRE(second.start(trajectory, trackerInput(state_b, 0)));
  for (int i = 1; i <= 10; ++i) {
    const auto now = static_cast<robot::TimeUs>(i) * 100000;
    const auto target = trajectory.sampleAt(static_cast<double>(now) * 1e-6);
    state_a = trackerState(target, now, static_cast<std::uint32_t>(i + 1));
    state_b = state_a;
    const auto a = first.update(trackerInput(
        state_a, now, robot::Quality::Good,
        static_cast<std::uint32_t>(i + 1)));
    const auto b = second.update(trackerInput(
        state_b, now, robot::Quality::Good,
        static_cast<std::uint32_t>(i + 1)));
    ROBOT_REQUIRE(a.state == b.state);
    ROBOT_REQUIRE_NEAR(velocity(a)->vx_mps, velocity(b)->vx_mps, 0.0);
    ROBOT_REQUIRE_NEAR(velocity(a)->omega_radps,
                       velocity(b)->omega_radps, 0.0);
  }
}
