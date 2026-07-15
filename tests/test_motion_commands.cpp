#include "robot/autonomy/motion_commands.hpp"
#include "test_framework.hpp"

#include <variant>

namespace {

class RecordingPublisher final : public robot::DriveRequestPublisher {
 public:
  bool publish(const robot::DriveRequest& request) noexcept override {
    ++count;
    last = request;
    return !reject;
  }
  robot::DriveRequest last{};
  int count{};
  bool reject{};
};

robot::MotionPrimitiveConfig commandConfig() {
  return {{1.0, 2.0},
          {2.0, 4.0},
          1.0,
          2.0,
          2.0,
          3.0,
          1.0,
          0.2,
          0.5,
          50000,
          30000,
          true,
          {0.02, 0.05, 100000, 3000000, 0.5, 0.02, 200000}};
}

robot::RobotState stateAt(robot::TimeUs time_us, std::uint32_t sequence = 1,
                          std::uint32_t epoch = 7) {
  robot::RobotState state{};
  state.h = {time_us, sequence, epoch};
  state.competition = {robot::CompetitionMode::AutonomousInterface, true,
                       false, epoch, 0, 0};
  state.translation_quality = robot::Quality::Good;
  state.heading_quality = robot::Quality::Good;
  return state;
}

robot::CommandContext context(robot::RobotState& state,
                              robot::TimeUs time_us,
                              std::uint32_t sequence = 1) {
  state.h.time_us = time_us;
  state.h.sequence = sequence;
  return {{time_us, sequence, state.h.mode_epoch},
          {robot::CompetitionMode::AutonomousInterface, true, false,
           state.h.mode_epoch, 0, 0},
          &state,
          0.01};
}

robot::OwnerToken owner(std::uint32_t epoch = 7) {
  return {11, robot::Requirement::kDrivetrain, 3, epoch};
}

const robot::ChassisVelocityPayload* velocity(
    const RecordingPublisher& publisher) {
  return std::get_if<robot::ChassisVelocityPayload>(&publisher.last.payload);
}

}  // namespace

ROBOT_TEST("drive distance command is one-tick nonblocking for both signs") {
  RecordingPublisher publisher;
  auto state = stateAt(0);
  robot::DriveDistanceCommand forward(11, 1.0, commandConfig(), publisher);
  forward.initialize(context(state, 0), owner());
  auto run = forward.execute(context(state, 100000, 2), owner());
  ROBOT_REQUIRE(run == robot::CommandRunState::Running);
  ROBOT_REQUIRE(forward.motionState() == robot::AutonomousMotionState::Running);
  ROBOT_REQUIRE(velocity(publisher) != nullptr);
  ROBOT_REQUIRE(velocity(publisher)->vx_mps > 0.0);
  ROBOT_REQUIRE(publisher.last.owner.command_id == 11);

  RecordingPublisher reverse_publisher;
  robot::DriveDistanceCommand reverse(12, -1.0, commandConfig(),
                                      reverse_publisher);
  auto reverse_owner = owner();
  reverse_owner.command_id = 12;
  reverse.initialize(context(state, 0), reverse_owner);
  run = reverse.execute(context(state, 100000, 3), reverse_owner);
  ROBOT_REQUIRE(run == robot::CommandRunState::Running);
  ROBOT_REQUIRE(velocity(reverse_publisher)->vx_mps < 0.0);
}

ROBOT_TEST("drive distance command settles then publishes brake") {
  RecordingPublisher publisher;
  auto state = stateAt(0);
  robot::DriveDistanceCommand command(11, 1.0, commandConfig(), publisher);
  command.initialize(context(state, 0), owner());
  state.pose.x_m = 1.0;
  state.body_velocity.vx_mps = 0.0;
  auto result = command.execute(context(state, 1600000, 2), owner());
  ROBOT_REQUIRE(result == robot::CommandRunState::Running);
  ROBOT_REQUIRE(command.motionState() ==
                robot::AutonomousMotionState::Settling);
  result = command.execute(context(state, 1700000, 3), owner());
  ROBOT_REQUIRE(result == robot::CommandRunState::Finished);
  ROBOT_REQUIRE(command.motionState() == robot::AutonomousMotionState::Success);
  ROBOT_REQUIRE(std::holds_alternative<robot::BrakePayload>(
      publisher.last.payload));
}

ROBOT_TEST("turn command wraps the shortest heading path") {
  RecordingPublisher publisher;
  auto state = stateAt(0);
  state.pose.theta_rad = 3.10;
  robot::TurnToHeadingCommand command(11, -3.10, commandConfig(), publisher);
  command.initialize(context(state, 0), owner());
  const auto result = command.execute(context(state, 10000, 2), owner());
  ROBOT_REQUIRE(result == robot::CommandRunState::Running);
  ROBOT_REQUIRE(velocity(publisher) != nullptr);
  ROBOT_REQUIRE(velocity(publisher)->vx_mps == 0.0);
  ROBOT_REQUIRE(velocity(publisher)->omega_radps > 0.0);
}

ROBOT_TEST("arc command publishes coupled linear and angular velocity") {
  RecordingPublisher publisher;
  auto state = stateAt(0);
  robot::DriveArcCommand command(11, 1.0, 1.0, 1, commandConfig(),
                                 publisher);
  command.initialize(context(state, 0), owner());
  const auto result = command.execute(context(state, 100000, 2), owner());
  ROBOT_REQUIRE(result == robot::CommandRunState::Running);
  ROBOT_REQUIRE(velocity(publisher) != nullptr);
  ROBOT_REQUIRE(velocity(publisher)->vx_mps > 0.0);
  ROBOT_REQUIRE(velocity(publisher)->omega_radps > 0.0);

  RecordingPublisher reverse_publisher;
  robot::DriveArcCommand reverse(12, 1.0, 1.0, -1, commandConfig(),
                                 reverse_publisher);
  auto reverse_owner = owner();
  reverse_owner.command_id = 12;
  reverse.initialize(context(state, 0), reverse_owner);
  ROBOT_REQUIRE(reverse.execute(context(state, 100000, 3), reverse_owner) ==
                robot::CommandRunState::Running);
  ROBOT_REQUIRE(velocity(reverse_publisher)->vx_mps < 0.0);
  ROBOT_REQUIRE(velocity(reverse_publisher)->omega_radps > 0.0);
}

ROBOT_TEST("drive to pose chooses reverse without changing final heading goal") {
  RecordingPublisher publisher;
  auto state = stateAt(0);
  robot::DriveToPoseCommand command(11, {-1.0, 0.0, 0.0}, true,
                                    commandConfig(), publisher);
  command.initialize(context(state, 0), owner());
  const auto result = command.execute(context(state, 10000, 2), owner());
  ROBOT_REQUIRE(result == robot::CommandRunState::Running);
  ROBOT_REQUIRE(velocity(publisher) != nullptr);
  ROBOT_REQUIRE(velocity(publisher)->vx_mps < 0.0);
  ROBOT_REQUIRE(std::abs(velocity(publisher)->omega_radps) < 1e-12);
}

ROBOT_TEST("brake command remains nonblocking until velocity settles") {
  RecordingPublisher publisher;
  auto state = stateAt(0);
  robot::BrakeCommand command(11, commandConfig(), publisher);
  command.initialize(context(state, 0), owner());
  state.body_velocity.vx_mps = 0.2;
  ROBOT_REQUIRE(command.execute(context(state, 0, 2), owner()) ==
                robot::CommandRunState::Running);
  ROBOT_REQUIRE(std::holds_alternative<robot::BrakePayload>(
      publisher.last.payload));
  state.body_velocity.vx_mps = 0.0;
  ROBOT_REQUIRE(command.execute(context(state, 100000, 3), owner()) ==
                robot::CommandRunState::Running);
  ROBOT_REQUIRE(command.execute(context(state, 200000, 4), owner()) ==
                robot::CommandRunState::Finished);
}

ROBOT_TEST("motion commands deterministically fail timeout stall and bad state") {
  RecordingPublisher timeout_publisher;
  auto state = stateAt(0);
  robot::DriveDistanceCommand timeout(11, 10.0, commandConfig(),
                                      timeout_publisher);
  timeout.initialize(context(state, 0), owner());
  ROBOT_REQUIRE(timeout.execute(context(state, 3000000, 2), owner()) ==
                robot::CommandRunState::Failed);
  ROBOT_REQUIRE(timeout.motionState() ==
                robot::AutonomousMotionState::Timeout);

  auto stall_config = commandConfig();
  stall_config.linear_profile.max_acceleration = 20.0;
  RecordingPublisher stall_publisher;
  robot::DriveDistanceCommand stall(11, 10.0, stall_config, stall_publisher);
  state = stateAt(0);
  stall.initialize(context(state, 0), owner());
  ROBOT_REQUIRE(stall.execute(context(state, 100000, 3), owner()) ==
                robot::CommandRunState::Running);
  ROBOT_REQUIRE(stall.execute(context(state, 300000, 4), owner()) ==
                robot::CommandRunState::Failed);
  ROBOT_REQUIRE(stall.motionState() ==
                robot::AutonomousMotionState::Stalled);

  RecordingPublisher invalid_publisher;
  robot::DriveDistanceCommand invalid(11, 1.0, commandConfig(),
                                      invalid_publisher);
  state = stateAt(0);
  invalid.initialize(context(state, 0), owner());
  state.translation_quality = robot::Quality::Invalid;
  ROBOT_REQUIRE(invalid.execute(context(state, 10000, 5), owner()) ==
                robot::CommandRunState::Failed);
  ROBOT_REQUIRE(invalid.motionState() ==
                robot::AutonomousMotionState::StateInvalid);
  ROBOT_REQUIRE(std::holds_alternative<robot::BrakePayload>(
      invalid_publisher.last.payload));
}

ROBOT_TEST("cancel and mode boundary cannot leave an old motion request") {
  RecordingPublisher publisher;
  auto state = stateAt(0);
  robot::DriveDistanceCommand command(11, 1.0, commandConfig(), publisher);
  command.initialize(context(state, 0), owner());
  ROBOT_REQUIRE(command.execute(context(state, 100000, 2), owner()) ==
                robot::CommandRunState::Running);
  ROBOT_REQUIRE(std::holds_alternative<robot::ChassisVelocityPayload>(
      publisher.last.payload));
  command.end(context(state, 110000, 3), owner(),
              robot::CommandEndReason::Cancelled);
  ROBOT_REQUIRE(command.motionState() ==
                robot::AutonomousMotionState::Interrupted);
  ROBOT_REQUIRE(std::holds_alternative<robot::BrakePayload>(
      publisher.last.payload));

  auto next_state = stateAt(120000, 4, 8);
  auto next_owner = owner(8);
  ROBOT_REQUIRE(command.execute(context(next_state, 120000, 4), next_owner) ==
                robot::CommandRunState::Failed);
  ROBOT_REQUIRE(command.motionState() ==
                robot::AutonomousMotionState::StateInvalid);
}
