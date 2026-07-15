#include "robot/autonomy/route_registry.hpp"
#include "robot/commands/command_groups.hpp"
#include "robot/commands/mechanism_commands.hpp"
#include "test_framework.hpp"

#include <array>
#include <variant>

namespace {

class FakeGroupCommand final : public robot::Command {
 public:
  FakeGroupCommand(robot::CommandId id, robot::RequirementMask requirement,
                   int finish_after, int fail_at = 0)
      : id_(id),
        requirement_(requirement),
        finish_after_(finish_after),
        fail_at_(fail_at) {}
  robot::CommandId id() const noexcept override { return id_; }
  robot::RequirementMask requirements() const noexcept override {
    return requirement_;
  }
  bool allowedInMode(robot::CompetitionMode mode) const noexcept override {
    return mode == robot::CompetitionMode::AutonomousInterface;
  }
  void initialize(const robot::CommandContext&,
                  const robot::OwnerToken&) noexcept override {
    ++initialize_count;
    execute_count = 0;
  }
  robot::CommandRunState execute(const robot::CommandContext&,
                                 const robot::OwnerToken&) noexcept override {
    ++execute_count;
    if (fail_at_ > 0 && execute_count >= fail_at_)
      return robot::CommandRunState::Failed;
    return execute_count >= finish_after_ ? robot::CommandRunState::Finished
                                          : robot::CommandRunState::Running;
  }
  void end(const robot::CommandContext&, const robot::OwnerToken&,
           robot::CommandEndReason reason) noexcept override {
    ++end_count;
    last_reason = reason;
  }

  int initialize_count{};
  int execute_count{};
  int end_count{};
  robot::CommandEndReason last_reason{robot::CommandEndReason::Completed};

 private:
  robot::CommandId id_{};
  robot::RequirementMask requirement_{};
  int finish_after_{};
  int fail_at_{};
};

robot::RobotState routeState(robot::TimeUs time_us = 0) {
  robot::RobotState state{};
  state.h = {time_us, 1, 7};
  state.competition = {robot::CompetitionMode::AutonomousInterface, true,
                       false, 7, 0, 0};
  state.translation_quality = robot::Quality::Good;
  state.heading_quality = robot::Quality::Good;
  return state;
}

robot::CommandContext autoContext(robot::RobotState& state,
                                  robot::TimeUs time_us,
                                  std::uint32_t sequence = 1) {
  state.h.time_us = time_us;
  state.h.sequence = sequence;
  return {{time_us, sequence, 7},
          {robot::CompetitionMode::AutonomousInterface, true, false, 7, 0, 0},
          &state,
          0.01};
}

robot::OwnerToken groupOwner(robot::RequirementMask requirements) {
  return {99, requirements, 5, 7};
}

class RecordingDrivePublisher final : public robot::DriveRequestPublisher {
 public:
  bool publish(const robot::DriveRequest& request) noexcept override {
    last = request;
    ++count;
    return true;
  }
  robot::DriveRequest last{};
  int count{};
};

class RecordingMechanismPublisher final
    : public robot::MechanismRequestPublisher {
 public:
  bool publish(const robot::MechanismRequest& request) noexcept override {
    last = request;
    ++count;
    return true;
  }
  robot::MechanismRequest last{};
  int count{};
};

bool chooseTrue(const robot::CommandContext& context) noexcept {
  return context.state != nullptr && context.state->controller_connected;
}

robot::Command* routeFactory(robot::RouteBuildContext& context) noexcept {
  return static_cast<robot::Command*>(context.user_context);
}

const robot::RouteRegistry<3>& routes() {
  static const std::array<robot::RouteDescriptor, 3> descriptors{{
      {robot::RouteIds::kDoNothing, "DO-NOTHING", "Do Nothing",
       robot::kAllianceAny, robot::kStartAny, {},
       robot::RouteQualityRequirement::None, 0, true, true},
      {0x101, "R-NEAR", "Red Near", robot::kAllianceRed,
       robot::kStartNear, {1.0, 2.0, 0.5},
       robot::RouteQualityRequirement::FullPoseGood, 0, true, true},
      {0x102, "FUTURE", "Future Route", robot::kAllianceAny,
       robot::kStartAny, {}, robot::RouteQualityRequirement::None, 0, false,
       false},
  }};
  static const robot::RouteRegistry<3> registry(descriptors, 12);
  return registry;
}

robot::AutonSelectionSnapshot lockedRoute(robot::RouteId id = 0x101) {
  robot::AutonSelectionSnapshot snapshot{};
  snapshot.route_id = id;
  snapshot.alliance = robot::Alliance::Red;
  snapshot.start_side = robot::StartSide::Near;
  snapshot.expected_start_pose = {1.0, 2.0, 0.5};
  snapshot.selection_revision = 3;
  snapshot.route_registry_revision = 12;
  snapshot.mode_epoch = 7;
  snapshot.valid = true;
  return snapshot;
}

}  // namespace

ROBOT_TEST("sequential group starts each child only after prior completion") {
  FakeGroupCommand first(1, robot::Requirement::kDrivetrain, 1);
  FakeGroupCommand second(2, robot::Requirement::kIntake, 1);
  std::array<robot::Command*, 2> children{{&first, &second}};
  robot::SequentialCommandGroup<2> group(99, children, 2);
  auto state = routeState();
  const auto owner = groupOwner(robot::Requirement::kDrivetrain |
                                robot::Requirement::kIntake);
  group.initialize(autoContext(state, 0), owner);
  ROBOT_REQUIRE(first.initialize_count == 1 && second.initialize_count == 0);
  ROBOT_REQUIRE(group.execute(autoContext(state, 10000, 2), owner) ==
                robot::CommandRunState::Running);
  ROBOT_REQUIRE(first.end_count == 1 && second.initialize_count == 1);
  ROBOT_REQUIRE(group.execute(autoContext(state, 20000, 3), owner) ==
                robot::CommandRunState::Finished);
  ROBOT_REQUIRE(second.end_count == 1);
}

ROBOT_TEST("parallel race and deadline groups have deterministic endings") {
  auto state = routeState();
  const auto owner = groupOwner(robot::Requirement::kDrivetrain |
                                robot::Requirement::kIntake);
  FakeGroupCommand p1(1, robot::Requirement::kDrivetrain, 1);
  FakeGroupCommand p2(2, robot::Requirement::kIntake, 2);
  std::array<robot::Command*, 2> parallel_children{{&p1, &p2}};
  robot::ParallelCommandGroup<2> parallel(99, parallel_children, 2);
  parallel.initialize(autoContext(state, 0), owner);
  ROBOT_REQUIRE(parallel.execute(autoContext(state, 10000, 2), owner) ==
                robot::CommandRunState::Running);
  ROBOT_REQUIRE(parallel.execute(autoContext(state, 20000, 3), owner) ==
                robot::CommandRunState::Finished);

  FakeGroupCommand winner(3, robot::Requirement::kDrivetrain, 1);
  FakeGroupCommand loser(4, robot::Requirement::kIntake, 10);
  std::array<robot::Command*, 2> race_children{{&winner, &loser}};
  robot::RaceCommandGroup<2> race(98, race_children, 2);
  const auto race_owner = groupOwner(robot::Requirement::kDrivetrain |
                                     robot::Requirement::kIntake);
  race.initialize(autoContext(state, 0), race_owner);
  ROBOT_REQUIRE(race.execute(autoContext(state, 10000, 4), race_owner) ==
                robot::CommandRunState::Finished);
  ROBOT_REQUIRE(loser.last_reason == robot::CommandEndReason::Interrupted);

  FakeGroupCommand deadline_child(5, robot::Requirement::kDrivetrain, 2);
  FakeGroupCommand background(6, robot::Requirement::kIntake, 10);
  std::array<robot::Command*, 2> deadline_children{{&deadline_child,
                                                    &background}};
  robot::DeadlineCommandGroup<2> deadline(97, deadline_children, 2, 0);
  deadline.initialize(autoContext(state, 0), race_owner);
  ROBOT_REQUIRE(deadline.execute(autoContext(state, 10000, 5), race_owner) ==
                robot::CommandRunState::Running);
  ROBOT_REQUIRE(deadline.execute(autoContext(state, 20000, 6), race_owner) ==
                robot::CommandRunState::Finished);
  ROBOT_REQUIRE(background.last_reason ==
                robot::CommandEndReason::Interrupted);
}

ROBOT_TEST("parallel groups reject overlapping subsystem ownership") {
  FakeGroupCommand first(1, robot::Requirement::kDrivetrain, 1);
  FakeGroupCommand second(2, robot::Requirement::kDrivetrain, 1);
  std::array<robot::Command*, 2> children{{&first, &second}};
  robot::ParallelCommandGroup<2> group(99, children, 2);
  auto state = routeState();
  const auto owner = groupOwner(robot::Requirement::kDrivetrain);
  group.initialize(autoContext(state, 0), owner);
  ROBOT_REQUIRE(group.execute(autoContext(state, 10000, 2), owner) ==
                robot::CommandRunState::Failed);
  ROBOT_REQUIRE(first.initialize_count == 0 && second.initialize_count == 0);
}

ROBOT_TEST("wait timeout and conditional commands are nonblocking and frozen") {
  auto state = routeState();
  robot::WaitCommand wait(7, 100000);
  robot::StaticScheduler<1> scheduler;
  auto scheduled = scheduler.schedule(
      wait, autoContext(state, 0), robot::ConflictPolicy::RejectIncoming);
  ROBOT_REQUIRE(scheduled.accepted);
  scheduler.tick(autoContext(state, 50000, 2));
  ROBOT_REQUIRE(scheduler.activeCount() == 1);
  scheduler.tick(autoContext(state, 100000, 3));
  ROBOT_REQUIRE(scheduler.activeCount() == 0);

  FakeGroupCommand slow(8, robot::Requirement::kDrivetrain, 100);
  robot::TimeoutCommand timeout(9, slow, 100000);
  const auto drive_owner = groupOwner(robot::Requirement::kDrivetrain);
  timeout.initialize(autoContext(state, 0), drive_owner);
  ROBOT_REQUIRE(timeout.execute(autoContext(state, 100000, 4), drive_owner) ==
                robot::CommandRunState::Failed);
  ROBOT_REQUIRE(timeout.timedOut());

  FakeGroupCommand true_child(10, robot::Requirement::kDrivetrain, 1);
  FakeGroupCommand false_child(11, robot::Requirement::kIntake, 1);
  robot::ConditionalCommand conditional(12, true_child, false_child,
                                        chooseTrue);
  state.controller_connected = true;
  const auto conditional_owner = groupOwner(robot::Requirement::kDrivetrain |
                                            robot::Requirement::kIntake);
  conditional.initialize(autoContext(state, 0), conditional_owner);
  state.controller_connected = false;
  ROBOT_REQUIRE(conditional.execute(autoContext(state, 10000, 5),
                                    conditional_owner) ==
                robot::CommandRunState::Finished);
  ROBOT_REQUIRE(true_child.execute_count == 1);
  ROBOT_REQUIRE(false_child.execute_count == 0);
}

ROBOT_TEST("mechanism command publishes a typed leased request only") {
  RecordingMechanismPublisher publisher;
  robot::InstantMechanismCommand command(
      13, robot::Requirement::kIntake, 1,
      robot::MechanismAction::SetNormalizedOutput, 0.75, 30000, publisher);
  auto state = routeState();
  const auto owner = groupOwner(robot::Requirement::kIntake);
  command.initialize(autoContext(state, 0), owner);
  ROBOT_REQUIRE(command.execute(autoContext(state, 10000, 2), owner) ==
                robot::CommandRunState::Finished);
  ROBOT_REQUIRE(publisher.count == 1);
  ROBOT_REQUIRE(publisher.last.mechanism_id == 1);
  ROBOT_REQUIRE_NEAR(publisher.last.value, 0.75, 0.0);
  ROBOT_REQUIRE(publisher.last.owner.command_id == 99);
}

ROBOT_TEST("route registry resolves stable approved ID through factory") {
  RecordingDrivePublisher drive_publisher;
  robot::DoNothingCommand do_nothing(90, 30000, drive_publisher);
  FakeGroupCommand route_command(50, robot::Requirement::kDrivetrain, 10);
  const std::array<robot::RouteFactoryDescriptor, 1> factories{{
      {0x101, routeFactory},
  }};
  robot::AutonomousRouteRegistry<3, 1> registry(
      routes(), factories, do_nothing, {0.05, 0.05});
  ROBOT_REQUIRE(registry.valid());
  auto state = routeState();
  state.pose = {1.0, 2.0, 0.5};
  robot::RouteBuildContext build{&route_command};
  const auto result = registry.resolve(
      lockedRoute(), state.competition, state, build);
  ROBOT_REQUIRE(!result.fallback_to_do_nothing);
  ROBOT_REQUIRE(result.resolved_route == 0x101);
  ROBOT_REQUIRE(result.command == &route_command);
  ROBOT_REQUIRE(result.reject_bits == robot::kRouteResolutionAccepted);
}

ROBOT_TEST("route registry falls back for quality pose ID and revision failures") {
  RecordingDrivePublisher drive_publisher;
  robot::DoNothingCommand do_nothing(90, 30000, drive_publisher);
  FakeGroupCommand route_command(50, robot::Requirement::kDrivetrain, 10);
  const std::array<robot::RouteFactoryDescriptor, 1> factories{{
      {0x101, routeFactory},
  }};
  robot::AutonomousRouteRegistry<3, 1> registry(
      routes(), factories, do_nothing, {0.05, 0.05});
  robot::RouteBuildContext build{&route_command};
  auto state = routeState();
  state.pose = {1.0, 2.0, 0.5};

  state.translation_quality = robot::Quality::Degraded;
  auto result = registry.resolve(lockedRoute(), state.competition, state, build);
  ROBOT_REQUIRE(result.fallback_to_do_nothing);
  ROBOT_REQUIRE((result.reject_bits & robot::kRouteResolutionQuality) != 0);

  state.translation_quality = robot::Quality::Good;
  state.pose.x_m += 0.1;
  result = registry.resolve(lockedRoute(), state.competition, state, build);
  ROBOT_REQUIRE((result.reject_bits & robot::kRouteResolutionStartPose) != 0);

  state.pose = {1.0, 2.0, 0.5};
  auto invalid_id = lockedRoute(0x999);
  result = registry.resolve(invalid_id, state.competition, state, build);
  ROBOT_REQUIRE((result.reject_bits & robot::kRouteResolutionNotApproved) != 0);
  auto stale = lockedRoute();
  stale.route_registry_revision = 11;
  result = registry.resolve(stale, state.competition, state, build);
  ROBOT_REQUIRE((result.reject_bits & robot::kRouteResolutionRevision) != 0);
  ROBOT_REQUIRE(result.command == &do_nothing);
}

ROBOT_TEST("DoNothing continuously publishes brake with the group lease") {
  RecordingDrivePublisher publisher;
  robot::DoNothingCommand command(90, 30000, publisher);
  auto state = routeState();
  const auto owner = groupOwner(robot::Requirement::kDrivetrain);
  command.initialize(autoContext(state, 0), owner);
  ROBOT_REQUIRE(command.execute(autoContext(state, 10000, 2), owner) ==
                robot::CommandRunState::Running);
  ROBOT_REQUIRE(std::holds_alternative<robot::BrakePayload>(
      publisher.last.payload));
  ROBOT_REQUIRE(publisher.last.owner.command_id == 99);
}
