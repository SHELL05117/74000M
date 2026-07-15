#include "robot/autonomy/safety_supervisor.hpp"
#include "robot/commands/drive_request_arbiter.hpp"
#include "test_framework.hpp"

#include <array>
#include <variant>

namespace {

const robot::FixedFailurePolicy<7>& failurePolicy() {
  static const std::array<robot::FailurePolicyEntry, 7> entries{{
      {robot::AutonomousOutcome::Timeout, robot::FailureAction::Retry, 1,
       robot::FailureAction::EndRoute},
      {robot::AutonomousOutcome::Stalled, robot::FailureAction::Skip, 0,
       robot::FailureAction::EndRoute},
      {robot::AutonomousOutcome::StateInvalid,
       robot::FailureAction::EndRoute, 0, robot::FailureAction::EndRoute},
      {robot::AutonomousOutcome::Interrupted,
       robot::FailureAction::EndRoute, 0, robot::FailureAction::EndRoute},
      {robot::AutonomousOutcome::Collision,
       robot::FailureAction::EmergencyStop, 0,
       robot::FailureAction::EmergencyStop},
      {robot::AutonomousOutcome::Deviation, robot::FailureAction::Retry, 1,
       robot::FailureAction::EndRoute},
      {robot::AutonomousOutcome::MechanismConflict,
       robot::FailureAction::EndRoute, 0, robot::FailureAction::EndRoute},
  }};
  static const robot::FixedFailurePolicy<7> policy(entries);
  return policy;
}

robot::AutonomySafetyConfig safetyConfig() {
  return {0.5, 0.5, 100000, 50000, 30000,
          robot::DegradedAutonomyPolicy::AbortAfterGrace};
}

robot::ModeSnapshot autoMode(std::uint32_t epoch = 7) {
  return {robot::CompetitionMode::AutonomousInterface, true, false, epoch, 0,
          0};
}

robot::OwnerToken commandOwner(std::uint32_t lease = 1,
                               robot::CommandId id = 20) {
  return {id, robot::Requirement::kDrivetrain, lease, 7};
}

robot::RobotState healthyState(robot::TimeUs time_us = 0) {
  robot::RobotState state{};
  state.h = {time_us, 1, 7};
  state.competition = autoMode();
  state.translation_quality = robot::Quality::Good;
  state.heading_quality = robot::Quality::Good;
  return state;
}

robot::AutonomySafetyInput safetyInput(robot::RobotState& state,
                                       robot::TimeUs time_us,
                                       const robot::OwnerToken& owner) {
  state.h.time_us = time_us;
  return {{time_us, static_cast<std::uint32_t>(time_us / 1000 + 1), 7},
          autoMode(),
          &state,
          owner,
          robot::Quality::Good,
          0.0,
          0,
          false,
          false,
          robot::AutonomousOutcome::Success};
}

robot::DriveRequest chassisRequest(const robot::OwnerToken& owner,
                                   robot::TimeUs time_us = 0) {
  robot::DriveRequest request{};
  request.h = {time_us, 1, 7};
  request.source = robot::RequestSource::FutureAutonomy;
  request.owner = owner;
  request.ttl_us = 30000;
  request.payload = robot::ChassisVelocityPayload{1.0, 0.4};
  return request;
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

}  // namespace

ROBOT_TEST("failure policy is complete unique and rejects invalid retry") {
  ROBOT_REQUIRE(failurePolicy().valid());
  ROBOT_REQUIRE(failurePolicy().find(robot::AutonomousOutcome::Collision) !=
                nullptr);
  const std::array<robot::FailurePolicyEntry, 1> invalid_entries{{
      {robot::AutonomousOutcome::Timeout, robot::FailureAction::Retry, 0,
       robot::FailureAction::Retry},
  }};
  ROBOT_REQUIRE(!robot::FixedFailurePolicy<1>(invalid_entries).valid());
}

ROBOT_TEST("degraded autonomy is filtered then aborts after grace") {
  robot::AutonomySafetySupervisor<7> supervisor(safetyConfig(),
                                                 failurePolicy());
  ROBOT_REQUIRE(supervisor.startRoute(0x101, autoMode(), 0));
  const auto owner = commandOwner();
  ROBOT_REQUIRE(supervisor.beginCommand(owner, autoMode()));
  auto state = healthyState();
  auto input = safetyInput(state, 0, owner);
  auto decision = supervisor.evaluate(input);
  ROBOT_REQUIRE(decision.action == robot::AutonomyDecisionAction::Continue);
  ROBOT_REQUIRE(!decision.has_safety_request);

  state.translation_quality = robot::Quality::Degraded;
  input = safetyInput(state, 1000, owner);
  decision = supervisor.evaluate(input);
  ROBOT_REQUIRE(decision.action ==
                robot::AutonomyDecisionAction::ContinueDegraded);
  ROBOT_REQUIRE_NEAR(decision.speed_scale, 0.5, 0.0);
  robot::DriveRequest filtered{};
  ROBOT_REQUIRE(supervisor.filterChassisRequest(
      decision, chassisRequest(owner, 1000), filtered));
  const auto* velocity =
      std::get_if<robot::ChassisVelocityPayload>(&filtered.payload);
  ROBOT_REQUIRE(velocity != nullptr);
  ROBOT_REQUIRE_NEAR(velocity->vx_mps, 0.5, 0.0);
  ROBOT_REQUIRE_NEAR(velocity->omega_radps, 0.2, 0.0);

  input = safetyInput(state, 101000, owner);
  decision = supervisor.evaluate(input);
  ROBOT_REQUIRE(decision.action == robot::AutonomyDecisionAction::EndRoute);
  ROBOT_REQUIRE(decision.outcome == robot::AutonomousOutcome::StateInvalid);
  ROBOT_REQUIRE(decision.has_safety_request && decision.latched);
  ROBOT_REQUIRE(decision.safety_request.source == robot::RequestSource::Safety);
  ROBOT_REQUIRE(std::holds_alternative<robot::BrakePayload>(
      decision.safety_request.payload));
}

ROBOT_TEST("retry requires new lease brake barrier and absent condition") {
  robot::AutonomySafetySupervisor<7> supervisor(safetyConfig(),
                                                 failurePolicy());
  ROBOT_REQUIRE(supervisor.startRoute(0x101, autoMode(), 0));
  const auto first_owner = commandOwner(1);
  ROBOT_REQUIRE(supervisor.beginCommand(first_owner, autoMode()));
  auto state = healthyState(1000);
  auto input = safetyInput(state, 1000, first_owner);
  input.command_terminal = true;
  input.command_outcome = robot::AutonomousOutcome::Timeout;
  auto decision = supervisor.evaluate(input);
  ROBOT_REQUIRE(decision.action ==
                robot::AutonomyDecisionAction::RetryRequired);
  ROBOT_REQUIRE(decision.failure.valid);
  ROBOT_REQUIRE(decision.failure.route_id == 0x101);
  ROBOT_REQUIRE(decision.failure.command_id == first_owner.command_id);
  ROBOT_REQUIRE(decision.failure.attempt == 1);
  ROBOT_REQUIRE(!supervisor.authorizeRetry(first_owner, autoMode(), true,
                                           true));
  const auto retry_owner = commandOwner(2);
  ROBOT_REQUIRE(!supervisor.authorizeRetry(retry_owner, autoMode(), false,
                                           true));
  ROBOT_REQUIRE(!supervisor.authorizeRetry(retry_owner, autoMode(), true,
                                           false));
  ROBOT_REQUIRE(supervisor.authorizeRetry(retry_owner, autoMode(), true,
                                          true));
  ROBOT_REQUIRE(supervisor.attempt() == 2);

  input = safetyInput(state, 2000, first_owner);
  decision = supervisor.evaluate(input);
  ROBOT_REQUIRE(decision.action ==
                robot::AutonomyDecisionAction::EmergencyStop);
  ROBOT_REQUIRE(!supervisor.latched());
  ROBOT_REQUIRE(robot::sameLease(supervisor.activeOwner(), retry_owner));
  robot::DriveRequest filtered{};
  ROBOT_REQUIRE(!supervisor.filterChassisRequest(
      {}, chassisRequest(first_owner, 2000), filtered));

  input = safetyInput(state, 3000, retry_owner);
  decision = supervisor.evaluate(input);
  ROBOT_REQUIRE(decision.action == robot::AutonomyDecisionAction::Continue);
  input.command_terminal = true;
  input.command_outcome = robot::AutonomousOutcome::Timeout;
  decision = supervisor.evaluate(input);
  ROBOT_REQUIRE(decision.action == robot::AutonomyDecisionAction::EndRoute);
}

ROBOT_TEST("stall skips only after brake barrier with a fresh owner") {
  robot::AutonomySafetySupervisor<7> supervisor(safetyConfig(),
                                                 failurePolicy());
  ROBOT_REQUIRE(supervisor.startRoute(0x101, autoMode(), 0));
  const auto first = commandOwner(1, 20);
  ROBOT_REQUIRE(supervisor.beginCommand(first, autoMode()));
  auto state = healthyState();
  auto input = safetyInput(state, 1000, first);
  input.command_terminal = true;
  input.command_outcome = robot::AutonomousOutcome::Stalled;
  const auto decision = supervisor.evaluate(input);
  ROBOT_REQUIRE(decision.action ==
                robot::AutonomyDecisionAction::SkipCommand);
  const auto next = commandOwner(2, 21);
  ROBOT_REQUIRE(!supervisor.advanceAfterSkip(next, autoMode(), false));
  ROBOT_REQUIRE(supervisor.advanceAfterSkip(next, autoMode(), true));
  ROBOT_REQUIRE(robot::sameLease(supervisor.activeOwner(), next));
}

ROBOT_TEST("collision deviation mechanism conflict and invalid state are explicit") {
  auto run_failure = [](robot::AutonomousOutcome expected,
                        auto mutate) {
    robot::AutonomySafetySupervisor<7> supervisor(safetyConfig(),
                                                   failurePolicy());
    ROBOT_REQUIRE(supervisor.startRoute(0x101, autoMode(), 0));
    const auto owner = commandOwner();
    ROBOT_REQUIRE(supervisor.beginCommand(owner, autoMode()));
    auto state = healthyState();
    auto input = safetyInput(state, 1000, owner);
    mutate(input, state);
    const auto decision = supervisor.evaluate(input);
    ROBOT_REQUIRE(decision.outcome == expected);
    ROBOT_REQUIRE(decision.has_safety_request);
    return decision.action;
  };

  ROBOT_REQUIRE(run_failure(robot::AutonomousOutcome::Collision,
                            [](auto& input, auto&) {
                              input.collision_detected = true;
                            }) ==
                robot::AutonomyDecisionAction::EmergencyStop);
  ROBOT_REQUIRE(run_failure(robot::AutonomousOutcome::Deviation,
                            [](auto& input, auto&) {
                              input.deviation_m = 0.6;
                            }) == robot::AutonomyDecisionAction::RetryRequired);
  ROBOT_REQUIRE(run_failure(robot::AutonomousOutcome::MechanismConflict,
                            [](auto& input, auto&) {
                              input.mechanism_conflict_bits = 1;
                            }) == robot::AutonomyDecisionAction::EndRoute);
  ROBOT_REQUIRE(run_failure(robot::AutonomousOutcome::StateInvalid,
                            [](auto&, auto& state) {
                              state.heading_quality = robot::Quality::Invalid;
                            }) == robot::AutonomyDecisionAction::EndRoute);
}

ROBOT_TEST("safety brake wins arbitration over an old autonomy request") {
  robot::AutonomySafetySupervisor<7> supervisor(safetyConfig(),
                                                 failurePolicy());
  ROBOT_REQUIRE(supervisor.startRoute(0x101, autoMode(), 0));
  const auto owner = commandOwner();
  ROBOT_REQUIRE(supervisor.beginCommand(owner, autoMode()));
  auto state = healthyState();
  auto input = safetyInput(state, 1000, owner);
  input.collision_detected = true;
  const auto decision = supervisor.evaluate(input);
  std::array<robot::DriveRequestCandidate, 2> candidates{{
      {chassisRequest(owner, 1000), true},
      {decision.safety_request, true},
  }};
  robot::DriveCapabilities capabilities{};
  capabilities.autonomous_chassis_velocity = true;
  const ExactAuthority authority(owner);
  robot::DriveRequestArbiter arbiter({40000});
  const auto selected = arbiter.select(candidates, autoMode(), 1000,
                                       capabilities, authority);
  ROBOT_REQUIRE(selected.has_selection);
  ROBOT_REQUIRE(selected.selected.source == robot::RequestSource::Safety);
  ROBOT_REQUIRE(std::holds_alternative<robot::BrakePayload>(
      selected.selected.payload));
}

ROBOT_TEST("mode interruption latches end route and success permits next command") {
  robot::AutonomySafetySupervisor<7> interrupted(safetyConfig(),
                                                  failurePolicy());
  ROBOT_REQUIRE(interrupted.startRoute(0x101, autoMode(), 0));
  const auto owner = commandOwner();
  ROBOT_REQUIRE(interrupted.beginCommand(owner, autoMode()));
  auto state = healthyState();
  auto input = safetyInput(state, 1000, owner);
  input.mode = {robot::CompetitionMode::Disabled, false, false, 8, 0, 0};
  auto decision = interrupted.evaluate(input);
  ROBOT_REQUIRE(decision.outcome == robot::AutonomousOutcome::Interrupted);
  ROBOT_REQUIRE(decision.action == robot::AutonomyDecisionAction::EndRoute);

  robot::AutonomySafetySupervisor<7> success(safetyConfig(), failurePolicy());
  ROBOT_REQUIRE(success.startRoute(0x101, autoMode(), 0));
  ROBOT_REQUIRE(success.beginCommand(owner, autoMode()));
  input = safetyInput(state, 1000, owner);
  input.command_terminal = true;
  input.command_outcome = robot::AutonomousOutcome::Success;
  decision = success.evaluate(input);
  ROBOT_REQUIRE(decision.action == robot::AutonomyDecisionAction::Continue);
  ROBOT_REQUIRE(!decision.has_safety_request);
  ROBOT_REQUIRE(success.beginCommand(commandOwner(2, 21), autoMode()));
}
