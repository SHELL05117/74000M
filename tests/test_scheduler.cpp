#include "robot/commands/drive_request_arbiter.hpp"
#include "robot/commands/request_sink.hpp"
#include "robot/commands/scheduler.hpp"
#include "test_framework.hpp"

#include <array>

namespace {

class FakeCommand final : public robot::Command {
 public:
  FakeCommand(robot::CommandId command_id, robot::RequirementMask requirements,
              bool can_interrupt = true)
      : id_(command_id),
        requirements_(requirements),
        interruptible_(can_interrupt) {}

  robot::CommandId id() const noexcept override { return id_; }
  robot::RequirementMask requirements() const noexcept override {
    return requirements_;
  }
  bool allowedInMode(robot::CompetitionMode mode) const noexcept override {
    return mode == robot::CompetitionMode::Driver;
  }
  bool interruptible() const noexcept override { return interruptible_; }
  void initialize(const robot::CommandContext&,
                  const robot::OwnerToken& owner) noexcept override {
    ++initialize_count;
    last_owner = owner;
  }
  robot::CommandRunState execute(const robot::CommandContext&,
                                 const robot::OwnerToken&) noexcept override {
    ++execute_count;
    return next_state;
  }
  void end(const robot::CommandContext&, const robot::OwnerToken&,
           robot::CommandEndReason reason) noexcept override {
    ++end_count;
    last_end_reason = reason;
  }

  robot::CommandRunState next_state{robot::CommandRunState::Running};
  robot::OwnerToken last_owner{};
  robot::CommandEndReason last_end_reason{robot::CommandEndReason::Completed};
  int initialize_count{};
  int execute_count{};
  int end_count{};

 private:
  robot::CommandId id_{};
  robot::RequirementMask requirements_{};
  bool interruptible_{};
};

class FakeSubsystem final : public robot::Subsystem {
 public:
  explicit FakeSubsystem(robot::RequirementMask requirement)
      : requirement_(requirement) {}
  robot::RequirementMask requirement() const noexcept override {
    return requirement_;
  }
  void periodic(const robot::CommandContext&) noexcept override {
    ++periodic_count;
  }
  void onModeBoundary(const robot::CommandContext&) noexcept override {
    ++boundary_count;
  }
  int periodic_count{};
  int boundary_count{};

 private:
  robot::RequirementMask requirement_{};
};

robot::CommandContext context(std::uint32_t epoch = 1,
                              std::uint32_t sequence = 1) {
  robot::CommandContext value{};
  value.h = {sequence * 10000u, sequence, epoch};
  value.mode =
      {robot::CompetitionMode::Driver, true, false, epoch, 0, 0};
  value.dt_s = 0.01;
  return value;
}

robot::DriveRequest neutralRequest(const robot::FrameHeader& header,
                                   const robot::OwnerToken& owner) {
  robot::DriveRequest request{};
  request.h = header;
  request.source = robot::RequestSource::Driver;
  request.owner = owner;
  request.ttl_us = 30000;
  request.payload = robot::DriverCurvaturePayload{
      0.0, 0.0, robot::DriverSteeringMode::Curvature,
      robot::AllocationPolicy::RatioPreserving};
  return request;
}

}  // namespace

ROBOT_TEST("scheduler conflict policy invalidates the interrupted lease") {
  robot::StaticScheduler<2> scheduler;
  FakeCommand first(1, robot::Requirement::kDrivetrain);
  FakeCommand second(2, robot::Requirement::kDrivetrain);
  const auto first_result = scheduler.schedule(
      first, context(), robot::ConflictPolicy::RejectIncoming);
  ROBOT_REQUIRE(first_result.accepted);
  ROBOT_REQUIRE(scheduler.owns(first_result.owner));

  auto second_result = scheduler.schedule(
      second, context(), robot::ConflictPolicy::RejectIncoming);
  ROBOT_REQUIRE(!second_result.accepted);
  ROBOT_REQUIRE((second_result.reject_bits & robot::kScheduleConflict) != 0);

  second_result = scheduler.schedule(
      second, context(), robot::ConflictPolicy::InterruptExisting);
  ROBOT_REQUIRE(second_result.accepted);
  ROBOT_REQUIRE(first.end_count == 1);
  ROBOT_REQUIRE(first.last_end_reason == robot::CommandEndReason::Interrupted);
  ROBOT_REQUIRE(!scheduler.owns(first_result.owner));
  ROBOT_REQUIRE(scheduler.owns(second_result.owner));
}

ROBOT_TEST("static subsystem registry rejects duplicate ownership bits") {
  robot::StaticSubsystemRegistry<2> registry;
  FakeSubsystem drivetrain(robot::Requirement::kDrivetrain);
  FakeSubsystem duplicate(robot::Requirement::kDrivetrain);
  FakeSubsystem invalid(robot::Requirement::kIntake |
                        robot::Requirement::kLift);
  ROBOT_REQUIRE(registry.registerSubsystem(drivetrain));
  ROBOT_REQUIRE(!registry.registerSubsystem(duplicate));
  ROBOT_REQUIRE(!registry.registerSubsystem(invalid));
  registry.tick(context());
  registry.tick(context(2, 2));
  ROBOT_REQUIRE(drivetrain.periodic_count == 2);
  ROBOT_REQUIRE(drivetrain.boundary_count == 1);
}

ROBOT_TEST("scheduler rejects requirements absent from the subsystem set") {
  robot::StaticScheduler<1> scheduler(robot::Requirement::kDrivetrain);
  FakeCommand lift(3, robot::Requirement::kLift);
  const auto result = scheduler.schedule(
      lift, context(), robot::ConflictPolicy::RejectIncoming);
  ROBOT_REQUIRE(!result.accepted);
  ROBOT_REQUIRE((result.reject_bits & robot::kScheduleUnknownRequirement) !=
                0);
}

ROBOT_TEST("noninterruptible command rejects a conflicting schedule") {
  robot::StaticScheduler<2> scheduler;
  FakeCommand locked(1, robot::Requirement::kDrivetrain, false);
  FakeCommand incoming(2, robot::Requirement::kDrivetrain);
  ROBOT_REQUIRE(scheduler
                    .schedule(locked, context(),
                              robot::ConflictPolicy::RejectIncoming)
                    .accepted);
  const auto result = scheduler.schedule(
      incoming, context(), robot::ConflictPolicy::InterruptExisting);
  ROBOT_REQUIRE(!result.accepted);
  ROBOT_REQUIRE((result.reject_bits & robot::kScheduleNotInterruptible) != 0);
  ROBOT_REQUIRE(locked.end_count == 0);
}

ROBOT_TEST("mode epoch boundary cancels every active command") {
  robot::StaticScheduler<2> scheduler;
  FakeCommand drive(1, robot::Requirement::kDrivetrain);
  const auto scheduled = scheduler.schedule(
      drive, context(), robot::ConflictPolicy::RejectIncoming);
  ROBOT_REQUIRE(scheduled.accepted);
  scheduler.tick(context(2, 2));
  ROBOT_REQUIRE(drive.end_count == 1);
  ROBOT_REQUIRE(drive.last_end_reason == robot::CommandEndReason::ModeBoundary);
  ROBOT_REQUIRE(!scheduler.owns(scheduled.owner));
}

ROBOT_TEST("finished command releases its lease deterministically") {
  robot::StaticScheduler<1> scheduler;
  FakeCommand drive(1, robot::Requirement::kDrivetrain);
  drive.next_state = robot::CommandRunState::Finished;
  const auto scheduled = scheduler.schedule(
      drive, context(), robot::ConflictPolicy::RejectIncoming);
  scheduler.tick(context());
  ROBOT_REQUIRE(drive.execute_count == 1);
  ROBOT_REQUIRE(drive.last_end_reason == robot::CommandEndReason::Completed);
  ROBOT_REQUIRE(!scheduler.owns(scheduled.owner));
}

ROBOT_TEST("request sink preserves a neutral drivetrain ownership request") {
  robot::StaticScheduler<1> scheduler;
  FakeCommand drive(1, robot::Requirement::kDrivetrain);
  const auto scheduled = scheduler.schedule(
      drive, context(), robot::ConflictPolicy::RejectIncoming);
  robot::DriveRequestSink sink;
  sink.beginFrame(context().h);
  const auto request = neutralRequest(context().h, scheduled.owner);
  ROBOT_REQUIRE(sink.publish(request, scheduler));
  robot::DriveRequest read{};
  ROBOT_REQUIRE(sink.read(read));
  const auto* payload =
      std::get_if<robot::DriverCurvaturePayload>(&read.payload);
  ROBOT_REQUIRE(payload != nullptr);
  ROBOT_REQUIRE_NEAR(payload->forward, 0.0, 0.0);
  ROBOT_REQUIRE(scheduler.owns(read.owner));
}

ROBOT_TEST("request sink rejects stale leases and a second writer") {
  robot::StaticScheduler<1> scheduler;
  FakeCommand drive(1, robot::Requirement::kDrivetrain);
  const auto scheduled = scheduler.schedule(
      drive, context(), robot::ConflictPolicy::RejectIncoming);
  robot::DriveRequestSink sink;
  sink.beginFrame(context().h);
  auto request = neutralRequest(context().h, scheduled.owner);
  ROBOT_REQUIRE(sink.publish(request, scheduler));
  ROBOT_REQUIRE(!sink.publish(request, scheduler));
  ROBOT_REQUIRE((sink.rejectBits() & robot::kSinkAlreadyPublished) != 0);

  scheduler.cancel(drive.id(), context());
  sink.beginFrame(context().h);
  ROBOT_REQUIRE(!sink.publish(request, scheduler));
  ROBOT_REQUIRE((sink.rejectBits() & robot::kSinkBadLease) != 0);
}

ROBOT_TEST("arbiter selects a valid neutral request when capability is open") {
  robot::StaticScheduler<1> scheduler;
  FakeCommand drive(1, robot::Requirement::kDrivetrain);
  const auto scheduled = scheduler.schedule(
      drive, context(), robot::ConflictPolicy::RejectIncoming);
  std::array<robot::DriveRequestCandidate, 2> candidates{};
  candidates[0] = {neutralRequest(context().h, scheduled.owner), true};
  robot::DriveCapabilities capabilities{};
  capabilities.driver_curvature = true;
  robot::DriveRequestArbiter arbiter({40000});
  const auto result = arbiter.select(candidates, context().mode, 10000,
                                     capabilities, scheduler);
  ROBOT_REQUIRE(result.has_selection);
  ROBOT_REQUIRE(result.selected.source == robot::RequestSource::Driver);
}

ROBOT_TEST("arbiter enforces offline capability lock and request TTL") {
  robot::StaticScheduler<1> scheduler;
  FakeCommand drive(1, robot::Requirement::kDrivetrain);
  const auto scheduled = scheduler.schedule(
      drive, context(), robot::ConflictPolicy::RejectIncoming);
  std::array<robot::DriveRequestCandidate, 1> candidates{
      robot::DriveRequestCandidate{
          neutralRequest(context().h, scheduled.owner), true}};
  robot::DriveRequestArbiter arbiter({40000});
  auto result = arbiter.select(candidates, context().mode, 10000,
                               robot::DriveCapabilities{}, scheduler);
  ROBOT_REQUIRE(!result.has_selection);
  ROBOT_REQUIRE((result.reject_bits & robot::kArbitrationUnsupported) != 0);

  robot::DriveCapabilities capabilities{};
  capabilities.driver_curvature = true;
  result = arbiter.select(candidates, context().mode, 40001, capabilities,
                          scheduler);
  ROBOT_REQUIRE(!result.has_selection);
  ROBOT_REQUIRE((result.reject_bits & robot::kArbitrationStale) != 0);
}

ROBOT_TEST("safety brake wins arbitration without a command lease") {
  robot::StaticScheduler<1> scheduler;
  FakeCommand drive(1, robot::Requirement::kDrivetrain);
  const auto scheduled = scheduler.schedule(
      drive, context(), robot::ConflictPolicy::RejectIncoming);
  std::array<robot::DriveRequestCandidate, 2> candidates{};
  candidates[0] = {neutralRequest(context().h, scheduled.owner), true};
  robot::DriveRequest safety{};
  safety.h = context().h;
  safety.source = robot::RequestSource::Safety;
  safety.owner.mode_epoch = context().mode.epoch;
  safety.ttl_us = 30000;
  safety.payload = robot::BrakePayload{robot::StopMode::Brake};
  candidates[1] = {safety, true};
  robot::DriveCapabilities capabilities{};
  capabilities.driver_curvature = true;
  robot::DriveRequestArbiter arbiter({40000});
  const auto result = arbiter.select(candidates, context().mode, 10000,
                                     capabilities, scheduler);
  ROBOT_REQUIRE(result.has_selection);
  ROBOT_REQUIRE(result.selected.source == robot::RequestSource::Safety);
  ROBOT_REQUIRE(std::holds_alternative<robot::BrakePayload>(
      result.selected.payload));
}
