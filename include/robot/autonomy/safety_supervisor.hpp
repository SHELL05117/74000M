#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "robot/drive/drive_request.hpp"
#include "robot/runtime/mode.hpp"
#include "robot/state/robot_state.hpp"
#include "robot/ui/registry_ids.hpp"

namespace robot {

enum class AutonomousOutcome : std::uint8_t {
  Success,
  Timeout,
  Stalled,
  StateInvalid,
  Interrupted,
  Collision,
  Deviation,
  MechanismConflict,
};

enum class FailureAction : std::uint8_t {
  Retry,
  Skip,
  EndRoute,
  EmergencyStop,
};

struct FailurePolicyEntry {
  AutonomousOutcome outcome{AutonomousOutcome::StateInvalid};
  FailureAction action{FailureAction::EndRoute};
  std::uint8_t max_retries{};
  FailureAction exhausted_action{FailureAction::EndRoute};
};

template <std::size_t Capacity>
class FixedFailurePolicy {
 public:
  explicit FixedFailurePolicy(
      const std::array<FailurePolicyEntry, Capacity>& entries) noexcept
      : entries_(entries) {}

  bool valid() const noexcept {
    for (std::size_t i = 0; i < Capacity; ++i) {
      if (entries_[i].outcome == AutonomousOutcome::Success ||
          (entries_[i].action == FailureAction::Retry &&
           (entries_[i].max_retries == 0 ||
            entries_[i].exhausted_action == FailureAction::Retry)))
        return false;
      for (std::size_t j = i + 1; j < Capacity; ++j)
        if (entries_[i].outcome == entries_[j].outcome) return false;
    }
    return true;
  }

  const FailurePolicyEntry* find(AutonomousOutcome outcome) const noexcept {
    for (const auto& entry : entries_)
      if (entry.outcome == outcome) return &entry;
    return nullptr;
  }

 private:
  std::array<FailurePolicyEntry, Capacity> entries_{};
};

enum class DegradedAutonomyPolicy : std::uint8_t {
  ContinueScaled,
  AbortAfterGrace,
};

struct AutonomySafetyConfig {
  double degraded_speed_scale{};
  double max_deviation_m{};
  TimeUs degraded_grace_us{};
  TimeUs max_state_age_us{};
  TimeUs safety_request_ttl_us{};
  DegradedAutonomyPolicy degraded_policy{
      DegradedAutonomyPolicy::ContinueScaled};
};

inline bool validAutonomySafetyConfig(
    const AutonomySafetyConfig& config) noexcept {
  return std::isfinite(config.degraded_speed_scale) &&
         config.degraded_speed_scale > 0.0 &&
         config.degraded_speed_scale <= 1.0 &&
         std::isfinite(config.max_deviation_m) &&
         config.max_deviation_m > 0.0 && config.degraded_grace_us > 0 &&
         config.max_state_age_us > 0 && config.safety_request_ttl_us > 0;
}

enum class AutonomyDecisionAction : std::uint8_t {
  Continue,
  ContinueDegraded,
  RetryRequired,
  SkipCommand,
  EndRoute,
  EmergencyStop,
};

struct AutonomyFailureRecord {
  FrameHeader h{};
  RouteId route_id{RouteIds::kDoNothing};
  CommandId command_id{};
  std::uint32_t lease_generation{};
  std::uint32_t attempt{};
  AutonomousOutcome outcome{AutonomousOutcome::Success};
  AutonomyDecisionAction decision{AutonomyDecisionAction::Continue};
  bool valid{};
};

struct AutonomySafetyInput {
  FrameHeader h{};
  ModeSnapshot mode{};
  const RobotState* state{};
  OwnerToken owner{};
  Quality slip_quality{Quality::Good};
  double deviation_m{};
  std::uint32_t mechanism_conflict_bits{};
  bool collision_detected{};
  bool command_terminal{};
  AutonomousOutcome command_outcome{AutonomousOutcome::Success};
};

struct AutonomySafetyDecision {
  DriveRequest safety_request{};
  AutonomyFailureRecord failure{};
  AutonomyDecisionAction action{AutonomyDecisionAction::Continue};
  AutonomousOutcome outcome{AutonomousOutcome::Success};
  double speed_scale{1.0};
  bool has_safety_request{};
  bool latched{};
};

template <std::size_t PolicyCapacity>
class AutonomySafetySupervisor {
 public:
  AutonomySafetySupervisor(AutonomySafetyConfig config,
                           const FixedFailurePolicy<PolicyCapacity>& policy)
      noexcept
      : config_(config), policy_(policy) {}

  bool valid() const noexcept {
    return validAutonomySafetyConfig(config_) && policy_.valid();
  }

  bool startRoute(RouteId route_id, const ModeSnapshot& mode,
                  TimeUs now_us) noexcept {
    reset();
    if (!valid() || !mode.enabled ||
        mode.mode != CompetitionMode::AutonomousInterface)
      return false;
    route_id_ = route_id;
    epoch_ = mode.epoch;
    route_start_us_ = now_us;
    route_active_ = true;
    return true;
  }

  bool beginCommand(const OwnerToken& owner,
                    const ModeSnapshot& mode) noexcept {
    if (!route_active_ || latched_ || active_command_ ||
        !validOwner(owner, mode) ||
        owner.lease_generation == last_lease_generation_)
      return false;
    active_owner_ = owner;
    failed_owner_ = {};
    last_lease_generation_ = owner.lease_generation;
    retry_count_ = 0;
    attempt_ = 1;
    active_command_ = true;
    degraded_timing_ = false;
    return true;
  }

  AutonomySafetyDecision evaluate(
      const AutonomySafetyInput& input) noexcept {
    if (!route_active_)
      return transientStop(input.h, AutonomousOutcome::StateInvalid);
    if (latched_) return latchedDecision(input.h);
    if (!active_command_)
      return transientStop(input.h, AutonomousOutcome::StateInvalid);
    if (!sameLease(input.owner, active_owner_))
      return transientStop(input.h, AutonomousOutcome::Interrupted);

    if (!input.mode.enabled ||
        input.mode.mode != CompetitionMode::AutonomousInterface ||
        input.mode.epoch != epoch_ || input.h.mode_epoch != epoch_ ||
        input.owner.mode_epoch != epoch_)
      return latchFailure(input.h, AutonomousOutcome::Interrupted);
    if (input.collision_detected)
      return latchFailure(input.h, AutonomousOutcome::Collision);
    if (input.mechanism_conflict_bits != 0)
      return latchFailure(input.h, AutonomousOutcome::MechanismConflict);
    if (input.state == nullptr || input.state->h.mode_epoch != epoch_ ||
        input.state->h.time_us > input.h.time_us ||
        input.h.time_us - input.state->h.time_us > config_.max_state_age_us ||
        input.state->translation_quality == Quality::Invalid ||
        input.state->heading_quality == Quality::Invalid ||
        input.slip_quality == Quality::Invalid ||
        !std::isfinite(input.deviation_m))
      return latchFailure(input.h, AutonomousOutcome::StateInvalid);
    if (input.deviation_m > config_.max_deviation_m)
      return latchFailure(input.h, AutonomousOutcome::Deviation);
    if (input.command_terminal &&
        input.command_outcome != AutonomousOutcome::Success)
      return latchFailure(input.h, input.command_outcome);
    if (input.command_terminal &&
        input.command_outcome == AutonomousOutcome::Success) {
      active_command_ = false;
      degraded_timing_ = false;
      return {};
    }

    const bool degraded =
        input.state->translation_quality == Quality::Degraded ||
        input.state->heading_quality == Quality::Degraded ||
        input.slip_quality == Quality::Degraded;
    if (!degraded) {
      degraded_timing_ = false;
      return {};
    }
    if (!degraded_timing_) {
      degraded_timing_ = true;
      degraded_start_us_ = input.h.time_us;
    }
    if (config_.degraded_policy ==
            DegradedAutonomyPolicy::AbortAfterGrace &&
        input.h.time_us - degraded_start_us_ >= config_.degraded_grace_us)
      return latchFailure(input.h, AutonomousOutcome::StateInvalid);
    AutonomySafetyDecision decision{};
    decision.action = AutonomyDecisionAction::ContinueDegraded;
    decision.speed_scale = config_.degraded_speed_scale;
    return decision;
  }

  bool authorizeRetry(const OwnerToken& new_owner,
                      const ModeSnapshot& mode,
                      bool brake_barrier_acknowledged,
                      bool failure_condition_absent) noexcept {
    if (!latched_ || latched_action_ != AutonomyDecisionAction::RetryRequired ||
        !brake_barrier_acknowledged || !failure_condition_absent ||
        !validOwner(new_owner, mode) ||
        new_owner.command_id != failed_owner_.command_id ||
        new_owner.requirements != failed_owner_.requirements ||
        new_owner.lease_generation == failed_owner_.lease_generation ||
        new_owner.lease_generation == last_lease_generation_)
      return false;
    ++retry_count_;
    ++attempt_;
    active_owner_ = new_owner;
    last_lease_generation_ = new_owner.lease_generation;
    active_command_ = true;
    clearLatch();
    return true;
  }

  bool advanceAfterSkip(const OwnerToken& next_owner,
                        const ModeSnapshot& mode,
                        bool brake_barrier_acknowledged) noexcept {
    if (!latched_ || latched_action_ != AutonomyDecisionAction::SkipCommand ||
        !brake_barrier_acknowledged || !validOwner(next_owner, mode) ||
        next_owner.lease_generation == last_lease_generation_)
      return false;
    active_owner_ = next_owner;
    last_lease_generation_ = next_owner.lease_generation;
    retry_count_ = 0;
    attempt_ = 1;
    active_command_ = true;
    clearLatch();
    return true;
  }

  bool filterChassisRequest(const AutonomySafetyDecision& decision,
                            const DriveRequest& candidate,
                            DriveRequest& filtered) const noexcept {
    if ((decision.action != AutonomyDecisionAction::Continue &&
         decision.action != AutonomyDecisionAction::ContinueDegraded) ||
        !active_command_ || !sameLease(candidate.owner, active_owner_) ||
        candidate.h.mode_epoch != epoch_ ||
        candidate.source != RequestSource::FutureAutonomy)
      return false;
    const auto* velocity =
        std::get_if<ChassisVelocityPayload>(&candidate.payload);
    if (velocity == nullptr || !finitePayload(candidate.payload)) return false;
    filtered = candidate;
    filtered.payload = ChassisVelocityPayload{
        velocity->vx_mps * decision.speed_scale,
        velocity->omega_radps * decision.speed_scale};
    return true;
  }

  void reset() noexcept {
    route_id_ = RouteIds::kDoNothing;
    epoch_ = 0;
    route_start_us_ = 0;
    degraded_start_us_ = 0;
    failure_time_us_ = 0;
    active_owner_ = {};
    failed_owner_ = {};
    failure_record_ = {};
    last_lease_generation_ = 0;
    retry_count_ = 0;
    attempt_ = 0;
    latched_outcome_ = AutonomousOutcome::Success;
    latched_action_ = AutonomyDecisionAction::Continue;
    route_active_ = false;
    active_command_ = false;
    degraded_timing_ = false;
    latched_ = false;
  }

  const OwnerToken& activeOwner() const noexcept { return active_owner_; }
  std::uint32_t attempt() const noexcept { return attempt_; }
  bool latched() const noexcept { return latched_; }

 private:
  bool validOwner(const OwnerToken& owner,
                  const ModeSnapshot& mode) const noexcept {
    return mode.enabled && mode.mode == CompetitionMode::AutonomousInterface &&
           mode.epoch == epoch_ && owner.command_id != 0 &&
           owner.requirements != 0 && owner.lease_generation != 0 &&
           owner.mode_epoch == epoch_;
  }

  AutonomySafetyDecision latchFailure(const FrameHeader& header,
                                      AutonomousOutcome outcome) noexcept {
    const FailurePolicyEntry* entry = policy_.find(outcome);
    FailureAction action =
        entry == nullptr ? FailureAction::EmergencyStop : entry->action;
    if (entry != nullptr && action == FailureAction::Retry &&
        retry_count_ >= entry->max_retries)
      action = entry->exhausted_action;
    latched_outcome_ = outcome;
    latched_action_ = mapAction(action);
    failed_owner_ = active_owner_;
    failure_time_us_ = header.time_us;
    latched_ = true;
    active_command_ = false;
    failure_record_ = {header,
                       route_id_,
                       failed_owner_.command_id,
                       failed_owner_.lease_generation,
                       attempt_,
                       outcome,
                       latched_action_,
                       true};
    return latchedDecision(header);
  }

  AutonomySafetyDecision latchedDecision(
      const FrameHeader& header) const noexcept {
    AutonomySafetyDecision decision{};
    decision.action = latched_action_;
    decision.outcome = latched_outcome_;
    decision.speed_scale = 0.0;
    decision.has_safety_request = true;
    decision.latched = true;
    decision.failure = failure_record_;
    decision.safety_request = safetyBrake(header);
    return decision;
  }

  AutonomySafetyDecision transientStop(const FrameHeader& header,
                                       AutonomousOutcome outcome) const
      noexcept {
    AutonomySafetyDecision decision{};
    decision.action = AutonomyDecisionAction::EmergencyStop;
    decision.outcome = outcome;
    decision.speed_scale = 0.0;
    decision.has_safety_request = true;
    decision.safety_request = safetyBrake(header);
    return decision;
  }

  DriveRequest safetyBrake(const FrameHeader& header) const noexcept {
    DriveRequest request{};
    request.h = header;
    request.source = RequestSource::Safety;
    request.owner.mode_epoch = header.mode_epoch;
    request.ttl_us = config_.safety_request_ttl_us;
    request.payload = BrakePayload{StopMode::Brake};
    return request;
  }

  static AutonomyDecisionAction mapAction(FailureAction action) noexcept {
    switch (action) {
      case FailureAction::Retry:
        return AutonomyDecisionAction::RetryRequired;
      case FailureAction::Skip:
        return AutonomyDecisionAction::SkipCommand;
      case FailureAction::EndRoute:
        return AutonomyDecisionAction::EndRoute;
      case FailureAction::EmergencyStop:
        return AutonomyDecisionAction::EmergencyStop;
    }
    return AutonomyDecisionAction::EmergencyStop;
  }

  void clearLatch() noexcept {
    latched_ = false;
    latched_outcome_ = AutonomousOutcome::Success;
    latched_action_ = AutonomyDecisionAction::Continue;
    failure_record_ = {};
    failure_time_us_ = 0;
    degraded_timing_ = false;
  }

  AutonomySafetyConfig config_{};
  const FixedFailurePolicy<PolicyCapacity>& policy_;
  RouteId route_id_{RouteIds::kDoNothing};
  std::uint32_t epoch_{};
  TimeUs route_start_us_{};
  TimeUs degraded_start_us_{};
  TimeUs failure_time_us_{};
  OwnerToken active_owner_{};
  OwnerToken failed_owner_{};
  AutonomyFailureRecord failure_record_{};
  std::uint32_t last_lease_generation_{};
  std::uint32_t retry_count_{};
  std::uint32_t attempt_{};
  AutonomousOutcome latched_outcome_{AutonomousOutcome::Success};
  AutonomyDecisionAction latched_action_{AutonomyDecisionAction::Continue};
  bool route_active_{};
  bool active_command_{};
  bool degraded_timing_{};
  bool latched_{};
};

}  // namespace robot
