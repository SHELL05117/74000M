#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "robot/drive/drive_request.hpp"
#include "robot/runtime/mode.hpp"
#include "robot/state/robot_state.hpp"

namespace robot {

class LeaseAuthority {
 public:
  virtual bool owns(const OwnerToken& token) const noexcept = 0;
  virtual ~LeaseAuthority() = default;
};

enum class CommandRunState : std::uint8_t {
  Running,
  Finished,
  Failed,
};

enum class CommandEndReason : std::uint8_t {
  Completed,
  Failed,
  Interrupted,
  ModeBoundary,
  Cancelled,
};

enum class ConflictPolicy : std::uint8_t {
  RejectIncoming,
  InterruptExisting,
};

struct CommandContext {
  FrameHeader h{};
  ModeSnapshot mode{};
  const RobotState* state{};
  double dt_s{};
};

class Subsystem {
 public:
  virtual RequirementMask requirement() const noexcept = 0;
  virtual void periodic(const CommandContext& context) noexcept = 0;
  virtual void onModeBoundary(const CommandContext& context) noexcept = 0;
  virtual ~Subsystem() = default;
};

template <std::size_t Capacity>
class StaticSubsystemRegistry {
  static_assert(Capacity > 0, "subsystem registry needs capacity");

 public:
  bool registerSubsystem(Subsystem& subsystem) noexcept {
    const RequirementMask requirement = subsystem.requirement();
    if (requirement == 0 || (requirement & (requirement - 1u)) != 0 ||
        size_ >= Capacity || (requirements_ & requirement) != 0) {
      return false;
    }
    subsystems_[size_++] = &subsystem;
    requirements_ |= requirement;
    return true;
  }

  void tick(const CommandContext& context) noexcept {
    if (have_epoch_ && context.mode.epoch != epoch_) {
      for (std::size_t i = 0; i < size_; ++i)
        subsystems_[i]->onModeBoundary(context);
    }
    epoch_ = context.mode.epoch;
    have_epoch_ = true;
    for (std::size_t i = 0; i < size_; ++i)
      subsystems_[i]->periodic(context);
  }

  RequirementMask requirements() const noexcept { return requirements_; }
  std::size_t size() const noexcept { return size_; }

 private:
  std::array<Subsystem*, Capacity> subsystems_{};
  RequirementMask requirements_{};
  std::size_t size_{};
  std::uint32_t epoch_{};
  bool have_epoch_{};
};

class Command {
 public:
  virtual CommandId id() const noexcept = 0;
  virtual RequirementMask requirements() const noexcept = 0;
  virtual bool allowedInMode(CompetitionMode mode) const noexcept = 0;
  virtual bool interruptible() const noexcept { return true; }
  virtual void initialize(const CommandContext& context,
                          const OwnerToken& owner) noexcept = 0;
  virtual CommandRunState execute(const CommandContext& context,
                                  const OwnerToken& owner) noexcept = 0;
  virtual void end(const CommandContext& context, const OwnerToken& owner,
                   CommandEndReason reason) noexcept = 0;
  virtual ~Command() = default;
};

enum ScheduleReject : std::uint32_t {
  kScheduleAccepted = 0,
  kScheduleBadCommand = 1u << 0,
  kScheduleWrongMode = 1u << 1,
  kScheduleDuplicate = 1u << 2,
  kScheduleConflict = 1u << 3,
  kScheduleNotInterruptible = 1u << 4,
  kScheduleFull = 1u << 5,
  kScheduleUnknownRequirement = 1u << 6,
};

struct ScheduleResult {
  OwnerToken owner{};
  std::uint32_t reject_bits{};
  bool accepted{};
};

template <std::size_t Capacity>
class StaticScheduler final : public LeaseAuthority {
  static_assert(Capacity > 0, "scheduler needs at least one slot");

 public:
  explicit StaticScheduler(
      RequirementMask known_requirements =
          static_cast<RequirementMask>(~RequirementMask{0})) noexcept
      : known_requirements_(known_requirements) {}

  ScheduleResult schedule(Command& command, const CommandContext& context,
                          ConflictPolicy policy) noexcept {
    ScheduleResult result{};
    if (command.id() == 0) {
      result.reject_bits |= kScheduleBadCommand;
      return result;
    }
    if ((command.requirements() & ~known_requirements_) != 0) {
      result.reject_bits |= kScheduleUnknownRequirement;
      return result;
    }
    if (!context.mode.enabled || context.h.mode_epoch != context.mode.epoch ||
        !command.allowedInMode(context.mode.mode)) {
      result.reject_bits |= kScheduleWrongMode;
      return result;
    }
    for (const auto& slot : slots_) {
      if (slot.active && slot.command->id() == command.id()) {
        result.reject_bits |= kScheduleDuplicate;
        return result;
      }
    }

    for (const auto& slot : slots_) {
      if (!conflicts(slot, command.requirements())) continue;
      if (policy == ConflictPolicy::RejectIncoming) {
        result.reject_bits |= kScheduleConflict;
        return result;
      }
      if (!slot.command->interruptible()) {
        result.reject_bits |= kScheduleNotInterruptible;
        return result;
      }
    }

    Slot* target = nullptr;
    for (auto& slot : slots_) {
      if (!slot.active) {
        target = &slot;
        break;
      }
    }
    if (target == nullptr) {
      bool conflict_will_free_slot = false;
      for (const auto& slot : slots_)
        if (conflicts(slot, command.requirements()))
          conflict_will_free_slot = true;
      if (!conflict_will_free_slot) {
        result.reject_bits |= kScheduleFull;
        return result;
      }
    }

    for (auto& slot : slots_) {
      if (!conflicts(slot, command.requirements())) continue;
      slot.command->end(context, slot.owner, CommandEndReason::Interrupted);
      slot.active = false;
      slot.command = nullptr;
    }
    if (target == nullptr || target->active) {
      for (auto& slot : slots_) {
        if (!slot.active) {
          target = &slot;
          break;
        }
      }
    }
    if (target == nullptr) {
      result.reject_bits |= kScheduleFull;
      return result;
    }

    target->command = &command;
    target->owner = {command.id(), command.requirements(), nextLease(),
                     context.mode.epoch};
    target->active = true;
    command.initialize(context, target->owner);
    result.owner = target->owner;
    result.accepted = true;
    epoch_ = context.mode.epoch;
    return result;
  }

  void tick(const CommandContext& context) noexcept {
    if (epoch_ != 0 && context.mode.epoch != epoch_) {
      cancelAll(context, CommandEndReason::ModeBoundary);
      epoch_ = context.mode.epoch;
      return;
    }
    epoch_ = context.mode.epoch;
    for (auto& slot : slots_) {
      if (!slot.active) continue;
      if (!context.mode.enabled ||
          !slot.command->allowedInMode(context.mode.mode) ||
          slot.owner.mode_epoch != context.mode.epoch) {
        finishSlot(slot, context, CommandEndReason::ModeBoundary);
        continue;
      }
      const CommandRunState state =
          slot.command->execute(context, slot.owner);
      if (state == CommandRunState::Finished)
        finishSlot(slot, context, CommandEndReason::Completed);
      else if (state == CommandRunState::Failed)
        finishSlot(slot, context, CommandEndReason::Failed);
    }
  }

  bool cancel(CommandId id, const CommandContext& context) noexcept {
    for (auto& slot : slots_) {
      if (slot.active && slot.command->id() == id) {
        finishSlot(slot, context, CommandEndReason::Cancelled);
        return true;
      }
    }
    return false;
  }

  void cancelAll(const CommandContext& context,
                 CommandEndReason reason = CommandEndReason::Cancelled)
      noexcept {
    for (auto& slot : slots_)
      if (slot.active) finishSlot(slot, context, reason);
  }

  bool owns(const OwnerToken& token) const noexcept override {
    if (token.command_id == 0 || token.requirements == 0 ||
        token.lease_generation == 0)
      return false;
    for (const auto& slot : slots_)
      if (slot.active && sameLease(slot.owner, token)) return true;
    return false;
  }

  std::size_t activeCount() const noexcept {
    std::size_t count{};
    for (const auto& slot : slots_)
      if (slot.active) ++count;
    return count;
  }

 private:
  struct Slot {
    Command* command{};
    OwnerToken owner{};
    bool active{};
  };

  static bool conflicts(const Slot& slot,
                        RequirementMask requirements) noexcept {
    return slot.active &&
           (slot.owner.requirements & requirements) != 0;
  }

  void finishSlot(Slot& slot, const CommandContext& context,
                  CommandEndReason reason) noexcept {
    slot.command->end(context, slot.owner, reason);
    slot.command = nullptr;
    slot.owner = {};
    slot.active = false;
  }

  std::uint32_t nextLease() noexcept {
    ++lease_generation_;
    if (lease_generation_ == 0) ++lease_generation_;
    return lease_generation_;
  }

  std::array<Slot, Capacity> slots_{};
  RequirementMask known_requirements_{};
  std::uint32_t lease_generation_{};
  std::uint32_t epoch_{};
};

}  // namespace robot
