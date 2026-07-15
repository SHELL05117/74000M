#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "robot/commands/scheduler.hpp"

namespace robot {

template <std::size_t Capacity>
class FixedCommandGroupBase : public Command {
  static_assert(Capacity > 0, "command group needs capacity");

 public:
  FixedCommandGroupBase(CommandId id,
                        const std::array<Command*, Capacity>& commands,
                        std::size_t count) noexcept
      : id_(id), count_(count) {
    if (id_ == 0 || count_ == 0 || count_ > Capacity) return;
    valid_ = true;
    for (std::size_t i = 0; i < count_; ++i) {
      if (commands[i] == nullptr || commands[i]->id() == 0) {
        valid_ = false;
        return;
      }
      for (std::size_t j = 0; j < i; ++j)
        if (commands[j]->id() == commands[i]->id()) valid_ = false;
      slots_[i].command = commands[i];
      requirements_ |= commands[i]->requirements();
    }
  }

  CommandId id() const noexcept override { return id_; }
  RequirementMask requirements() const noexcept override {
    return requirements_;
  }
  bool allowedInMode(CompetitionMode mode) const noexcept override {
    if (!valid_) return false;
    for (std::size_t i = 0; i < count_; ++i)
      if (!slots_[i].command->allowedInMode(mode)) return false;
    return true;
  }
  bool interruptible() const noexcept override {
    for (std::size_t i = 0; i < count_; ++i)
      if (!slots_[i].command->interruptible()) return false;
    return true;
  }

  void end(const CommandContext& context, const OwnerToken& owner,
           CommandEndReason reason) noexcept override {
    if (sameLease(owner_, owner)) cancelRunning(context, reason);
  }

  bool valid() const noexcept { return valid_; }

 protected:
  struct Slot {
    Command* command{};
    bool started{};
    bool finished{};
  };

  void beginGroup(const OwnerToken& owner) noexcept {
    owner_ = owner;
    for (std::size_t i = 0; i < count_; ++i) {
      slots_[i].started = false;
      slots_[i].finished = false;
    }
  }

  void startChild(std::size_t index,
                  const CommandContext& context) noexcept {
    slots_[index].command->initialize(context, owner_);
    slots_[index].started = true;
  }

  CommandRunState tickChild(std::size_t index,
                            const CommandContext& context) noexcept {
    return slots_[index].command->execute(context, owner_);
  }

  void finishChild(std::size_t index, const CommandContext& context,
                   CommandEndReason reason) noexcept {
    if (!slots_[index].started || slots_[index].finished) return;
    slots_[index].command->end(context, owner_, reason);
    slots_[index].finished = true;
  }

  void cancelRunning(const CommandContext& context,
                     CommandEndReason reason) noexcept {
    for (std::size_t i = 0; i < count_; ++i)
      finishChild(i, context, reason);
  }

  bool parallelRequirementsDisjoint() const noexcept {
    RequirementMask claimed{};
    for (std::size_t i = 0; i < count_; ++i) {
      const RequirementMask next = slots_[i].command->requirements();
      if ((claimed & next) != 0) return false;
      claimed |= next;
    }
    return true;
  }

  std::array<Slot, Capacity> slots_{};
  std::size_t count_{};
  OwnerToken owner_{};
  bool valid_{};

 private:
  CommandId id_{};
  RequirementMask requirements_{};
};

template <std::size_t Capacity>
class SequentialCommandGroup final : public FixedCommandGroupBase<Capacity> {
  using Base = FixedCommandGroupBase<Capacity>;

 public:
  SequentialCommandGroup(CommandId id,
                         const std::array<Command*, Capacity>& commands,
                         std::size_t count) noexcept
      : Base(id, commands, count) {}

  void initialize(const CommandContext& context,
                  const OwnerToken& owner) noexcept override {
    this->beginGroup(owner);
    current_ = 0;
    if (this->valid()) this->startChild(0, context);
  }

  CommandRunState execute(const CommandContext& context,
                          const OwnerToken& owner) noexcept override {
    if (!this->valid() || !sameLease(this->owner_, owner))
      return CommandRunState::Failed;
    const auto state = this->tickChild(current_, context);
    if (state == CommandRunState::Running) return state;
    if (state == CommandRunState::Failed) {
      this->finishChild(current_, context, CommandEndReason::Failed);
      return CommandRunState::Failed;
    }
    this->finishChild(current_, context, CommandEndReason::Completed);
    ++current_;
    if (current_ == this->count_) return CommandRunState::Finished;
    this->startChild(current_, context);
    return CommandRunState::Running;
  }

 private:
  std::size_t current_{};
};

template <std::size_t Capacity>
class ParallelCommandGroup final : public FixedCommandGroupBase<Capacity> {
  using Base = FixedCommandGroupBase<Capacity>;

 public:
  ParallelCommandGroup(CommandId id,
                       const std::array<Command*, Capacity>& commands,
                       std::size_t count) noexcept
      : Base(id, commands, count) {}

  void initialize(const CommandContext& context,
                  const OwnerToken& owner) noexcept override {
    this->beginGroup(owner);
    parallel_valid_ = this->valid() && this->parallelRequirementsDisjoint();
    if (!parallel_valid_) return;
    for (std::size_t i = 0; i < this->count_; ++i)
      this->startChild(i, context);
  }

  CommandRunState execute(const CommandContext& context,
                          const OwnerToken& owner) noexcept override {
    if (!parallel_valid_ || !sameLease(this->owner_, owner))
      return CommandRunState::Failed;
    bool all_finished = true;
    for (std::size_t i = 0; i < this->count_; ++i) {
      if (this->slots_[i].finished) continue;
      const auto state = this->tickChild(i, context);
      if (state == CommandRunState::Failed) {
        this->finishChild(i, context, CommandEndReason::Failed);
        this->cancelRunning(context, CommandEndReason::Interrupted);
        return CommandRunState::Failed;
      }
      if (state == CommandRunState::Finished)
        this->finishChild(i, context, CommandEndReason::Completed);
      else
        all_finished = false;
    }
    return all_finished ? CommandRunState::Finished
                        : CommandRunState::Running;
  }

 private:
  bool parallel_valid_{};
};

template <std::size_t Capacity>
class RaceCommandGroup final : public FixedCommandGroupBase<Capacity> {
  using Base = FixedCommandGroupBase<Capacity>;

 public:
  RaceCommandGroup(CommandId id,
                   const std::array<Command*, Capacity>& commands,
                   std::size_t count) noexcept
      : Base(id, commands, count) {}

  void initialize(const CommandContext& context,
                  const OwnerToken& owner) noexcept override {
    this->beginGroup(owner);
    parallel_valid_ = this->valid() && this->parallelRequirementsDisjoint();
    if (!parallel_valid_) return;
    for (std::size_t i = 0; i < this->count_; ++i)
      this->startChild(i, context);
  }

  CommandRunState execute(const CommandContext& context,
                          const OwnerToken& owner) noexcept override {
    if (!parallel_valid_ || !sameLease(this->owner_, owner))
      return CommandRunState::Failed;
    for (std::size_t i = 0; i < this->count_; ++i) {
      const auto state = this->tickChild(i, context);
      if (state == CommandRunState::Running) continue;
      this->finishChild(i, context,
                        state == CommandRunState::Finished
                            ? CommandEndReason::Completed
                            : CommandEndReason::Failed);
      this->cancelRunning(context, CommandEndReason::Interrupted);
      return state;
    }
    return CommandRunState::Running;
  }

 private:
  bool parallel_valid_{};
};

template <std::size_t Capacity>
class DeadlineCommandGroup final : public FixedCommandGroupBase<Capacity> {
  using Base = FixedCommandGroupBase<Capacity>;

 public:
  DeadlineCommandGroup(CommandId id,
                       const std::array<Command*, Capacity>& commands,
                       std::size_t count, std::size_t deadline_index) noexcept
      : Base(id, commands, count), deadline_index_(deadline_index) {}

  void initialize(const CommandContext& context,
                  const OwnerToken& owner) noexcept override {
    this->beginGroup(owner);
    parallel_valid_ = this->valid() && deadline_index_ < this->count_ &&
                      this->parallelRequirementsDisjoint();
    if (!parallel_valid_) return;
    for (std::size_t i = 0; i < this->count_; ++i)
      this->startChild(i, context);
  }

  CommandRunState execute(const CommandContext& context,
                          const OwnerToken& owner) noexcept override {
    if (!parallel_valid_ || !sameLease(this->owner_, owner))
      return CommandRunState::Failed;
    for (std::size_t i = 0; i < this->count_; ++i) {
      if (this->slots_[i].finished) continue;
      const auto state = this->tickChild(i, context);
      if (state == CommandRunState::Failed) {
        this->finishChild(i, context, CommandEndReason::Failed);
        this->cancelRunning(context, CommandEndReason::Interrupted);
        return CommandRunState::Failed;
      }
      if (state != CommandRunState::Finished) continue;
      this->finishChild(i, context, CommandEndReason::Completed);
      if (i == deadline_index_) {
        this->cancelRunning(context, CommandEndReason::Interrupted);
        return CommandRunState::Finished;
      }
    }
    return CommandRunState::Running;
  }

 private:
  std::size_t deadline_index_{};
  bool parallel_valid_{};
};

class WaitCommand final : public Command {
 public:
  WaitCommand(CommandId id, TimeUs duration_us) noexcept
      : id_(id), duration_us_(duration_us) {}
  CommandId id() const noexcept override { return id_; }
  RequirementMask requirements() const noexcept override { return 0; }
  bool allowedInMode(CompetitionMode mode) const noexcept override {
    return mode == CompetitionMode::AutonomousInterface;
  }
  void initialize(const CommandContext& context,
                  const OwnerToken&) noexcept override {
    start_us_ = context.h.time_us;
    initialized_ = true;
  }
  CommandRunState execute(const CommandContext& context,
                          const OwnerToken&) noexcept override {
    if (!initialized_ || context.h.time_us < start_us_)
      return CommandRunState::Failed;
    return context.h.time_us - start_us_ >= duration_us_
               ? CommandRunState::Finished
               : CommandRunState::Running;
  }
  void end(const CommandContext&, const OwnerToken&,
           CommandEndReason) noexcept override {
    initialized_ = false;
  }

 private:
  CommandId id_{};
  TimeUs duration_us_{};
  TimeUs start_us_{};
  bool initialized_{};
};

class TimeoutCommand final : public Command {
 public:
  TimeoutCommand(CommandId id, Command& child, TimeUs timeout_us) noexcept
      : id_(id), child_(child), timeout_us_(timeout_us) {}
  CommandId id() const noexcept override { return id_; }
  RequirementMask requirements() const noexcept override {
    return child_.requirements();
  }
  bool allowedInMode(CompetitionMode mode) const noexcept override {
    return child_.allowedInMode(mode);
  }
  bool interruptible() const noexcept override {
    return child_.interruptible();
  }
  void initialize(const CommandContext& context,
                  const OwnerToken& owner) noexcept override {
    owner_ = owner;
    start_us_ = context.h.time_us;
    timed_out_ = false;
    active_ = timeout_us_ > 0;
    if (active_) child_.initialize(context, owner_);
  }
  CommandRunState execute(const CommandContext& context,
                          const OwnerToken& owner) noexcept override {
    if (!active_ || !sameLease(owner_, owner) ||
        context.h.time_us < start_us_)
      return CommandRunState::Failed;
    if (context.h.time_us - start_us_ >= timeout_us_) {
      child_.end(context, owner_, CommandEndReason::Failed);
      active_ = false;
      timed_out_ = true;
      return CommandRunState::Failed;
    }
    const auto state = child_.execute(context, owner_);
    if (state != CommandRunState::Running) {
      child_.end(context, owner_, state == CommandRunState::Finished
                                      ? CommandEndReason::Completed
                                      : CommandEndReason::Failed);
      active_ = false;
    }
    return state;
  }
  void end(const CommandContext& context, const OwnerToken& owner,
           CommandEndReason reason) noexcept override {
    if (active_ && sameLease(owner_, owner)) child_.end(context, owner_, reason);
    active_ = false;
  }
  bool timedOut() const noexcept { return timed_out_; }

 private:
  CommandId id_{};
  Command& child_;
  TimeUs timeout_us_{};
  TimeUs start_us_{};
  OwnerToken owner_{};
  bool active_{};
  bool timed_out_{};
};

using CommandCondition = bool (*)(const CommandContext&) noexcept;

class ConditionalCommand final : public Command {
 public:
  ConditionalCommand(CommandId id, Command& when_true, Command& when_false,
                     CommandCondition condition) noexcept
      : id_(id),
        when_true_(when_true),
        when_false_(when_false),
        condition_(condition) {}
  CommandId id() const noexcept override { return id_; }
  RequirementMask requirements() const noexcept override {
    return when_true_.requirements() | when_false_.requirements();
  }
  bool allowedInMode(CompetitionMode mode) const noexcept override {
    return when_true_.allowedInMode(mode) && when_false_.allowedInMode(mode);
  }
  void initialize(const CommandContext& context,
                  const OwnerToken& owner) noexcept override {
    owner_ = owner;
    selected_ = condition_ != nullptr && condition_(context) ? &when_true_
                                                             : &when_false_;
    active_ = selected_ != nullptr;
    if (active_) selected_->initialize(context, owner_);
  }
  CommandRunState execute(const CommandContext& context,
                          const OwnerToken& owner) noexcept override {
    if (!active_ || !sameLease(owner_, owner)) return CommandRunState::Failed;
    const auto state = selected_->execute(context, owner_);
    if (state != CommandRunState::Running) {
      selected_->end(context, owner_, state == CommandRunState::Finished
                                          ? CommandEndReason::Completed
                                          : CommandEndReason::Failed);
      active_ = false;
    }
    return state;
  }
  void end(const CommandContext& context, const OwnerToken& owner,
           CommandEndReason reason) noexcept override {
    if (active_ && sameLease(owner_, owner)) selected_->end(context, owner_, reason);
    active_ = false;
  }

 private:
  CommandId id_{};
  Command& when_true_;
  Command& when_false_;
  CommandCondition condition_{};
  Command* selected_{};
  OwnerToken owner_{};
  bool active_{};
};

}  // namespace robot
