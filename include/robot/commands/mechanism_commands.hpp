#pragma once

#include <cmath>
#include <cstdint>

#include "robot/commands/scheduler.hpp"

namespace robot {

using MechanismId = std::uint8_t;

enum class MechanismAction : std::uint8_t {
  Stop,
  SetNormalizedOutput,
  SetPosition,
  SetVelocity,
};

struct MechanismRequest {
  FrameHeader h{};
  OwnerToken owner{};
  MechanismId mechanism_id{};
  MechanismAction action{MechanismAction::Stop};
  double value{};
  TimeUs ttl_us{};
};

class MechanismRequestPublisher {
 public:
  virtual ~MechanismRequestPublisher() = default;
  virtual bool publish(const MechanismRequest& request) noexcept = 0;
};

class InstantMechanismCommand final : public Command {
 public:
  InstantMechanismCommand(CommandId id, RequirementMask requirement,
                          MechanismId mechanism_id, MechanismAction action,
                          double value, TimeUs ttl_us,
                          MechanismRequestPublisher& publisher) noexcept
      : id_(id),
        requirement_(requirement),
        mechanism_id_(mechanism_id),
        action_(action),
        value_(value),
        ttl_us_(ttl_us),
        publisher_(publisher) {}

  CommandId id() const noexcept override { return id_; }
  RequirementMask requirements() const noexcept override {
    return requirement_;
  }
  bool allowedInMode(CompetitionMode mode) const noexcept override {
    return mode == CompetitionMode::AutonomousInterface;
  }
  void initialize(const CommandContext&, const OwnerToken& owner) noexcept override {
    owner_ = owner;
    emitted_ = false;
  }
  CommandRunState execute(const CommandContext& context,
                          const OwnerToken& owner) noexcept override {
    if (emitted_ || !sameLease(owner_, owner) || id_ == 0 ||
        requirement_ == 0 || (owner.requirements & requirement_) == 0 ||
        mechanism_id_ == 0 || ttl_us_ == 0 || !std::isfinite(value_) ||
        (action_ == MechanismAction::SetNormalizedOutput &&
         std::abs(value_) > 1.0))
      return CommandRunState::Failed;
    const MechanismRequest request{context.h, owner_, mechanism_id_, action_,
                                   value_, ttl_us_};
    if (!publisher_.publish(request)) return CommandRunState::Failed;
    emitted_ = true;
    return CommandRunState::Finished;
  }
  void end(const CommandContext&, const OwnerToken&,
           CommandEndReason) noexcept override {}

 private:
  CommandId id_{};
  RequirementMask requirement_{};
  MechanismId mechanism_id_{};
  MechanismAction action_{MechanismAction::Stop};
  double value_{};
  TimeUs ttl_us_{};
  MechanismRequestPublisher& publisher_;
  OwnerToken owner_{};
  bool emitted_{};
};

}  // namespace robot
