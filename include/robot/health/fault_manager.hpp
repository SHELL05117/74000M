#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "robot/core/fault.hpp"
#include "robot/core/frame.hpp"
#include "robot/runtime/mode.hpp"

namespace robot {

enum class FaultSeverity : std::uint8_t {
  Info,
  Warning,
  Derate,
  Stop,
  LatchedStop,
};

enum class SafetyState : std::uint8_t {
  Normal,
  Warning,
  Derated,
  Stopped,
  LatchedStop,
};

struct FaultRuleConfig {
  Fault fault{Fault::None};
  FaultSeverity severity{FaultSeverity::Warning};
  TimeUs enter_delay_us{};
  TimeUs clear_delay_us{};
  double derate{1.0};
  bool latched{};
};

inline bool validFaultRule(const FaultRuleConfig& rule) noexcept {
  const FaultBits bit = faultBit(rule.fault);
  return bit != 0 && (bit & (bit - 1u)) == 0 &&
         std::isfinite(rule.derate) && rule.derate >= 0.0 &&
         rule.derate <= 1.0 &&
         (rule.severity != FaultSeverity::Derate || rule.derate < 1.0) &&
         (rule.severity != FaultSeverity::LatchedStop || rule.latched);
}

struct FaultEvidence {
  FaultBits known_bits{};
  FaultBits asserted_bits{};
  std::uint32_t affected_motor_mask{};
};

struct FaultSummary {
  FaultBits active_bits{};
  FaultBits latched_bits{};
  FaultBits entered_bits{};
  FaultBits exited_bits{};
  std::uint32_t affected_motor_mask{};
  FaultSeverity highest_severity{FaultSeverity::Info};
  SafetyState safety_state{SafetyState::Normal};
  double target_derate{1.0};
  bool stop_required{};
  bool valid{};
};

template <std::size_t Capacity>
class FaultManager {
  static_assert(Capacity > 0, "fault manager needs at least one rule");

 public:
  explicit FaultManager(
      const std::array<FaultRuleConfig, Capacity>& rules) noexcept
      : rules_(rules) {}

  bool valid() const noexcept {
    FaultBits seen{};
    for (const auto& rule : rules_) {
      if (!validFaultRule(rule)) return false;
      const FaultBits bit = faultBit(rule.fault);
      if ((seen & bit) != 0) return false;
      seen |= bit;
    }
    return true;
  }

  FaultSummary update(const FaultEvidence& evidence, TimeUs now_us) noexcept {
    FaultSummary summary{};
    summary.valid = valid() && (!have_time_ || now_us >= last_time_us_);
    if (!summary.valid) return buildSummary({}, 0, 0, false);
    have_time_ = true;
    last_time_us_ = now_us;

    FaultBits entered{};
    FaultBits exited{};
    for (std::size_t i = 0; i < Capacity; ++i) {
      const FaultBits bit = faultBit(rules_[i].fault);
      if ((evidence.known_bits & bit) == 0) continue;
      Runtime& runtime = runtime_[i];
      runtime.raw_asserted = (evidence.asserted_bits & bit) != 0;
      if (runtime.raw_asserted) {
        runtime.clear_timing = false;
        if (!runtime.active) {
          if (!runtime.enter_timing) {
            runtime.enter_timing = true;
            runtime.enter_since_us = now_us;
          }
          if (now_us - runtime.enter_since_us >=
              rules_[i].enter_delay_us) {
            runtime.active = true;
            runtime.enter_timing = false;
            entered |= bit;
            if (rules_[i].latched) latched_bits_ |= bit;
          }
        }
      } else {
        runtime.enter_timing = false;
        if (runtime.active && (latched_bits_ & bit) == 0) {
          if (!runtime.clear_timing) {
            runtime.clear_timing = true;
            runtime.clear_since_us = now_us;
          }
          if (now_us - runtime.clear_since_us >=
              rules_[i].clear_delay_us) {
            runtime.active = false;
            runtime.clear_timing = false;
            exited |= bit;
          }
        } else if (!runtime.active) {
          runtime.clear_timing = false;
        }
      }
    }
    affected_motor_mask_ = evidence.affected_motor_mask;
    return buildSummary(entered, exited, affected_motor_mask_, true);
  }

  FaultBits clearLatched(FaultBits requested, const ModeSnapshot& mode,
                         bool authorized) noexcept {
    if (!authorized || mode.enabled || mode.mode != CompetitionMode::Disabled)
      return 0;
    FaultBits cleared{};
    for (std::size_t i = 0; i < Capacity; ++i) {
      const FaultBits bit = faultBit(rules_[i].fault);
      if ((requested & bit) == 0 || (latched_bits_ & bit) == 0 ||
          runtime_[i].raw_asserted)
        continue;
      latched_bits_ &= ~bit;
      runtime_[i].active = false;
      runtime_[i].enter_timing = false;
      runtime_[i].clear_timing = false;
      cleared |= bit;
    }
    return cleared;
  }

  FaultSummary summary() const noexcept {
    return buildSummary(0, 0, affected_motor_mask_, valid());
  }

 private:
  struct Runtime {
    TimeUs enter_since_us{};
    TimeUs clear_since_us{};
    bool raw_asserted{};
    bool active{};
    bool enter_timing{};
    bool clear_timing{};
  };

  FaultSummary buildSummary(FaultBits entered, FaultBits exited,
                            std::uint32_t affected_motor_mask,
                            bool is_valid) const noexcept {
    FaultSummary summary{};
    summary.valid = is_valid;
    summary.entered_bits = entered;
    summary.exited_bits = exited;
    summary.latched_bits = latched_bits_;
    summary.affected_motor_mask = affected_motor_mask;
    summary.target_derate = 1.0;
    for (std::size_t i = 0; i < Capacity; ++i) {
      if (!runtime_[i].active) continue;
      const FaultBits bit = faultBit(rules_[i].fault);
      summary.active_bits |= bit;
      if (static_cast<std::uint8_t>(rules_[i].severity) >
          static_cast<std::uint8_t>(summary.highest_severity))
        summary.highest_severity = rules_[i].severity;
      if (rules_[i].severity == FaultSeverity::Derate)
        summary.target_derate =
            std::min(summary.target_derate, rules_[i].derate);
      if (rules_[i].severity == FaultSeverity::Stop ||
          rules_[i].severity == FaultSeverity::LatchedStop) {
        summary.stop_required = true;
        summary.target_derate = 0.0;
      }
    }
    switch (summary.highest_severity) {
      case FaultSeverity::Info:
        summary.safety_state = summary.active_bits == 0
                                   ? SafetyState::Normal
                                   : SafetyState::Warning;
        break;
      case FaultSeverity::Warning:
        summary.safety_state = SafetyState::Warning;
        break;
      case FaultSeverity::Derate:
        summary.safety_state = SafetyState::Derated;
        break;
      case FaultSeverity::Stop:
        summary.safety_state = SafetyState::Stopped;
        break;
      case FaultSeverity::LatchedStop:
        summary.safety_state = SafetyState::LatchedStop;
        break;
    }
    return summary;
  }

  std::array<FaultRuleConfig, Capacity> rules_{};
  std::array<Runtime, Capacity> runtime_{};
  FaultBits latched_bits_{};
  std::uint32_t affected_motor_mask_{};
  TimeUs last_time_us_{};
  bool have_time_{};
};

struct DerateRecoveryConfig {
  double recovery_per_s{};
  double max_dt_s{};
};

class DerateController {
 public:
  explicit DerateController(DerateRecoveryConfig config) noexcept
      : config_(config) {}

  bool valid() const noexcept {
    return std::isfinite(config_.recovery_per_s) &&
           config_.recovery_per_s > 0.0 &&
           std::isfinite(config_.max_dt_s) && config_.max_dt_s > 0.0;
  }

  bool update(double target, double dt_s, double& applied) noexcept {
    if (!valid() || !std::isfinite(target) || target < 0.0 || target > 1.0 ||
        !std::isfinite(dt_s) || dt_s < 0.0 || dt_s > config_.max_dt_s) {
      applied = 0.0;
      return false;
    }
    if (target < applied_)
      applied_ = target;
    else
      applied_ = std::min(target, applied_ + config_.recovery_per_s * dt_s);
    applied = applied_;
    return true;
  }

  void reset(double applied = 1.0) noexcept {
    applied_ = std::isfinite(applied)
                   ? std::clamp(applied, 0.0, 1.0)
                   : 0.0;
  }

 private:
  DerateRecoveryConfig config_{};
  double applied_{1.0};
};

}  // namespace robot
