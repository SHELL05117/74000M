#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "robot/hmi/edit_lease.hpp"
#include "robot/hmi/permissions.hpp"
#include "robot/hmi/registry.hpp"

namespace robot {

struct ParameterValue {
  ParameterId id{ParameterIds::kInvalid};
  double value{};
};

class ParameterBackend {
 public:
  virtual ~ParameterBackend() = default;
  virtual bool read(ParameterId id, double& value) const noexcept = 0;
  virtual bool validate(ParameterId id, double value) const noexcept = 0;
  virtual bool apply(const ParameterValue* values, std::size_t count,
                     ApplyPolicy strongest_policy) noexcept = 0;
  virtual std::uint32_t revision() const noexcept = 0;
};

enum ParameterApplyReject : std::uint32_t {
  kParameterApplyAccepted = 0,
  kParameterApplyNoTransaction = 1u << 0,
  kParameterApplyPermission = 1u << 1,
  kParameterApplyLease = 1u << 2,
  kParameterApplyEpoch = 1u << 3,
  kParameterApplyExpired = 1u << 4,
  kParameterApplyRevision = 1u << 5,
  kParameterApplyValidation = 1u << 6,
  kParameterApplyBackend = 1u << 7,
};

struct ParameterApplyResult {
  bool applied{};
  std::uint32_t reject_bits{kParameterApplyNoTransaction};
  ApplyPolicy strongest_policy{ApplyPolicy::ImmediateWhenDisabled};
  std::uint32_t new_revision{};
};

struct ParameterTransactionContext {
  HmiPermissionContext permission{};
  TimeUs now_us{};
};

template <std::size_t RegistryCapacity, std::size_t MaxChanges>
class ParameterTransaction {
  static_assert(MaxChanges > 0, "parameter transaction needs capacity");

 public:
  ParameterTransaction(const ParameterRegistry<RegistryCapacity>& registry,
                       ParameterBackend& backend) noexcept
      : registry_(registry), backend_(backend) {}

  bool begin(const EditLease& lease, TimeUs now_us, TimeUs ttl_us,
             std::uint32_t mode_epoch) noexcept {
    if (!lease.active || lease.domain != EditDomain::Parameters || ttl_us == 0 ||
        lease.mode_epoch != mode_epoch || now_us > lease.expires_us)
      return false;
    clear();
    active_ = true;
    lease_ = lease;
    expires_us_ = now_us + ttl_us;
    base_revision_ = backend_.revision();
    return true;
  }

  bool stage(ParameterId id, std::int32_t delta_steps, bool coarse) noexcept {
    if (!active_ || delta_steps == 0) return false;
    const auto* descriptor = registry_.find(id);
    if (descriptor == nullptr || !descriptor->available ||
        descriptor->access != ParameterAccess::BenchEditable)
      return false;

    std::size_t index = find(id);
    if (index == size_) {
      if (size_ == MaxChanges) return false;
      double current{};
      if (!backend_.read(id, current) || !std::isfinite(current)) return false;
      changes_[size_] = {id, current, current, descriptor->apply_policy};
      index = size_++;
    }
    const double step = coarse ? descriptor->coarse_step : descriptor->fine_step;
    const double next = std::clamp(
        changes_[index].staged + static_cast<double>(delta_steps) * step,
        descriptor->min_value, descriptor->max_value);
    if (!std::isfinite(next) || !backend_.validate(id, next)) return false;
    changes_[index].staged = next;
    return true;
  }

  ParameterApplyResult apply(const ParameterTransactionContext& context,
                             const EditLeaseManager& leases) noexcept {
    ParameterApplyResult result{};
    result.new_revision = backend_.revision();
    if (!active_ || size_ == 0) return result;
    result.reject_bits = kParameterApplyAccepted;
    if (!hmiPermissionAllows(HmiAction::ApplyParameterTransaction,
                             context.permission))
      result.reject_bits |= kParameterApplyPermission;
    if (!leases.owns(lease_, context.now_us, context.permission.mode.epoch))
      result.reject_bits |= kParameterApplyLease;
    if (lease_.mode_epoch != context.permission.mode.epoch)
      result.reject_bits |= kParameterApplyEpoch;
    if (context.now_us > expires_us_)
      result.reject_bits |= kParameterApplyExpired;
    if (backend_.revision() != base_revision_)
      result.reject_bits |= kParameterApplyRevision;

    std::array<ParameterValue, MaxChanges> values{};
    ApplyPolicy strongest = ApplyPolicy::ImmediateWhenDisabled;
    for (std::size_t i = 0; i < size_; ++i) {
      values[i] = {changes_[i].id, changes_[i].staged};
      if (!backend_.validate(values[i].id, values[i].value))
        result.reject_bits |= kParameterApplyValidation;
      if (static_cast<std::uint8_t>(changes_[i].policy) >
          static_cast<std::uint8_t>(strongest))
        strongest = changes_[i].policy;
    }
    result.strongest_policy = strongest;
    if (result.reject_bits != kParameterApplyAccepted) return result;
    if (!backend_.apply(values.data(), size_, strongest)) {
      result.reject_bits = kParameterApplyBackend;
      return result;
    }
    result.applied = true;
    result.new_revision = backend_.revision();
    clear();
    return result;
  }

  void rollback() noexcept { clear(); }
  void onModeEpochChanged() noexcept { clear(); }
  void onOwnerDisconnected(HmiOrigin owner) noexcept {
    if (active_ && lease_.owner == owner) clear();
  }

  bool active() const noexcept { return active_; }
  std::size_t size() const noexcept { return size_; }
  bool stagedValue(ParameterId id, double& value) const noexcept {
    const std::size_t index = find(id);
    if (index == size_) return false;
    value = changes_[index].staged;
    return true;
  }

 private:
  struct Change {
    ParameterId id{ParameterIds::kInvalid};
    double original{};
    double staged{};
    ApplyPolicy policy{ApplyPolicy::ImmediateWhenDisabled};
  };

  std::size_t find(ParameterId id) const noexcept {
    for (std::size_t i = 0; i < size_; ++i)
      if (changes_[i].id == id) return i;
    return size_;
  }

  void clear() noexcept {
    changes_ = {};
    size_ = 0;
    lease_ = {};
    expires_us_ = 0;
    base_revision_ = 0;
    active_ = false;
  }

  const ParameterRegistry<RegistryCapacity>& registry_;
  ParameterBackend& backend_;
  std::array<Change, MaxChanges> changes_{};
  std::size_t size_{};
  EditLease lease_{};
  TimeUs expires_us_{};
  std::uint32_t base_revision_{};
  bool active_{};
};

}  // namespace robot
