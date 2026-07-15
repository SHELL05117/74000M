#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "robot/core/frame.hpp"
#include "robot/hmi/types.hpp"

namespace robot {

enum class EditDomain : std::uint8_t {
  AutonSelection,
  Parameters,
  Maintenance,
};

struct EditLease {
  EditDomain domain{EditDomain::AutonSelection};
  HmiOrigin owner{HmiOrigin::Brain};
  std::uint32_t generation{};
  TimeUs expires_us{};
  std::uint32_t mode_epoch{};
  bool active{};
};

class EditLeaseManager {
 public:
  bool acquire(EditDomain domain, HmiOrigin owner, TimeUs now_us,
               TimeUs duration_us, std::uint32_t mode_epoch,
               EditLease& lease) noexcept {
    if (duration_us == 0) return false;
    expire(now_us, mode_epoch);
    auto& slot = leases_[static_cast<std::size_t>(domain)];
    if (slot.active && slot.owner != owner) return false;
    if (!slot.active) {
      ++generation_;
      if (generation_ == 0) ++generation_;
      slot = {domain, owner, generation_, now_us + duration_us, mode_epoch,
              true};
    } else {
      slot.expires_us = now_us + duration_us;
    }
    lease = slot;
    return true;
  }

  bool refresh(const EditLease& lease, TimeUs now_us,
               TimeUs duration_us) noexcept {
    if (duration_us == 0) return false;
    auto& slot = leases_[static_cast<std::size_t>(lease.domain)];
    if (!same(slot, lease) || now_us > slot.expires_us) return false;
    slot.expires_us = now_us + duration_us;
    return true;
  }

  bool owns(const EditLease& lease, TimeUs now_us,
            std::uint32_t mode_epoch) const noexcept {
    const auto& slot = leases_[static_cast<std::size_t>(lease.domain)];
    return same(slot, lease) && slot.mode_epoch == mode_epoch &&
           now_us <= slot.expires_us;
  }

  void release(const EditLease& lease) noexcept {
    auto& slot = leases_[static_cast<std::size_t>(lease.domain)];
    if (same(slot, lease)) slot = {};
  }

  void releaseOwner(HmiOrigin owner) noexcept {
    for (auto& lease : leases_)
      if (lease.active && lease.owner == owner) lease = {};
  }

  void expire(TimeUs now_us, std::uint32_t mode_epoch) noexcept {
    for (auto& lease : leases_)
      if (lease.active &&
          (lease.mode_epoch != mode_epoch || now_us > lease.expires_us))
        lease = {};
  }

  EditOwner owner(EditDomain domain) const noexcept {
    const auto& lease = leases_[static_cast<std::size_t>(domain)];
    if (!lease.active) return EditOwner::None;
    return lease.owner == HmiOrigin::Brain ? EditOwner::Brain
                                           : EditOwner::Controller;
  }

 private:
  static bool same(const EditLease& left, const EditLease& right) noexcept {
    return left.active && right.active && left.domain == right.domain &&
           left.owner == right.owner && left.generation == right.generation &&
           left.mode_epoch == right.mode_epoch;
  }

  std::array<EditLease, 3> leases_{};
  std::uint32_t generation_{};
};

}  // namespace robot
