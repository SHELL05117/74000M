#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "robot/hmi/types.hpp"

namespace robot {

inline bool criticalHmiAction(HmiAction action) noexcept {
  return action == HmiAction::ConfirmRoute ||
         action == HmiAction::ApplyParameterTransaction ||
         action == HmiAction::SaveParameterProfile ||
         action == HmiAction::RequestPoseReset ||
         action == HmiAction::RequestImuCalibration ||
         action == HmiAction::RequestFaultClear;
}

inline bool coalescibleHmiAction(HmiAction action) noexcept {
  return action == HmiAction::Navigate || action == HmiAction::Back;
}

struct HmiQueueStats {
  std::uint32_t high_watermark{};
  std::uint32_t navigation_dropped{};
  std::uint32_t critical_rejected{};
};

template <std::size_t Capacity>
class HmiEventQueue {
  static_assert(Capacity > 0, "HMI queue requires capacity");

 public:
  bool push(const HmiEvent& event) noexcept {
    if (size_ > 0 && coalescibleHmiAction(event.action)) {
      HmiEvent& last = events_[size_ - 1];
      if (last.origin == event.origin && last.action == event.action &&
          last.h.mode_epoch == event.h.mode_epoch) {
        last = event;
        return true;
      }
    }
    if (size_ >= Capacity) {
      if (!criticalHmiAction(event.action)) {
        ++stats_.navigation_dropped;
        return false;
      }
      std::size_t replace = Capacity;
      for (std::size_t i = 0; i < size_; ++i) {
        if (coalescibleHmiAction(events_[i].action)) {
          replace = i;
          break;
        }
      }
      if (replace == Capacity) {
        ++stats_.critical_rejected;
        return false;
      }
      for (std::size_t i = replace + 1; i < size_; ++i)
        events_[i - 1] = events_[i];
      --size_;
      ++stats_.navigation_dropped;
    }
    events_[size_++] = event;
    if (size_ > stats_.high_watermark)
      stats_.high_watermark = static_cast<std::uint32_t>(size_);
    return true;
  }

  bool pop(HmiEvent& event) noexcept {
    if (size_ == 0) return false;
    event = events_[0];
    for (std::size_t i = 1; i < size_; ++i) events_[i - 1] = events_[i];
    --size_;
    return true;
  }

  std::size_t size() const noexcept { return size_; }
  const HmiQueueStats& stats() const noexcept { return stats_; }

 private:
  std::array<HmiEvent, Capacity> events_{};
  std::size_t size_{};
  HmiQueueStats stats_{};
};

}  // namespace robot
