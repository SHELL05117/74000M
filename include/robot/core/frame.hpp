#pragma once

#include <cstdint>

namespace robot {

using TimeUs = std::uint64_t;

struct FrameHeader {
  TimeUs time_us{};
  std::uint32_t sequence{};
  std::uint32_t mode_epoch{};
};

inline bool sameEpoch(const FrameHeader& left,
                      const FrameHeader& right) noexcept {
  return left.mode_epoch == right.mode_epoch;
}

}  // namespace robot
