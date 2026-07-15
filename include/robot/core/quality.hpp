#pragma once

#include <cstdint>

namespace robot {

enum class Quality : std::uint8_t { Good, Degraded, Invalid };

inline Quality worstQuality(Quality left, Quality right) noexcept {
  return static_cast<std::uint8_t>(left) >= static_cast<std::uint8_t>(right)
             ? left
             : right;
}

}  // namespace robot
