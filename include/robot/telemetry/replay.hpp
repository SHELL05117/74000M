#pragma once

#include <array>
#include <cstddef>

#include "robot/state/raw_inputs.hpp"

namespace robot {

template <std::size_t N>
class RawInputReplay {
 public:
  explicit RawInputReplay(const std::array<RawDriveInputs, N>& frames)
      : frames_(frames) {}

  bool next(RawDriveInputs& output) noexcept {
    if (index_ >= N) return false;
    output = frames_[index_++];
    return true;
  }

  void reset() noexcept { index_ = 0; }
  std::size_t remaining() const noexcept { return N - index_; }

 private:
  std::array<RawDriveInputs, N> frames_{};
  std::size_t index_{};
};

}  // namespace robot
