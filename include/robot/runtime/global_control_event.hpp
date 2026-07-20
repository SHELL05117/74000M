#pragma once

#include <cstdint>

#include "robot/core/frame.hpp"
#include "robot/state/controller_snapshot.hpp"

namespace robot {

enum GlobalControlEventBits : std::uint32_t {
  kGlobalControlNoEvent = 0,
  kGlobalCoastOnce = 1u << 0,
};

struct GlobalControlEvent {
  FrameHeader h{};
  std::uint32_t event_sequence{};
  std::uint32_t event_bits{};
};

class GlobalControlEventDetector {
 public:
  explicit GlobalControlEventDetector(
      std::uint32_t coast_button = kButtonLeft) noexcept
      : coast_button_(coast_button) {}

  GlobalControlEvent observe(
      const FrameHeader& header,
      const ControllerSnapshot& controller) noexcept {
    GlobalControlEvent event{};
    event.h = header;
    event.event_sequence = event_sequence_;
    const bool valid = coast_button_ != 0 && controller.connected &&
                       controller.api_ok &&
                       controller.h.time_us == header.time_us &&
                       controller.h.sequence == header.sequence &&
                       controller.h.mode_epoch == header.mode_epoch;
    if (!valid) {
      armed_ = false;
      button_down_ = false;
      return event;
    }

    const bool down = (controller.buttons & coast_button_) != 0;
    if (!armed_) {
      button_down_ = down;
      armed_ = !down;
      return event;
    }
    if (down && !button_down_) {
      event.event_sequence = ++event_sequence_;
      event.event_bits = kGlobalCoastOnce;
    }
    button_down_ = down;
    return event;
  }

 private:
  std::uint32_t coast_button_{};
  std::uint32_t event_sequence_{};
  bool button_down_{};
  bool armed_{};
};

}  // namespace robot
