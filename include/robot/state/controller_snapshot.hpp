#pragma once

#include <cstdint>

#include "robot/core/frame.hpp"

namespace robot {

enum ControllerButton : std::uint32_t {
  kButtonL1 = 1u << 0,
  kButtonL2 = 1u << 1,
  kButtonR1 = 1u << 2,
  kButtonR2 = 1u << 3,
  kButtonUp = 1u << 4,
  kButtonDown = 1u << 5,
  kButtonLeft = 1u << 6,
  kButtonRight = 1u << 7,
  kButtonX = 1u << 8,
  kButtonB = 1u << 9,
  kButtonY = 1u << 10,
  kButtonA = 1u << 11,
};

struct ControllerSnapshot {
  FrameHeader h;
  double left_x{};
  double left_y{};
  double right_x{};
  double right_y{};
  std::uint32_t buttons{};
  bool connected{};
  bool api_ok{};
};

struct CompetitionSnapshot {
  FrameHeader h;
  bool disabled{true};
  bool autonomous{};
  bool field_connected{};
  bool api_ok{};
};

}  // namespace robot
