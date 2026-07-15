#pragma once

#include <cstdint>

namespace robot {

using RouteId = std::uint16_t;
using ParameterId = std::uint16_t;
using PageId = std::uint8_t;

namespace RouteIds {
constexpr RouteId kDoNothing = 0x0000;
}  // namespace RouteIds

namespace ParameterIds {
constexpr ParameterId kInvalid = 0x0000;
}  // namespace ParameterIds

namespace PageIds {
constexpr PageId kDashboard = 0x01;
constexpr PageId kAbout = 0x02;
}  // namespace PageIds

}  // namespace robot
