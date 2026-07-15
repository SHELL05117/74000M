#pragma once

#include <array>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <cstring>

#include "robot/hmi/types.hpp"
#include "robot/platform/io.hpp"

namespace robot {

constexpr std::size_t kControllerRows = 3;
constexpr std::size_t kControllerColumnsWithTerminator = 20;

struct ControllerFrame {
  std::array<std::array<char, kControllerColumnsWithTerminator>,
             kControllerRows>
      lines{};
  std::uint32_t model_sequence{};
};

inline bool printableAsciiLine(
    const std::array<char, kControllerColumnsWithTerminator>& line) noexcept {
  bool terminated = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const unsigned char value = static_cast<unsigned char>(line[i]);
    if (value == 0) {
      terminated = true;
      break;
    }
    if (value < 0x20 || value > 0x7E) return false;
  }
  return terminated;
}

inline void controllerLine(
    std::array<char, kControllerColumnsWithTerminator>& destination,
    const char* format, ...) noexcept {
  destination.fill('\0');
  va_list arguments;
  va_start(arguments, format);
  std::vsnprintf(destination.data(), destination.size(), format, arguments);
  va_end(arguments);
  destination.back() = '\0';
}

inline const char* shortModeName(CompetitionMode mode) noexcept {
  switch (mode) {
    case CompetitionMode::Boot:
      return "BOOT";
    case CompetitionMode::Calibrating:
      return "CAL";
    case CompetitionMode::Disabled:
      return "DIS";
    case CompetitionMode::Driver:
      return "DRV";
    case CompetitionMode::AutonomousInterface:
      return "AUTO";
    case CompetitionMode::Test:
      return "TEST";
    case CompetitionMode::FaultStop:
      return "STOP";
  }
  return "?";
}

inline ControllerFrame formatControllerFrame(const HmiModel& model) noexcept {
  ControllerFrame frame{};
  frame.model_sequence = model.h.sequence;
  switch (model.controller_page) {
    case ControllerPage::CompetitionHud:
      controllerLine(frame.lines[0], "%.7s %-4s %s", model.team_number.data(),
                     shortModeName(model.mode), model.enabled ? "EN" : "SAFE");
      controllerLine(frame.lines[1], "X%+.2f Y%+.2f", model.pose.x_m,
                     model.pose.y_m);
      controllerLine(frame.lines[2], "B%.1fV F%u D%.0f%%", model.battery_V,
                     model.active_fault_count, model.output_derate * 100.0);
      break;
    case ControllerPage::AutoSelect:
      controllerLine(frame.lines[0], "AUTON %s",
                     model.route_locked ? "LOCKED" : "DRAFT");
      controllerLine(frame.lines[1], "D%lu C%lu",
                     static_cast<unsigned long>(model.draft_route),
                     static_cast<unsigned long>(model.confirmed_route));
      controllerLine(frame.lines[2], "REV %lu %s",
                     static_cast<unsigned long>(model.route_revision),
                     model.route_confirmed ? "OK" : "CONFIRM");
      break;
    case ControllerPage::PoseDebug:
      controllerLine(frame.lines[0], "POSE Q%u/%u",
                     static_cast<unsigned>(model.translation_quality),
                     static_cast<unsigned>(model.heading_quality));
      controllerLine(frame.lines[1], "X%+.3f Y%+.3f", model.pose.x_m,
                     model.pose.y_m);
      controllerLine(frame.lines[2], "H%+.3f V%.2f", model.pose.theta_rad,
                     model.velocity.vx_mps);
      break;
    case ControllerPage::HealthSummary:
      controllerLine(frame.lines[0], "HEALTH F%u",
                     model.active_fault_count);
      controllerLine(frame.lines[1], "TEMP %.1fC", model.max_motor_temperature_C);
      controllerLine(frame.lines[2], "DERATE %.0f%%", model.output_derate * 100.0);
      break;
    case ControllerPage::FaultDetail:
      controllerLine(frame.lines[0], "FAULT SEV %u",
                     static_cast<unsigned>(model.highest_fault_severity));
      controllerLine(frame.lines[1], "%.19s",
                     model.highest_fault_short.data());
      controllerLine(frame.lines[2], "COUNT %u", model.active_fault_count);
      break;
    case ControllerPage::TimingAndLog:
      controllerLine(frame.lines[0], "LOG %s SD %s",
                     model.log_sink_ok ? "OK" : "ERR",
                     model.sd_inserted ? "YES" : "NO");
      controllerLine(frame.lines[1], "DROP %lu",
                     static_cast<unsigned long>(model.log_dropped_total));
      controllerLine(frame.lines[2], "SEQ %lu",
                     static_cast<unsigned long>(model.h.sequence));
      break;
    case ControllerPage::ParameterEdit:
      controllerLine(frame.lines[0], "PARAM %lu",
                     static_cast<unsigned long>(model.selected_parameter));
      controllerLine(frame.lines[1], "%.4g -> %.4g", model.active_value,
                     model.staged_value);
      controllerLine(frame.lines[2], "%s REV %lu",
                     model.parameter_apply_allowed ? "APPLY" : "LOCK",
                     static_cast<unsigned long>(model.parameter_revision));
      break;
    case ControllerPage::UiLocked:
      controllerLine(frame.lines[0], "UI LOCKED");
      controllerLine(frame.lines[1], "OWNER %s",
                     model.parameter_edit_owner == EditOwner::Brain
                         ? "BRAIN"
                         : "OTHER");
      controllerLine(frame.lines[2], "BACK TO HUD");
      break;
  }
  return frame;
}

inline ControllerFrame formatPoseControllerFrame(
    const HmiModel& model) noexcept {
  ControllerFrame frame{};
  frame.model_sequence = model.h.sequence;
  const bool translation_valid =
      model.translation_quality != Quality::Invalid &&
      std::isfinite(model.pose.x_m) && std::isfinite(model.pose.y_m);
  const bool heading_valid = model.heading_quality != Quality::Invalid &&
                             std::isfinite(model.pose.theta_rad);

  if (translation_valid) {
    controllerLine(frame.lines[0], "X:%+.3fm", model.pose.x_m);
    controllerLine(frame.lines[1], "Y:%+.3fm", model.pose.y_m);
  } else {
    controllerLine(frame.lines[0], "X:ERR");
    controllerLine(frame.lines[1], "Y:ERR");
  }
  if (heading_valid) {
    controllerLine(frame.lines[2], "H:%+.1fdeg",
                   model.pose.theta_rad / units::kRadPerDeg);
  } else {
    controllerLine(frame.lines[2], "H:ERR");
  }
  return frame;
}

// Sends at most one changed row per tick. This respects the controller's slow
// text transport and also prevents screen traffic from occupying the 10 ms
// control path.
class ControllerPoseRenderer {
 public:
  bool tick(const HmiModel& model, ControllerDisplayIO& io) noexcept {
    const ControllerFrame target = formatPoseControllerFrame(model);
    for (std::size_t offset = 0; offset < kControllerRows; ++offset) {
      const std::size_t row = (next_row_ + offset) % kControllerRows;
      if (initialized_rows_[row] &&
          std::strncmp(last_.lines[row].data(), target.lines[row].data(),
                       kControllerColumnsWithTerminator) == 0) {
        continue;
      }
      if (!io.writeLine(static_cast<std::uint8_t>(row),
                        target.lines[row].data())) {
        return false;
      }
      next_row_ = (row + 1) % kControllerRows;
      last_.lines[row] = target.lines[row];
      last_.model_sequence = target.model_sequence;
      initialized_rows_[row] = true;
      return true;
    }
    last_.model_sequence = target.model_sequence;
    return false;
  }

 private:
  ControllerFrame last_{};
  std::array<bool, kControllerRows> initialized_rows_{};
  std::size_t next_row_{};
};

struct BrainRect {
  std::int16_t x{};
  std::int16_t y{};
  std::int16_t width{};
  std::int16_t height{};
};

struct BrainLayout {
  BrainRect status_bar{0, 0, 480, 24};
  BrainRect content{0, 24, 480, 172};
  BrainRect navigation{0, 196, 480, 44};
  std::array<BrainRect, 4> navigation_targets{
      BrainRect{6, 200, 68, 36}, BrainRect{138, 200, 68, 36},
      BrainRect{274, 200, 68, 36}, BrainRect{406, 200, 68, 36}};
};

inline bool rectInsideBrain(const BrainRect& rect) noexcept {
  return rect.x >= 0 && rect.y >= 0 && rect.width > 0 && rect.height > 0 &&
         static_cast<std::int32_t>(rect.x) + rect.width <= 480 &&
         static_cast<std::int32_t>(rect.y) + rect.height <= 240;
}

inline bool validTouchTarget(const BrainRect& rect) noexcept {
  return rectInsideBrain(rect) && rect.width >= 40 && rect.height >= 36;
}

inline bool validBrainLayout(const BrainLayout& layout) noexcept {
  if (!rectInsideBrain(layout.status_bar) || !rectInsideBrain(layout.content) ||
      !rectInsideBrain(layout.navigation))
    return false;
  for (const auto& target : layout.navigation_targets)
    if (!validTouchTarget(target) || target.y < layout.navigation.y ||
        target.y + target.height >
            layout.navigation.y + layout.navigation.height)
      return false;
  return true;
}

}  // namespace robot
