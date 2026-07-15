#pragma once

#include <algorithm>
#include <cmath>

namespace robot {

struct AxisShapeConfig {
  double center_offset{};
  double deadband{};
  double cubic_weight{};
};

inline bool validAxisShape(const AxisShapeConfig& config) noexcept {
  return std::isfinite(config.center_offset) &&
         std::abs(config.center_offset) < 1.0 &&
         std::isfinite(config.deadband) && config.deadband >= 0.0 &&
         config.deadband < 1.0 && std::isfinite(config.cubic_weight) &&
         config.cubic_weight >= 0.0 && config.cubic_weight <= 1.0;
}

inline double centeredAxis(double raw, const AxisShapeConfig& config) noexcept {
  if (!std::isfinite(raw) || !validAxisShape(config)) return 0.0;
  return std::clamp(raw - config.center_offset, -1.0, 1.0);
}

inline double remapDeadband(double centered, double deadband) noexcept {
  if (!std::isfinite(centered) || !std::isfinite(deadband) ||
      deadband < 0.0 || deadband >= 1.0) {
    return 0.0;
  }
  centered = std::clamp(centered, -1.0, 1.0);
  if (std::abs(centered) <= deadband) return 0.0;
  return std::copysign((std::abs(centered) - deadband) /
                           (1.0 - deadband),
                       centered);
}

inline double shapeCenteredAxis(double centered,
                                const AxisShapeConfig& config) noexcept {
  if (!validAxisShape(config) || !std::isfinite(centered)) return 0.0;
  const double remapped = remapDeadband(centered, config.deadband);
  return (1.0 - config.cubic_weight) * remapped +
         config.cubic_weight * remapped * remapped * remapped;
}

class AsymmetricAxisSlew {
 public:
  AsymmetricAxisSlew(double rise_per_s, double fall_per_s,
                     double max_dt_s) noexcept
      : rise_per_s_(rise_per_s),
        fall_per_s_(fall_per_s),
        max_dt_s_(max_dt_s) {}

  bool valid() const noexcept {
    return std::isfinite(rise_per_s_) && rise_per_s_ > 0.0 &&
           std::isfinite(fall_per_s_) && fall_per_s_ > 0.0 &&
           std::isfinite(max_dt_s_) && max_dt_s_ > 0.0;
  }

  bool update(double target, double dt_s, double& output) noexcept {
    if (!valid() || !std::isfinite(target) || !std::isfinite(dt_s) ||
        dt_s < 0.0 || dt_s > max_dt_s_) {
      output = 0.0;
      return false;
    }
    target = std::clamp(target, -1.0, 1.0);
    if (value_ * target < 0.0) {
      value_ = moveToward(value_, 0.0, fall_per_s_ * dt_s);
    } else {
      const bool moving_away = std::abs(target) > std::abs(value_);
      value_ = moveToward(value_, target,
                          (moving_away ? rise_per_s_ : fall_per_s_) * dt_s);
    }
    output = value_;
    return true;
  }

  void reset() noexcept { value_ = 0.0; }

 private:
  static double moveToward(double from, double to,
                           double max_delta) noexcept {
    return from + std::clamp(to - from, -max_delta, max_delta);
  }

  double rise_per_s_{};
  double fall_per_s_{};
  double max_dt_s_{};
  double value_{};
};

}  // namespace robot
