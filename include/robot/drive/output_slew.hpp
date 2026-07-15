#pragma once

#include <algorithm>
#include <cmath>

#include "robot/drive/voltage_allocation.hpp"

namespace robot {

struct OutputSlewConfig {
  double rise_V_per_s{};
  double fall_V_per_s{};
  double max_dt_s{};
};

class AsymmetricVoltageSlew {
 public:
  explicit AsymmetricVoltageSlew(OutputSlewConfig config) noexcept
      : config_(config) {}

  bool valid() const noexcept {
    return std::isfinite(config_.rise_V_per_s) &&
           config_.rise_V_per_s > 0.0 &&
           std::isfinite(config_.fall_V_per_s) &&
           config_.fall_V_per_s > 0.0 &&
           std::isfinite(config_.max_dt_s) && config_.max_dt_s > 0.0;
  }

  bool update(double target_V, double dt_s, double& output_V) noexcept {
    if (!valid() || !std::isfinite(target_V) || !std::isfinite(dt_s) ||
        dt_s < 0.0 || dt_s > config_.max_dt_s) {
      output_V = 0.0;
      return false;
    }

    if (value_V_ * target_V < 0.0) {
      value_V_ = moveToward(value_V_, 0.0,
                            config_.fall_V_per_s * dt_s);
    } else {
      const bool moving_away =
          std::abs(target_V) > std::abs(value_V_);
      const double rate = moving_away ? config_.rise_V_per_s
                                      : config_.fall_V_per_s;
      value_V_ = moveToward(value_V_, target_V, rate * dt_s);
    }
    output_V = value_V_;
    return true;
  }

  void reset(double value_V = 0.0) noexcept {
    value_V_ = std::isfinite(value_V) ? value_V : 0.0;
  }

  double value() const noexcept { return value_V_; }

 private:
  static double moveToward(double from, double to,
                           double max_delta) noexcept {
    return from + std::clamp(to - from, -max_delta, max_delta);
  }

  OutputSlewConfig config_{};
  double value_V_{};
};

class DifferentialOutputSlew {
 public:
  explicit DifferentialOutputSlew(OutputSlewConfig config) noexcept
      : left_(config), right_(config) {}

  bool valid() const noexcept { return left_.valid() && right_.valid(); }

  bool update(WheelVoltages target, double dt_s,
              WheelVoltages& output) noexcept {
    double left{};
    double right{};
    const bool left_ok = left_.update(target.left_V, dt_s, left);
    const bool right_ok = right_.update(target.right_V, dt_s, right);
    if (!left_ok || !right_ok) {
      reset();
      output = {};
      return false;
    }
    output = {left, right};
    return true;
  }

  void reset() noexcept {
    left_.reset();
    right_.reset();
  }

 private:
  AsymmetricVoltageSlew left_;
  AsymmetricVoltageSlew right_;
};

}  // namespace robot
