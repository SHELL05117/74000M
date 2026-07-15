#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace robot {

struct PidConfig {
  double kp{};
  double ki{};
  double kd{};
  double output_min{};
  double output_max{};
  double integral_min{};
  double integral_max{};
  double integral_zone{};
  double derivative_cutoff_hz{};
  double error_deadband{};
  double min_dt_s{};
  double max_dt_s{};
};

enum PidStatus : std::uint32_t {
  kPidOk = 0,
  kPidInvalidConfig = 1u << 0,
  kPidInvalidInput = 1u << 1,
  kPidDeadband = 1u << 2,
  kPidIntegralOutsideZone = 1u << 3,
  kPidIntegralHeldForSaturation = 1u << 4,
  kPidOutputSaturatedLow = 1u << 5,
  kPidOutputSaturatedHigh = 1u << 6,
};

struct PidResult {
  double error{};
  double proportional{};
  double integral{};
  double derivative{};
  double unsaturated_output{};
  double output{};
  std::uint32_t status{kPidInvalidConfig};
  bool valid{};
};

inline bool validPidConfig(const PidConfig& config) noexcept {
  const bool finite =
      std::isfinite(config.kp) && std::isfinite(config.ki) &&
      std::isfinite(config.kd) && std::isfinite(config.output_min) &&
      std::isfinite(config.output_max) &&
      std::isfinite(config.integral_min) &&
      std::isfinite(config.integral_max) &&
      std::isfinite(config.integral_zone) &&
      std::isfinite(config.derivative_cutoff_hz) &&
      std::isfinite(config.error_deadband) &&
      std::isfinite(config.min_dt_s) && std::isfinite(config.max_dt_s);
  return finite && config.output_min < config.output_max &&
         config.integral_min <= config.integral_max &&
         config.integral_zone >= 0.0 && config.derivative_cutoff_hz >= 0.0 &&
         (config.kd == 0.0 || config.derivative_cutoff_hz > 0.0) &&
         config.error_deadband >= 0.0 && config.min_dt_s > 0.0 &&
         config.max_dt_s >= config.min_dt_s;
}

class EngineeringPid {
 public:
  explicit EngineeringPid(PidConfig config) noexcept : config_(config) {}

  bool valid() const noexcept { return validPidConfig(config_); }

  PidResult update(double setpoint, double measurement, double dt_s,
                   double feedforward = 0.0) noexcept {
    PidResult result{};
    if (!valid()) {
      result.status = kPidInvalidConfig;
      return result;
    }
    if (!std::isfinite(setpoint) || !std::isfinite(measurement) ||
        !std::isfinite(dt_s) || !std::isfinite(feedforward) ||
        dt_s < config_.min_dt_s || dt_s > config_.max_dt_s) {
      result.status = kPidInvalidInput;
      return result;
    }

    result.status = kPidOk;
    result.error = setpoint - measurement;
    if (std::abs(result.error) <= config_.error_deadband) {
      result.error = 0.0;
      result.status |= kPidDeadband;
    }
    result.proportional = config_.kp * result.error;

    double derivative_raw{};
    if (have_measurement_)
      derivative_raw = -(measurement - last_measurement_) / dt_s;
    if (!have_measurement_) {
      derivative_state_ = 0.0;
    } else if (config_.derivative_cutoff_hz > 0.0) {
      constexpr double kTwoPi = 6.283185307179586476925286766559;
      const double tau_s = 1.0 / (kTwoPi * config_.derivative_cutoff_hz);
      const double alpha = dt_s / (tau_s + dt_s);
      derivative_state_ += alpha * (derivative_raw - derivative_state_);
    } else {
      derivative_state_ = derivative_raw;
    }
    result.derivative = config_.kd * derivative_state_;

    double proposed_integral = integral_state_;
    if (std::abs(result.error) <= config_.integral_zone) {
      proposed_integral =
          std::clamp(integral_state_ + config_.ki * result.error * dt_s,
                     config_.integral_min, config_.integral_max);
      const double proposed_raw = result.proportional + proposed_integral +
                                  result.derivative + feedforward;
      const bool pushes_high = proposed_raw > config_.output_max &&
                               result.error > 0.0;
      const bool pushes_low = proposed_raw < config_.output_min &&
                              result.error < 0.0;
      if (pushes_high || pushes_low) {
        proposed_integral = integral_state_;
        result.status |= kPidIntegralHeldForSaturation;
      }
    } else {
      result.status |= kPidIntegralOutsideZone;
    }
    integral_state_ = proposed_integral;
    result.integral = integral_state_;
    result.unsaturated_output = result.proportional + result.integral +
                                result.derivative + feedforward;
    result.output = std::clamp(result.unsaturated_output, config_.output_min,
                               config_.output_max);
    if (result.unsaturated_output < config_.output_min)
      result.status |= kPidOutputSaturatedLow;
    if (result.unsaturated_output > config_.output_max)
      result.status |= kPidOutputSaturatedHigh;
    last_measurement_ = measurement;
    have_measurement_ = true;
    result.valid = std::isfinite(result.output);
    if (!result.valid) result.status |= kPidInvalidInput;
    return result;
  }

  void reset() noexcept {
    integral_state_ = 0.0;
    derivative_state_ = 0.0;
    last_measurement_ = 0.0;
    have_measurement_ = false;
  }

  double integralState() const noexcept { return integral_state_; }
  double derivativeState() const noexcept { return derivative_state_; }

 private:
  PidConfig config_{};
  double integral_state_{};
  double derivative_state_{};
  double last_measurement_{};
  bool have_measurement_{};
};

}  // namespace robot
