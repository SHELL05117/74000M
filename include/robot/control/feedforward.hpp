#pragma once

#include <cmath>

namespace robot {

struct DirectionalFeedforwardGains {
  double kS_forward_V{};
  double kS_reverse_V{};
  double kV_Vs_per_unit{};
  double kA_Vs2_per_unit{};
};

inline bool validFeedforwardGains(
    const DirectionalFeedforwardGains& gains) noexcept {
  return std::isfinite(gains.kS_forward_V) &&
         std::isfinite(gains.kS_reverse_V) &&
         std::isfinite(gains.kV_Vs_per_unit) &&
         std::isfinite(gains.kA_Vs2_per_unit) &&
         gains.kS_forward_V >= 0.0 && gains.kS_reverse_V >= 0.0 &&
         gains.kV_Vs_per_unit >= 0.0 && gains.kA_Vs2_per_unit >= 0.0;
}

struct FeedforwardResult {
  double voltage_V{};
  bool valid{};
};

inline FeedforwardResult calculateFeedforward(
    const DirectionalFeedforwardGains& gains, double velocity,
    double acceleration, double motion_epsilon = 1e-9) noexcept {
  FeedforwardResult result{};
  if (!validFeedforwardGains(gains) || !std::isfinite(velocity) ||
      !std::isfinite(acceleration) || !std::isfinite(motion_epsilon) ||
      motion_epsilon < 0.0)
    return result;
  double static_voltage{};
  if (velocity > motion_epsilon ||
      (std::abs(velocity) <= motion_epsilon && acceleration > motion_epsilon))
    static_voltage = gains.kS_forward_V;
  else if (velocity < -motion_epsilon ||
           (std::abs(velocity) <= motion_epsilon &&
            acceleration < -motion_epsilon))
    static_voltage = -gains.kS_reverse_V;
  result.voltage_V = static_voltage + gains.kV_Vs_per_unit * velocity +
                     gains.kA_Vs2_per_unit * acceleration;
  result.valid = std::isfinite(result.voltage_V);
  return result;
}

struct DifferentialFeedforwardConfig {
  DirectionalFeedforwardGains left{};
  DirectionalFeedforwardGains right{};
};

struct DifferentialFeedforwardResult {
  double left_V{};
  double right_V{};
  bool valid{};
};

inline DifferentialFeedforwardResult calculateDifferentialFeedforward(
    const DifferentialFeedforwardConfig& config, double left_velocity,
    double left_acceleration, double right_velocity,
    double right_acceleration) noexcept {
  const auto left = calculateFeedforward(config.left, left_velocity,
                                         left_acceleration);
  const auto right = calculateFeedforward(config.right, right_velocity,
                                          right_acceleration);
  return {left.voltage_V, right.voltage_V, left.valid && right.valid};
}

}  // namespace robot
