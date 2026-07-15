#pragma once

#include <cmath>

namespace robot {

struct ChassisSpeeds {
  double vx_mps{};
  double omega_radps{};
};

struct WheelSpeeds {
  double left_mps{};
  double right_mps{};
};

class DifferentialKinematics {
 public:
  explicit DifferentialKinematics(double track_width_m) noexcept
      : track_width_m_(track_width_m) {}

  bool valid() const noexcept {
    return std::isfinite(track_width_m_) && track_width_m_ > 0.0;
  }

  bool inverse(const ChassisSpeeds& chassis, WheelSpeeds& wheels) const
      noexcept {
    if (!valid() || !std::isfinite(chassis.vx_mps) ||
        !std::isfinite(chassis.omega_radps)) {
      wheels = {};
      return false;
    }
    const double half_turn =
        0.5 * track_width_m_ * chassis.omega_radps;
    wheels = {chassis.vx_mps - half_turn, chassis.vx_mps + half_turn};
    return std::isfinite(wheels.left_mps) &&
           std::isfinite(wheels.right_mps);
  }

  bool forward(const WheelSpeeds& wheels, ChassisSpeeds& chassis) const
      noexcept {
    if (!valid() || !std::isfinite(wheels.left_mps) ||
        !std::isfinite(wheels.right_mps)) {
      chassis = {};
      return false;
    }
    chassis = {0.5 * (wheels.left_mps + wheels.right_mps),
               (wheels.right_mps - wheels.left_mps) / track_width_m_};
    return std::isfinite(chassis.vx_mps) &&
           std::isfinite(chassis.omega_radps);
  }

 private:
  double track_width_m_{};
};

}  // namespace robot
