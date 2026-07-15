#pragma once

#include <cmath>
#include <limits>

#include "robot/core/units.hpp"

namespace robot {

struct Vec2 {
  double x_m{};
  double y_m{};
};

// T_WB: the body frame expressed in the world frame.
struct Pose2d {
  double x_m{};
  double y_m{};
  double theta_rad{};
};

// Finite body-frame motion over an interval, not instantaneous velocity.
struct Twist2d {
  double dx_m{};
  double dy_m{};
  double dtheta_rad{};
};

struct BodyVelocity2d {
  double vx_mps{};
  double vy_mps{};
  double omega_radps{};
};

inline bool finite(double value) noexcept { return std::isfinite(value); }

inline bool valid(const Vec2& value) noexcept {
  return finite(value.x_m) && finite(value.y_m);
}

inline bool valid(const Pose2d& value) noexcept {
  return finite(value.x_m) && finite(value.y_m) && finite(value.theta_rad);
}

inline bool valid(const Twist2d& value) noexcept {
  return finite(value.dx_m) && finite(value.dy_m) &&
         finite(value.dtheta_rad);
}

inline double wrapPi(double angle_rad) noexcept {
  const double two_pi = 2.0 * units::kPi;
  double wrapped =
      angle_rad - two_pi * std::floor((angle_rad + units::kPi) / two_pi);
  if (wrapped >= units::kPi) wrapped -= two_pi;
  return wrapped;
}

inline Vec2 rotate(const Vec2& vector, double angle_rad) noexcept {
  const double cosine = std::cos(angle_rad);
  const double sine = std::sin(angle_rad);
  return {cosine * vector.x_m - sine * vector.y_m,
          sine * vector.x_m + cosine * vector.y_m};
}

inline Vec2 inverseRotate(const Vec2& vector, double angle_rad) noexcept {
  return rotate(vector, -angle_rad);
}

inline Pose2d compose(const Pose2d& t_wa, const Pose2d& t_ab) noexcept {
  const Vec2 translated = rotate({t_ab.x_m, t_ab.y_m}, t_wa.theta_rad);
  return {t_wa.x_m + translated.x_m, t_wa.y_m + translated.y_m,
          t_wa.theta_rad + t_ab.theta_rad};
}

inline Pose2d inverse(const Pose2d& t_wb) noexcept {
  const Vec2 translated =
      inverseRotate({-t_wb.x_m, -t_wb.y_m}, t_wb.theta_rad);
  return {translated.x_m, translated.y_m, -t_wb.theta_rad};
}

inline Pose2d relativeTo(const Pose2d& t_wb,
                         const Pose2d& t_wa) noexcept {
  return compose(inverse(t_wa), t_wb);
}

inline double sinc(double angle_rad) noexcept {
  if (std::abs(angle_rad) < 1e-4) {
    const double squared = angle_rad * angle_rad;
    return 1.0 - squared / 6.0 + squared * squared / 120.0;
  }
  return std::sin(angle_rad) / angle_rad;
}

inline double cosc(double angle_rad) noexcept {
  if (std::abs(angle_rad) < 1e-4) {
    const double squared = angle_rad * angle_rad;
    return angle_rad * (0.5 - squared / 24.0 + squared * squared / 720.0);
  }
  return (1.0 - std::cos(angle_rad)) / angle_rad;
}

inline Pose2d exp(const Twist2d& twist) noexcept {
  const double sine_cardinal = sinc(twist.dtheta_rad);
  const double cosine_cardinal = cosc(twist.dtheta_rad);
  return {sine_cardinal * twist.dx_m - cosine_cardinal * twist.dy_m,
          cosine_cardinal * twist.dx_m + sine_cardinal * twist.dy_m,
          twist.dtheta_rad};
}

inline Pose2d integrateBodyTwist(const Pose2d& t_wb,
                                 const Twist2d& body_twist) noexcept {
  return compose(t_wb, exp(body_twist));
}

inline bool log(const Pose2d& t_ab, Twist2d& twist) noexcept {
  const double angle = t_ab.theta_rad;
  const double sine_cardinal = sinc(angle);
  const double cosine_cardinal = cosc(angle);
  const double determinant = sine_cardinal * sine_cardinal +
                             cosine_cardinal * cosine_cardinal;
  if (!(determinant > std::numeric_limits<double>::epsilon())) return false;
  twist.dx_m =
      (sine_cardinal * t_ab.x_m + cosine_cardinal * t_ab.y_m) / determinant;
  twist.dy_m =
      (-cosine_cardinal * t_ab.x_m + sine_cardinal * t_ab.y_m) / determinant;
  twist.dtheta_rad = angle;
  return valid(twist);
}

}  // namespace robot
