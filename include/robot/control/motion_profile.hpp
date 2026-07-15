#pragma once

#include <algorithm>
#include <cmath>

namespace robot {

struct ProfileSample {
  double position{};
  double velocity{};
  double acceleration{};
  double jerk{};
  bool valid{};
};

struct TrapezoidProfileConfig {
  double max_velocity{};
  double max_acceleration{};
};

class TrapezoidProfile {
 public:
  bool configure(double displacement, TrapezoidProfileConfig config,
                 double initial_velocity = 0.0,
                 double final_velocity = 0.0) noexcept {
    reset();
    if (!std::isfinite(displacement) ||
        !std::isfinite(config.max_velocity) ||
        !std::isfinite(config.max_acceleration) ||
        !std::isfinite(initial_velocity) ||
        !std::isfinite(final_velocity) || config.max_velocity <= 0.0 ||
        config.max_acceleration <= 0.0)
      return false;
    direction_ = displacement < 0.0 ? -1.0 : 1.0;
    distance_ = std::abs(displacement);
    initial_velocity_ = initial_velocity * direction_;
    final_velocity_ = final_velocity * direction_;
    if (initial_velocity_ < 0.0 || final_velocity_ < 0.0 ||
        initial_velocity_ > config.max_velocity ||
        final_velocity_ > config.max_velocity)
      return false;
    max_acceleration_ = config.max_acceleration;
    if (distance_ == 0.0) {
      valid_ = initial_velocity_ == 0.0 && final_velocity_ == 0.0;
      return valid_;
    }

    const double unconstrained_peak_squared =
        max_acceleration_ * distance_ +
        0.5 * (initial_velocity_ * initial_velocity_ +
               final_velocity_ * final_velocity_);
    if (!(unconstrained_peak_squared >= 0.0)) return false;
    peak_velocity_ =
        std::min(config.max_velocity, std::sqrt(unconstrained_peak_squared));
    if (peak_velocity_ < initial_velocity_ ||
        peak_velocity_ < final_velocity_)
      return false;
    acceleration_distance_ =
        (peak_velocity_ * peak_velocity_ -
         initial_velocity_ * initial_velocity_) /
        (2.0 * max_acceleration_);
    deceleration_distance_ =
        (peak_velocity_ * peak_velocity_ -
         final_velocity_ * final_velocity_) /
        (2.0 * max_acceleration_);
    cruise_distance_ =
        distance_ - acceleration_distance_ - deceleration_distance_;
    if (cruise_distance_ < -1e-9) return false;
    cruise_distance_ = std::max(0.0, cruise_distance_);
    acceleration_time_s_ =
        (peak_velocity_ - initial_velocity_) / max_acceleration_;
    cruise_time_s_ = cruise_distance_ / peak_velocity_;
    deceleration_time_s_ =
        (peak_velocity_ - final_velocity_) / max_acceleration_;
    total_time_s_ =
        acceleration_time_s_ + cruise_time_s_ + deceleration_time_s_;
    valid_ = std::isfinite(total_time_s_) && total_time_s_ > 0.0;
    return valid_;
  }

  ProfileSample sample(double time_s) const noexcept {
    if (!valid_ || !std::isfinite(time_s)) return {};
    if (time_s <= 0.0)
      return {0.0, direction_ * initial_velocity_, 0.0, 0.0, true};
    if (time_s < acceleration_time_s_) {
      const double position = initial_velocity_ * time_s +
                              0.5 * max_acceleration_ * time_s * time_s;
      return {direction_ * position,
              direction_ *
                  (initial_velocity_ + max_acceleration_ * time_s),
              direction_ * max_acceleration_, 0.0, true};
    }
    if (time_s < acceleration_time_s_ + cruise_time_s_) {
      const double cruise_time = time_s - acceleration_time_s_;
      const double position =
          acceleration_distance_ + peak_velocity_ * cruise_time;
      return {direction_ * position, direction_ * peak_velocity_, 0.0, 0.0,
              true};
    }
    if (time_s < total_time_s_) {
      const double deceleration_time =
          time_s - acceleration_time_s_ - cruise_time_s_;
      const double position = acceleration_distance_ + cruise_distance_ +
                              peak_velocity_ * deceleration_time -
                              0.5 * max_acceleration_ * deceleration_time *
                                  deceleration_time;
      return {direction_ * position,
              direction_ *
                  (peak_velocity_ -
                   max_acceleration_ * deceleration_time),
              -direction_ * max_acceleration_, 0.0, true};
    }
    return {direction_ * distance_, direction_ * final_velocity_, 0.0, 0.0,
            true};
  }

  void reset() noexcept { *this = TrapezoidProfile{}; }
  bool valid() const noexcept { return valid_; }
  bool triangular() const noexcept {
    return valid_ && cruise_time_s_ <= 1e-12;
  }
  double totalTime() const noexcept { return total_time_s_; }
  double peakVelocity() const noexcept {
    return direction_ * peak_velocity_;
  }

 private:
  double direction_{1.0};
  double distance_{};
  double initial_velocity_{};
  double final_velocity_{};
  double peak_velocity_{};
  double max_acceleration_{};
  double acceleration_distance_{};
  double cruise_distance_{};
  double deceleration_distance_{};
  double acceleration_time_s_{};
  double cruise_time_s_{};
  double deceleration_time_s_{};
  double total_time_s_{};
  bool valid_{};
};

struct SCurveProfileConfig {
  double max_velocity{};
  double max_acceleration{};
  double max_jerk{};
};

// Minimum-duration quintic smoothstep with zero endpoint velocity and
// acceleration. It is an S-curve reference, not a claim about measured plant
// limits; those limits remain hardware-locked configuration.
class QuinticSCurveProfile {
 public:
  bool configure(double displacement, SCurveProfileConfig config) noexcept {
    reset();
    if (!std::isfinite(displacement) ||
        !std::isfinite(config.max_velocity) ||
        !std::isfinite(config.max_acceleration) ||
        !std::isfinite(config.max_jerk) || config.max_velocity <= 0.0 ||
        config.max_acceleration <= 0.0 || config.max_jerk <= 0.0)
      return false;
    displacement_ = displacement;
    const double distance = std::abs(displacement);
    if (distance == 0.0) {
      valid_ = true;
      return true;
    }
    constexpr double kMaxNormalizedVelocity = 1.875;
    constexpr double kMaxNormalizedAcceleration =
        5.7735026918962576451;  // 10 / sqrt(3)
    constexpr double kMaxNormalizedJerk = 60.0;
    const double velocity_time =
        kMaxNormalizedVelocity * distance / config.max_velocity;
    const double acceleration_time = std::sqrt(
        kMaxNormalizedAcceleration * distance / config.max_acceleration);
    const double jerk_time =
        std::cbrt(kMaxNormalizedJerk * distance / config.max_jerk);
    total_time_s_ =
        std::max(velocity_time, std::max(acceleration_time, jerk_time));
    valid_ = std::isfinite(total_time_s_) && total_time_s_ > 0.0;
    return valid_;
  }

  ProfileSample sample(double time_s) const noexcept {
    if (!valid_ || !std::isfinite(time_s)) return {};
    if (total_time_s_ == 0.0) return {displacement_, 0.0, 0.0, 0.0, true};
    const double u = std::clamp(time_s / total_time_s_, 0.0, 1.0);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double position_scale = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
    const double velocity_scale = 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
    const double acceleration_scale = 60.0 * u - 180.0 * u2 + 120.0 * u3;
    const double jerk_scale = 60.0 - 360.0 * u + 360.0 * u2;
    return {displacement_ * position_scale,
            displacement_ * velocity_scale / total_time_s_,
            displacement_ * acceleration_scale /
                (total_time_s_ * total_time_s_),
            displacement_ * jerk_scale /
                (total_time_s_ * total_time_s_ * total_time_s_),
            true};
  }

  void reset() noexcept {
    displacement_ = 0.0;
    total_time_s_ = 0.0;
    valid_ = false;
  }
  bool valid() const noexcept { return valid_; }
  double totalTime() const noexcept { return total_time_s_; }

 private:
  double displacement_{};
  double total_time_s_{};
  bool valid_{};
};

}  // namespace robot
