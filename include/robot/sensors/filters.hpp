#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include "robot/core/frame.hpp"

namespace robot {

class Median3 {
 public:
  bool update(double input, double& output) noexcept {
    if (!std::isfinite(input)) return false;
    samples_[count_ < samples_.size() ? count_ : index_] = input;
    if (count_ < samples_.size()) {
      ++count_;
    } else {
      index_ = (index_ + 1) % samples_.size();
    }
    if (count_ < samples_.size()) {
      output = input;
      return true;
    }
    auto sorted = samples_;
    std::sort(sorted.begin(), sorted.end());
    output = sorted[1];
    return true;
  }

  void reset() noexcept {
    samples_ = {};
    count_ = 0;
    index_ = 0;
  }

 private:
  std::array<double, 3> samples_{};
  std::size_t count_{};
  std::size_t index_{};
};

class OnePoleLowPass {
 public:
  explicit OnePoleLowPass(double cutoff_hz) noexcept
      : cutoff_hz_(cutoff_hz) {}

  bool valid() const noexcept {
    return std::isfinite(cutoff_hz_) && cutoff_hz_ > 0.0;
  }

  bool update(double input, double dt_s, double& output) noexcept {
    if (!valid() || !std::isfinite(input) || !std::isfinite(dt_s) ||
        dt_s <= 0.0) {
      return false;
    }
    if (!initialized_) {
      value_ = input;
      initialized_ = true;
    } else {
      constexpr double kTwoPi =
          6.283185307179586476925286766559;
      const double tau_s = 1.0 / (kTwoPi * cutoff_hz_);
      const double alpha = dt_s / (tau_s + dt_s);
      value_ += alpha * (input - value_);
    }
    output = value_;
    return std::isfinite(output);
  }

  void reset() noexcept {
    value_ = 0.0;
    initialized_ = false;
  }

 private:
  double cutoff_hz_{};
  double value_{};
  bool initialized_{};
};

template <std::size_t N>
class WindowedSlope {
  static_assert(N >= 2, "slope window needs at least two samples");

 public:
  bool update(double value, TimeUs time_us, double& slope_per_s) noexcept {
    if (!std::isfinite(value) || (count_ > 0 && time_us <= last_time_us_))
      return false;

    values_[next_] = value;
    times_us_[next_] = time_us;
    next_ = (next_ + 1) % N;
    if (count_ < N) ++count_;
    last_time_us_ = time_us;
    if (count_ < 2) {
      slope_per_s = 0.0;
      return false;
    }

    const std::size_t oldest = count_ == N ? next_ : 0;
    const TimeUs origin_us = times_us_[oldest];
    double mean_t{};
    double mean_v{};
    for (std::size_t i = 0; i < count_; ++i) {
      const std::size_t index = (oldest + i) % N;
      mean_t += static_cast<double>(times_us_[index] - origin_us) * 1e-6;
      mean_v += values_[index];
    }
    mean_t /= static_cast<double>(count_);
    mean_v /= static_cast<double>(count_);

    double numerator{};
    double denominator{};
    for (std::size_t i = 0; i < count_; ++i) {
      const std::size_t index = (oldest + i) % N;
      const double t =
          static_cast<double>(times_us_[index] - origin_us) * 1e-6;
      numerator += (t - mean_t) * (values_[index] - mean_v);
      denominator += (t - mean_t) * (t - mean_t);
    }
    if (!std::isfinite(denominator) || denominator <= 0.0) return false;
    slope_per_s = numerator / denominator;
    return std::isfinite(slope_per_s);
  }

  void reset() noexcept {
    values_ = {};
    times_us_ = {};
    count_ = 0;
    next_ = 0;
    last_time_us_ = 0;
  }

 private:
  std::array<double, N> values_{};
  std::array<TimeUs, N> times_us_{};
  std::size_t count_{};
  std::size_t next_{};
  TimeUs last_time_us_{};
};

}  // namespace robot
