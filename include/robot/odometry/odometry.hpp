#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "robot/core/fault.hpp"
#include "robot/core/quality.hpp"
#include "robot/sensors/validated_inputs.hpp"
#include "robot/state/pose2d.hpp"

namespace robot {

enum class OdomLayout : std::uint8_t {
  DriveEncodersImu,
  TwoTrackingImu,
  ThreeTrackingImu,
};

struct WheelGeometry {
  double x_m{};
  double y_m{};
  double phi_rad{};
  double meters_per_sensor_rad{};
};

struct OdometryConfig {
  OdomLayout layout{OdomLayout::DriveEncodersImu};
  double left_drive_m_per_motor_rad{};
  double right_drive_m_per_motor_rad{};
  double effective_track_width_m{};
  std::array<WheelGeometry, kMaxTrackingWheels> tracking{};
  std::uint8_t tracking_count{};
  double max_dt_s{};
  double min_geometry_determinant{};
  double heading_disagreement_rad{};
  std::uint16_t slip_confirm_samples{};
};

struct OdomObservation {
  FrameHeader h{};
  CheckedScalar left_distance_m;
  CheckedScalar right_distance_m;
  std::array<CheckedScalar, kMaxTrackingWheels> tracking_distance_m{};
  CheckedScalar imu_rotation_rad;
  CheckedScalar imu_rate_radps;
};

enum class SlipState : std::uint8_t {
  None,
  Suspected,
  Confirmed,
  Unknown,
};

enum OdomFault : std::uint32_t {
  kOdomOk = 0,
  kOdomBadConfig = 1u << 0,
  kOdomTimingInvalid = 1u << 1,
  kOdomWarmingUp = 1u << 2,
  kOdomHeadingUnavailable = 1u << 3,
  kOdomTranslationUnavailable = 1u << 4,
  kOdomHeadingDisagreement = 1u << 5,
  kOdomInvalidReset = 1u << 6,
};

struct PoseEstimate {
  Pose2d pose{};
  BodyVelocity2d body_velocity{};
  Quality translation_quality{Quality::Invalid};
  Quality heading_quality{Quality::Invalid};
  SlipState slip{SlipState::Unknown};
  std::uint32_t fault_bits{};
  TimeUs timestamp_us{};
  std::uint32_t sequence{};
  std::uint32_t mode_epoch{};
  std::uint32_t reset_generation{};
};

struct ResetRequest {
  Pose2d target{};
  std::uint32_t request_id{};
};

inline bool validWheelGeometry(const WheelGeometry& wheel) noexcept {
  return std::isfinite(wheel.x_m) && std::isfinite(wheel.y_m) &&
         std::isfinite(wheel.phi_rad) &&
         std::isfinite(wheel.meters_per_sensor_rad) &&
         wheel.meters_per_sensor_rad > 0.0;
}

inline double translationAxisDeterminant(const WheelGeometry& first,
                                         const WheelGeometry& second) noexcept {
  return std::cos(first.phi_rad) * std::sin(second.phi_rad) -
         std::sin(first.phi_rad) * std::cos(second.phi_rad);
}

inline double fullWheelDeterminant(
    const std::array<WheelGeometry, kMaxTrackingWheels>& wheels) noexcept {
  double matrix[3][3]{};
  for (std::size_t i = 0; i < 3; ++i) {
    const double c = std::cos(wheels[i].phi_rad);
    const double s = std::sin(wheels[i].phi_rad);
    matrix[i][0] = c;
    matrix[i][1] = s;
    matrix[i][2] = -wheels[i].y_m * c + wheels[i].x_m * s;
  }
  return matrix[0][0] *
             (matrix[1][1] * matrix[2][2] -
              matrix[1][2] * matrix[2][1]) -
         matrix[0][1] *
             (matrix[1][0] * matrix[2][2] -
              matrix[1][2] * matrix[2][0]) +
         matrix[0][2] *
             (matrix[1][0] * matrix[2][1] -
              matrix[1][1] * matrix[2][0]);
}

inline bool validOdometryConfig(const OdometryConfig& config) noexcept {
  if (!std::isfinite(config.max_dt_s) || config.max_dt_s <= 0.0 ||
      !std::isfinite(config.min_geometry_determinant) ||
      config.min_geometry_determinant <= 0.0 ||
      !std::isfinite(config.heading_disagreement_rad) ||
      config.heading_disagreement_rad <= 0.0 ||
      config.slip_confirm_samples == 0) {
    return false;
  }
  if (config.layout == OdomLayout::DriveEncodersImu) {
    return std::isfinite(config.left_drive_m_per_motor_rad) &&
           config.left_drive_m_per_motor_rad > 0.0 &&
           std::isfinite(config.right_drive_m_per_motor_rad) &&
           config.right_drive_m_per_motor_rad > 0.0 &&
           std::isfinite(config.effective_track_width_m) &&
           config.effective_track_width_m > 0.0;
  }
  const std::size_t required =
      config.layout == OdomLayout::TwoTrackingImu ? 2u : 3u;
  if (config.tracking_count != required) return false;
  for (std::size_t i = 0; i < required; ++i)
    if (!validWheelGeometry(config.tracking[i])) return false;
  if (required == 2) {
    if (std::abs(translationAxisDeterminant(config.tracking[0],
                                            config.tracking[1])) <
        config.min_geometry_determinant) {
      return false;
    }
  } else {
    double best_axis_determinant{};
    for (std::size_t first = 0; first < 3; ++first) {
      for (std::size_t second = first + 1; second < 3; ++second) {
        best_axis_determinant = std::max(
            best_axis_determinant,
            std::abs(translationAxisDeterminant(config.tracking[first],
                                                config.tracking[second])));
      }
    }
    if (best_axis_determinant < config.min_geometry_determinant ||
        std::abs(fullWheelDeterminant(config.tracking)) <
            config.min_geometry_determinant) {
      return false;
    }
  }
  return true;
}

inline CheckedScalar averageMotorDistance(
    const std::array<ValidatedMotor, kMotorsPerSide>& motors,
    double meters_per_motor_rad) noexcept {
  CheckedScalar output{};
  if (!std::isfinite(meters_per_motor_rad) || meters_per_motor_rad <= 0.0)
    return output;
  double sum{};
  std::size_t count{};
  Quality quality{Quality::Good};
  TimeUs oldest_time{};
  std::uint32_t rejects{};
  for (const auto& motor : motors) {
    rejects |= motor.position_rad.reject_bits;
    if (motor.position_rad.quality == Quality::Invalid) continue;
    sum += motor.position_rad.value * meters_per_motor_rad;
    oldest_time = count == 0
                      ? motor.position_rad.sample_time_us
                      : std::min(oldest_time,
                                 motor.position_rad.sample_time_us);
    quality = worstQuality(quality, motor.position_rad.quality);
    ++count;
  }
  if (count == 0) {
    output.reject_bits = rejects;
    return output;
  }
  output.value = sum / static_cast<double>(count);
  output.sample_time_us = oldest_time;
  output.quality = count == motors.size() ? quality : Quality::Degraded;
  output.reject_bits = rejects;
  return output;
}

inline OdomObservation makeOdomObservation(const ValidatedInputs& inputs,
                                           const OdometryConfig& config) {
  OdomObservation observation{};
  observation.h = inputs.h;
  observation.left_distance_m = averageMotorDistance(
      inputs.left, config.left_drive_m_per_motor_rad);
  observation.right_distance_m = averageMotorDistance(
      inputs.right, config.right_drive_m_per_motor_rad);
  for (std::size_t i = 0; i < kMaxTrackingWheels; ++i) {
    const auto& checked = inputs.tracking[i].position_rad;
    auto& converted = observation.tracking_distance_m[i];
    converted = checked;
    if (checked.quality != Quality::Invalid &&
        validWheelGeometry(config.tracking[i])) {
      converted.value =
          checked.value * config.tracking[i].meters_per_sensor_rad;
    } else {
      converted.quality = Quality::Invalid;
    }
  }
  observation.imu_rotation_rad = inputs.imu.rotation_rad;
  observation.imu_rate_radps = inputs.imu.yaw_rate_radps;
  return observation;
}

class Odometry {
 public:
  explicit Odometry(OdometryConfig config) noexcept : config_(config) {}

  bool valid() const noexcept { return validOdometryConfig(config_); }

  bool requestReset(const ResetRequest& request) noexcept {
    if (!robot::valid(request.target) || request.request_id == 0 ||
        request.request_id == last_reset_request_id_) {
      return false;
    }
    pending_reset_ = request;
    reset_pending_ = true;
    return true;
  }

  PoseEstimate update(const OdomObservation& observation,
                      double dt_s) noexcept {
    PoseEstimate output = baseOutput(observation.h);
    if (!valid()) {
      output.fault_bits |= kOdomBadConfig;
      return output;
    }
    if (!validTiming(observation.h, dt_s)) {
      output.fault_bits |= kOdomTimingInvalid;
      return output;
    }

    if (!have_frame_ || observation.h.mode_epoch != last_epoch_) {
      clearBaselines();
      have_frame_ = true;
      last_epoch_ = observation.h.mode_epoch;
    }
    last_sequence_ = observation.h.sequence;

    if (reset_pending_) {
      pose_ = pending_reset_.target;
      last_reset_request_id_ = pending_reset_.request_id;
      reset_pending_ = false;
      ++reset_generation_;
      captureBaselines(observation);
      slip_count_ = 0;
      output = baseOutput(observation.h);
      output.translation_quality =
          resetTranslationBaselineAvailable() ? Quality::Degraded
                                              : Quality::Invalid;
      output.heading_quality =
          resetHeadingBaselineAvailable() ? Quality::Degraded
                                          : Quality::Invalid;
      output.fault_bits |= kOdomWarmingUp;
      if (output.translation_quality == Quality::Invalid)
        output.fault_bits |= kOdomTranslationUnavailable;
      if (output.heading_quality == Quality::Invalid)
        output.fault_bits |= kOdomHeadingUnavailable;
      return output;
    }

    Delta left = makeDelta(observation.left_distance_m, left_baseline_);
    Delta right = makeDelta(observation.right_distance_m, right_baseline_);
    std::array<Delta, kMaxTrackingWheels> tracking{};
    for (std::size_t i = 0; i < tracking.size(); ++i)
      tracking[i] =
          makeDelta(observation.tracking_distance_m[i], tracking_baseline_[i]);
    Delta imu = makeDelta(observation.imu_rotation_rad, imu_baseline_);

    WheelHeading wheel_heading = solveWheelHeading(left, right, tracking);
    HeadingStep heading{};
    if (imu.valid) {
      heading.dtheta_rad = imu.value;
      heading.quality = imu.quality;
      heading.using_imu = true;
    } else if (wheel_heading.valid) {
      heading.dtheta_rad = wheel_heading.dtheta_rad;
      heading.quality = Quality::Degraded;
    } else {
      output.fault_bits |= kOdomHeadingUnavailable | kOdomWarmingUp;
    }

    if (heading.quality != Quality::Invalid && wheel_heading.valid &&
        heading.using_imu) {
      const double disagreement =
          std::abs(wrapPi(wheel_heading.dtheta_rad - heading.dtheta_rad));
      if (disagreement > config_.heading_disagreement_rad) {
        output.fault_bits |= kOdomHeadingDisagreement;
        ++slip_count_;
        output.slip = slip_count_ >= config_.slip_confirm_samples
                          ? SlipState::Confirmed
                          : SlipState::Suspected;
        heading.quality = Quality::Degraded;
      } else {
        slip_count_ = 0;
        output.slip = SlipState::None;
      }
    } else {
      slip_count_ = 0;
      output.slip = SlipState::Unknown;
    }

    TranslationStep translation{};
    if (heading.quality != Quality::Invalid)
      translation = solveTranslation(left, right, tracking,
                                     heading.dtheta_rad);
    if (translation.quality == Quality::Invalid)
      output.fault_bits |= kOdomTranslationUnavailable;
    if (!heading.using_imu && translation.quality != Quality::Invalid)
      translation.quality = Quality::Degraded;
    if (output.slip == SlipState::Suspected &&
        translation.quality == Quality::Good)
      translation.quality = Quality::Degraded;
    if (output.slip == SlipState::Confirmed)
      translation.quality = Quality::Invalid;

    if (heading.quality != Quality::Invalid &&
        translation.quality != Quality::Invalid) {
      const Twist2d twist{translation.dx_m, translation.dy_m,
                          heading.dtheta_rad};
      if (robot::valid(twist)) {
        pose_ = integrateBodyTwist(pose_, twist);
        output.body_velocity = {translation.dx_m / dt_s,
                                translation.dy_m / dt_s,
                                heading.dtheta_rad / dt_s};
      }
    } else if (heading.quality != Quality::Invalid) {
      pose_.theta_rad += heading.dtheta_rad;
      output.body_velocity.omega_radps = heading.dtheta_rad / dt_s;
    }

    output.pose = pose_;
    output.translation_quality = translation.quality;
    output.heading_quality = heading.quality;
    output.reset_generation = reset_generation_;
    return output;
  }

  void resetEstimator() noexcept {
    pose_ = {};
    clearBaselines();
    have_frame_ = false;
    last_epoch_ = 0;
    last_sequence_ = 0;
    slip_count_ = 0;
    reset_pending_ = false;
  }

 private:
  struct Baseline {
    double value{};
    bool ready{};
  };

  struct Delta {
    double value{};
    Quality quality{Quality::Invalid};
    bool valid{};
  };

  struct WheelHeading {
    double dtheta_rad{};
    bool valid{};
  };

  struct HeadingStep {
    double dtheta_rad{};
    Quality quality{Quality::Invalid};
    bool using_imu{};
  };

  struct TranslationStep {
    double dx_m{};
    double dy_m{};
    Quality quality{Quality::Invalid};
  };

  PoseEstimate baseOutput(const FrameHeader& header) const noexcept {
    PoseEstimate output{};
    output.pose = pose_;
    output.timestamp_us = header.time_us;
    output.sequence = header.sequence;
    output.mode_epoch = header.mode_epoch;
    output.reset_generation = reset_generation_;
    return output;
  }

  bool validTiming(const FrameHeader& header, double dt_s) const noexcept {
    return std::isfinite(dt_s) && dt_s > 0.0 && dt_s <= config_.max_dt_s &&
           (!have_frame_ || header.mode_epoch != last_epoch_ ||
            header.sequence > last_sequence_);
  }

  static Delta makeDelta(const CheckedScalar& checked,
                         Baseline& baseline) noexcept {
    if (checked.quality == Quality::Invalid ||
        !std::isfinite(checked.value)) {
      baseline.ready = false;
      return {};
    }
    if (!baseline.ready) {
      baseline.value = checked.value;
      baseline.ready = true;
      return {};
    }
    const double delta = checked.value - baseline.value;
    baseline.value = checked.value;
    if (!std::isfinite(delta)) {
      baseline.ready = false;
      return {};
    }
    return {delta, checked.quality, true};
  }

  WheelHeading solveWheelHeading(
      const Delta& left, const Delta& right,
      const std::array<Delta, kMaxTrackingWheels>& tracking) const noexcept {
    if (config_.layout == OdomLayout::DriveEncodersImu) {
      if (!left.valid || !right.valid) return {};
      return {(right.value - left.value) /
                  config_.effective_track_width_m,
              true};
    }
    if (config_.layout != OdomLayout::ThreeTrackingImu) return {};
    for (const auto& delta : tracking)
      if (!delta.valid) return {};

    double augmented[3][4]{};
    for (std::size_t i = 0; i < 3; ++i) {
      const double c = std::cos(config_.tracking[i].phi_rad);
      const double s = std::sin(config_.tracking[i].phi_rad);
      augmented[i][0] = c;
      augmented[i][1] = s;
      augmented[i][2] =
          -config_.tracking[i].y_m * c + config_.tracking[i].x_m * s;
      augmented[i][3] = tracking[i].value;
    }
    for (std::size_t column = 0; column < 3; ++column) {
      std::size_t pivot = column;
      for (std::size_t row = column + 1; row < 3; ++row)
        if (std::abs(augmented[row][column]) >
            std::abs(augmented[pivot][column]))
          pivot = row;
      if (std::abs(augmented[pivot][column]) <
          config_.min_geometry_determinant)
        return {};
      if (pivot != column)
        for (std::size_t entry = column; entry < 4; ++entry)
          std::swap(augmented[pivot][entry], augmented[column][entry]);
      const double divisor = augmented[column][column];
      for (std::size_t entry = column; entry < 4; ++entry)
        augmented[column][entry] /= divisor;
      for (std::size_t row = 0; row < 3; ++row) {
        if (row == column) continue;
        const double factor = augmented[row][column];
        for (std::size_t entry = column; entry < 4; ++entry)
          augmented[row][entry] -= factor * augmented[column][entry];
      }
    }
    return {augmented[2][3], std::isfinite(augmented[2][3])};
  }

  TranslationStep solveTranslation(
      const Delta& left, const Delta& right,
      const std::array<Delta, kMaxTrackingWheels>& tracking,
      double dtheta_rad) const noexcept {
    if (config_.layout == OdomLayout::DriveEncodersImu) {
      if (!left.valid || !right.valid) return {};
      return {0.5 * (left.value + right.value), 0.0,
              worstQuality(left.quality, right.quality)};
    }

    double n00{};
    double n01{};
    double n11{};
    double r0{};
    double r1{};
    std::size_t count{};
    Quality quality{Quality::Good};
    for (std::size_t i = 0; i < config_.tracking_count; ++i) {
      if (!tracking[i].valid) continue;
      const double c = std::cos(config_.tracking[i].phi_rad);
      const double s = std::sin(config_.tracking[i].phi_rad);
      const double lever =
          -config_.tracking[i].y_m * c + config_.tracking[i].x_m * s;
      const double corrected = tracking[i].value - lever * dtheta_rad;
      n00 += c * c;
      n01 += c * s;
      n11 += s * s;
      r0 += c * corrected;
      r1 += s * corrected;
      quality = worstQuality(quality, tracking[i].quality);
      ++count;
    }
    const double determinant = n00 * n11 - n01 * n01;
    if (count < 2 || !std::isfinite(determinant) ||
        std::abs(determinant) < config_.min_geometry_determinant)
      return {};
    const double dx = (n11 * r0 - n01 * r1) / determinant;
    const double dy = (-n01 * r0 + n00 * r1) / determinant;
    if (!std::isfinite(dx) || !std::isfinite(dy)) return {};
    if (count < config_.tracking_count) quality = Quality::Degraded;
    return {dx, dy, quality};
  }

  void captureBaselines(const OdomObservation& observation) noexcept {
    capture(observation.left_distance_m, left_baseline_);
    capture(observation.right_distance_m, right_baseline_);
    for (std::size_t i = 0; i < tracking_baseline_.size(); ++i)
      capture(observation.tracking_distance_m[i], tracking_baseline_[i]);
    capture(observation.imu_rotation_rad, imu_baseline_);
  }

  static void capture(const CheckedScalar& checked,
                      Baseline& baseline) noexcept {
    baseline.ready = checked.quality != Quality::Invalid &&
                     std::isfinite(checked.value);
    baseline.value = baseline.ready ? checked.value : 0.0;
  }

  void clearBaselines() noexcept {
    left_baseline_ = {};
    right_baseline_ = {};
    tracking_baseline_ = {};
    imu_baseline_ = {};
  }

  bool resetHeadingBaselineAvailable() const noexcept {
    if (imu_baseline_.ready) return true;
    if (config_.layout == OdomLayout::DriveEncodersImu)
      return left_baseline_.ready && right_baseline_.ready;
    if (config_.layout == OdomLayout::ThreeTrackingImu)
      return tracking_baseline_[0].ready && tracking_baseline_[1].ready &&
             tracking_baseline_[2].ready;
    return false;
  }

  bool resetTranslationBaselineAvailable() const noexcept {
    if (!resetHeadingBaselineAvailable()) return false;
    if (config_.layout == OdomLayout::DriveEncodersImu)
      return left_baseline_.ready && right_baseline_.ready;
    double best_axis_determinant{};
    for (std::size_t first = 0; first < config_.tracking_count; ++first) {
      if (!tracking_baseline_[first].ready) continue;
      for (std::size_t second = first + 1;
           second < config_.tracking_count; ++second) {
        if (!tracking_baseline_[second].ready) continue;
        best_axis_determinant = std::max(
            best_axis_determinant,
            std::abs(translationAxisDeterminant(config_.tracking[first],
                                                config_.tracking[second])));
      }
    }
    return best_axis_determinant >= config_.min_geometry_determinant;
  }

  OdometryConfig config_{};
  Pose2d pose_{};
  Baseline left_baseline_{};
  Baseline right_baseline_{};
  std::array<Baseline, kMaxTrackingWheels> tracking_baseline_{};
  Baseline imu_baseline_{};
  ResetRequest pending_reset_{};
  std::uint32_t last_reset_request_id_{};
  std::uint32_t reset_generation_{};
  std::uint32_t last_epoch_{};
  std::uint32_t last_sequence_{};
  std::uint16_t slip_count_{};
  bool reset_pending_{};
  bool have_frame_{};
};

}  // namespace robot
