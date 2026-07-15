#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "robot/config/robot_identity.hpp"
#include "robot/core/frame.hpp"

namespace robot {

enum class ProfileState : std::uint8_t {
  Draft,
  BenchValidated,
  FieldValidated,
  CompetitionApproved,
  Retired,
};

enum class CalibrationSource : std::uint8_t {
  Assumed,
  NominalCad,
  Measured,
  Fitted,
};

struct CalibrationProvenance {
  std::uint32_t schema_version{};
  std::uint32_t calibration_revision{};
  std::uint32_t robot_id_hash{};
  std::uint32_t software_commit_hash{};
  std::uint32_t training_dataset_hash{};
  std::uint32_t validation_dataset_hash{};
  std::uint32_t conditions_hash{};
  CalibrationSource source{CalibrationSource::Assumed};
  ProfileState profile_state{ProfileState::Draft};
  VerificationLevel verification{VerificationLevel::Unverified};
};

inline bool validCalibrationProvenance(
    const CalibrationProvenance& provenance) noexcept {
  VerificationLevel required{VerificationLevel::Unverified};
  switch (provenance.profile_state) {
    case ProfileState::Draft:
      required = VerificationLevel::Implemented;
      break;
    case ProfileState::BenchValidated:
      required = VerificationLevel::HILValidated;
      break;
    case ProfileState::FieldValidated:
      required = VerificationLevel::FieldValidated;
      break;
    case ProfileState::CompetitionApproved:
      required = VerificationLevel::CompetitionApproved;
      break;
    case ProfileState::Retired:
      required = VerificationLevel::HILValidated;
      break;
  }
  return provenance.schema_version > 0 && provenance.robot_id_hash != 0 &&
         provenance.software_commit_hash != 0 &&
         provenance.training_dataset_hash != 0 &&
         provenance.validation_dataset_hash != 0 &&
         provenance.training_dataset_hash !=
             provenance.validation_dataset_hash &&
         provenance.conditions_hash != 0 &&
         atLeast(provenance.verification, required);
}

struct RegressionMetrics {
  double rmse{};
  double mae{};
  double max_abs_error{};
  std::uint32_t sample_count{};
};

template <typename Sample, std::size_t Capacity>
class FixedDataset {
  static_assert(Capacity > 0, "dataset requires capacity");

 public:
  bool add(const Sample& sample) noexcept {
    if (size_ >= Capacity) return false;
    samples_[size_++] = sample;
    return true;
  }

  const Sample& operator[](std::size_t index) const noexcept {
    return samples_[index];
  }
  std::size_t size() const noexcept { return size_; }
  constexpr std::size_t capacity() const noexcept { return Capacity; }
  void clear() noexcept { size_ = 0; }

 private:
  std::array<Sample, Capacity> samples_{};
  std::size_t size_{};
};

struct LineFitSample {
  double input{};
  double output{};
  bool training{};
  bool accepted{};
};

struct LineFitResult {
  double slope{};
  double intercept{};
  RegressionMetrics training{};
  RegressionMetrics validation{};
  std::uint32_t excluded_count{};
  bool valid{};
};

inline void addError(RegressionMetrics& metrics, double error,
                     double& squared_sum,
                     double& absolute_sum) noexcept {
  squared_sum += error * error;
  absolute_sum += std::abs(error);
  metrics.max_abs_error =
      std::max(metrics.max_abs_error, std::abs(error));
  ++metrics.sample_count;
}

template <std::size_t Capacity>
LineFitResult fitLine(const FixedDataset<LineFitSample, Capacity>& data,
                      std::uint32_t minimum_training_samples = 2) noexcept {
  LineFitResult result{};
  double sum_x{};
  double sum_y{};
  double sum_xx{};
  double sum_xy{};
  std::uint32_t training_count{};
  for (std::size_t i = 0; i < data.size(); ++i) {
    const auto& sample = data[i];
    if (!sample.accepted || !std::isfinite(sample.input) ||
        !std::isfinite(sample.output)) {
      ++result.excluded_count;
      continue;
    }
    if (!sample.training) continue;
    sum_x += sample.input;
    sum_y += sample.output;
    sum_xx += sample.input * sample.input;
    sum_xy += sample.input * sample.output;
    ++training_count;
  }
  const double denominator =
      static_cast<double>(training_count) * sum_xx - sum_x * sum_x;
  if (training_count < minimum_training_samples ||
      !std::isfinite(denominator) || std::abs(denominator) < 1e-12)
    return result;
  result.slope =
      (static_cast<double>(training_count) * sum_xy - sum_x * sum_y) /
      denominator;
  result.intercept =
      (sum_y - result.slope * sum_x) / static_cast<double>(training_count);
  if (!std::isfinite(result.slope) || !std::isfinite(result.intercept))
    return result;

  double train_squared{};
  double train_absolute{};
  double validation_squared{};
  double validation_absolute{};
  for (std::size_t i = 0; i < data.size(); ++i) {
    const auto& sample = data[i];
    if (!sample.accepted || !std::isfinite(sample.input) ||
        !std::isfinite(sample.output))
      continue;
    const double error =
        sample.output - (result.slope * sample.input + result.intercept);
    if (sample.training)
      addError(result.training, error, train_squared, train_absolute);
    else
      addError(result.validation, error, validation_squared,
               validation_absolute);
  }
  auto finish = [](RegressionMetrics& metrics, double squared,
                   double absolute) noexcept {
    if (metrics.sample_count == 0) return;
    metrics.rmse =
        std::sqrt(squared / static_cast<double>(metrics.sample_count));
    metrics.mae = absolute / static_cast<double>(metrics.sample_count);
  };
  finish(result.training, train_squared, train_absolute);
  finish(result.validation, validation_squared, validation_absolute);
  result.valid = result.validation.sample_count > 0;
  return result;
}

struct GeometryCalibrationCandidate {
  double value{};
  double intercept{};
  RegressionMetrics training{};
  RegressionMetrics validation{};
  bool valid{};
};

template <std::size_t Capacity>
GeometryCalibrationCandidate fitDistanceScale(
    const FixedDataset<LineFitSample, Capacity>& reported_to_reference)
    noexcept {
  const auto fit = fitLine(reported_to_reference);
  return {fit.slope, fit.intercept, fit.training, fit.validation,
          fit.valid && fit.slope > 0.0};
}

template <std::size_t Capacity>
GeometryCalibrationCandidate fitEffectiveTrackWidth(
    const FixedDataset<LineFitSample, Capacity>& angle_to_wheel_difference)
    noexcept {
  const auto fit = fitLine(angle_to_wheel_difference);
  return {fit.slope, fit.intercept, fit.training, fit.validation,
          fit.valid && fit.slope > 0.0};
}

template <std::size_t Capacity>
GeometryCalibrationCandidate fitParallelTrackingOffset(
    const FixedDataset<LineFitSample, Capacity>& angle_to_wheel_distance)
    noexcept {
  const auto fit = fitLine(angle_to_wheel_distance);
  return {-fit.slope, fit.intercept, fit.training, fit.validation,
          fit.valid};
}

template <std::size_t Capacity>
GeometryCalibrationCandidate fitLateralTrackingOffset(
    const FixedDataset<LineFitSample, Capacity>& angle_to_wheel_distance)
    noexcept {
  const auto fit = fitLine(angle_to_wheel_distance);
  return {fit.slope, fit.intercept, fit.training, fit.validation, fit.valid};
}

struct SysIdSample {
  double voltage_V{};
  double velocity_mps{};
  double acceleration_mps2{};
  bool training{};
  bool accepted{};
  std::uint32_t exclusion_bits{};
};

struct FeedforwardFit {
  double kS_V{};
  double kV_Vs_per_m{};
  double kA_Vs2_per_m{};
  RegressionMetrics training{};
  RegressionMetrics validation{};
  std::uint32_t excluded_count{};
  bool valid{};
};

inline bool solveThreeByThree(double matrix[3][4], double result[3]) noexcept {
  for (std::size_t column = 0; column < 3; ++column) {
    std::size_t pivot = column;
    for (std::size_t row = column + 1; row < 3; ++row)
      if (std::abs(matrix[row][column]) >
          std::abs(matrix[pivot][column]))
        pivot = row;
    if (!std::isfinite(matrix[pivot][column]) ||
        std::abs(matrix[pivot][column]) < 1e-12)
      return false;
    if (pivot != column)
      for (std::size_t entry = column; entry < 4; ++entry)
        std::swap(matrix[pivot][entry], matrix[column][entry]);
    const double divisor = matrix[column][column];
    for (std::size_t entry = column; entry < 4; ++entry)
      matrix[column][entry] /= divisor;
    for (std::size_t row = 0; row < 3; ++row) {
      if (row == column) continue;
      const double factor = matrix[row][column];
      for (std::size_t entry = column; entry < 4; ++entry)
        matrix[row][entry] -= factor * matrix[column][entry];
    }
  }
  for (std::size_t i = 0; i < 3; ++i) result[i] = matrix[i][3];
  return std::isfinite(result[0]) && std::isfinite(result[1]) &&
         std::isfinite(result[2]);
}

template <std::size_t Capacity>
FeedforwardFit fitFeedforward(
    const FixedDataset<SysIdSample, Capacity>& data,
    double minimum_speed_mps) noexcept {
  FeedforwardFit fit{};
  if (!std::isfinite(minimum_speed_mps) || minimum_speed_mps <= 0.0)
    return fit;
  double normal[3][3]{};
  double rhs[3]{};
  std::uint32_t training_count{};
  for (std::size_t i = 0; i < data.size(); ++i) {
    const auto& sample = data[i];
    const bool usable = sample.accepted &&
                        std::isfinite(sample.voltage_V) &&
                        std::isfinite(sample.velocity_mps) &&
                        std::isfinite(sample.acceleration_mps2) &&
                        std::abs(sample.velocity_mps) >= minimum_speed_mps;
    if (!usable) {
      ++fit.excluded_count;
      continue;
    }
    if (!sample.training) continue;
    const double x[3] = {std::copysign(1.0, sample.velocity_mps),
                         sample.velocity_mps, sample.acceleration_mps2};
    for (std::size_t row = 0; row < 3; ++row) {
      rhs[row] += x[row] * sample.voltage_V;
      for (std::size_t column = 0; column < 3; ++column)
        normal[row][column] += x[row] * x[column];
    }
    ++training_count;
  }
  if (training_count < 3) return fit;
  double augmented[3][4]{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column)
      augmented[row][column] = normal[row][column];
    augmented[row][3] = rhs[row];
  }
  double coefficients[3]{};
  if (!solveThreeByThree(augmented, coefficients)) return fit;
  fit.kS_V = coefficients[0];
  fit.kV_Vs_per_m = coefficients[1];
  fit.kA_Vs2_per_m = coefficients[2];

  double train_squared{};
  double train_absolute{};
  double validation_squared{};
  double validation_absolute{};
  for (std::size_t i = 0; i < data.size(); ++i) {
    const auto& sample = data[i];
    if (!sample.accepted || !std::isfinite(sample.voltage_V) ||
        !std::isfinite(sample.velocity_mps) ||
        !std::isfinite(sample.acceleration_mps2) ||
        std::abs(sample.velocity_mps) < minimum_speed_mps)
      continue;
    const double predicted =
        fit.kS_V * std::copysign(1.0, sample.velocity_mps) +
        fit.kV_Vs_per_m * sample.velocity_mps +
        fit.kA_Vs2_per_m * sample.acceleration_mps2;
    const double error = sample.voltage_V - predicted;
    if (sample.training)
      addError(fit.training, error, train_squared, train_absolute);
    else
      addError(fit.validation, error, validation_squared,
               validation_absolute);
  }
  if (fit.training.sample_count > 0) {
    fit.training.rmse = std::sqrt(
        train_squared / static_cast<double>(fit.training.sample_count));
    fit.training.mae =
        train_absolute / static_cast<double>(fit.training.sample_count);
  }
  if (fit.validation.sample_count > 0) {
    fit.validation.rmse = std::sqrt(
        validation_squared /
        static_cast<double>(fit.validation.sample_count));
    fit.validation.mae = validation_absolute /
                         static_cast<double>(fit.validation.sample_count);
  }
  fit.valid = fit.validation.sample_count > 0 && fit.kS_V >= 0.0 &&
              fit.kV_Vs_per_m > 0.0 && fit.kA_Vs2_per_m > 0.0;
  return fit;
}

}  // namespace robot
