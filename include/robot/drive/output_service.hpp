#pragma once

#include <cmath>
#include <cstdint>

#include "robot/drive/actuator_frame.hpp"
#include "robot/platform/io.hpp"
#include "robot/runtime/mode.hpp"

namespace robot {

enum OutputReject : std::uint32_t {
  kOutputAccepted = 0,
  kOutputNoFrame = 1u << 0,
  kOutputModeDisabled = 1u << 1,
  kOutputEpochMismatch = 1u << 2,
  kOutputFutureTimestamp = 1u << 3,
  kOutputStale = 1u << 4,
  kOutputSequenceRegression = 1u << 5,
  kOutputNonfinite = 1u << 6,
  kOutputOutOfRange = 1u << 7,
};

enum class OutputAction : std::uint8_t { WroteVoltage, Stopped };

struct OutputServiceConfig {
  TimeUs frame_ttl_us{};
  double max_voltage_V{12.0};
  double zero_epsilon_V{1e-9};
  StopMode stale_stop_mode{StopMode::Brake};
  StopMode lift_stale_stop_mode{StopMode::Hold};
};

struct OutputResult {
  OutputAction action{OutputAction::Stopped};
  std::uint32_t reject_bits{};
  bool io_ok{};
  TimeUs frame_age_us{};
};

class OutputService {
 public:
  OutputService(DriveIO& io, OutputServiceConfig config)
      : io_(io), config_(config) {}

  OutputResult tick(const ModeSnapshot& mode, const ActuatorFrame* frame,
                    TimeUs now_us) {
    OutputResult result{};
    if (frame == nullptr) {
      result.reject_bits |= kOutputNoFrame;
    } else {
      if (!mode.enabled) result.reject_bits |= kOutputModeDisabled;
      if (frame->h.mode_epoch != mode.epoch)
        result.reject_bits |= kOutputEpochMismatch;
      if (frame->h.time_us > now_us) {
        result.reject_bits |= kOutputFutureTimestamp;
      } else {
        result.frame_age_us = now_us - frame->h.time_us;
        if (config_.frame_ttl_us == 0 ||
            result.frame_age_us > config_.frame_ttl_us)
          result.reject_bits |= kOutputStale;
      }
      if (last_epoch_ == mode.epoch &&
          frame->h.sequence < last_sequence_)
        result.reject_bits |= kOutputSequenceRegression;
      if (!std::isfinite(frame->left_V) || !std::isfinite(frame->right_V) ||
          !std::isfinite(frame->lift_V))
        result.reject_bits |= kOutputNonfinite;
      if (!std::isfinite(config_.max_voltage_V) ||
          config_.max_voltage_V <= 0.0 ||
          config_.max_voltage_V > 12.0 ||
          std::abs(frame->left_V) > config_.max_voltage_V ||
          std::abs(frame->right_V) > config_.max_voltage_V ||
          std::abs(frame->lift_V) > config_.max_voltage_V)
        result.reject_bits |= kOutputOutOfRange;
    }

    if (result.reject_bits != kOutputAccepted) {
      result.action = OutputAction::Stopped;
      const bool drive_ok = io_.stop(config_.stale_stop_mode);
      const bool lift_ok = io_.stopLift(config_.lift_stale_stop_mode);
      result.io_ok = drive_ok && lift_ok;
      return result;
    }

    last_epoch_ = mode.epoch;
    last_sequence_ = frame->h.sequence;
    const bool both_zero =
        std::abs(frame->left_V) <= config_.zero_epsilon_V &&
        std::abs(frame->right_V) <= config_.zero_epsilon_V;
    const bool lift_zero =
        std::abs(frame->lift_V) <= config_.zero_epsilon_V;
    const bool drive_ok = both_zero
                              ? io_.stop(frame->zero_behavior)
                              : io_.writeVoltage(frame->left_V,
                                                 frame->right_V);
    const bool lift_ok = lift_zero
                             ? io_.stopLift(frame->lift_zero_behavior)
                             : io_.writeLiftVoltage(frame->lift_V);
    result.action = both_zero && lift_zero ? OutputAction::Stopped
                                           : OutputAction::WroteVoltage;
    result.io_ok = drive_ok && lift_ok;
    return result;
  }

 private:
  DriveIO& io_;
  OutputServiceConfig config_{};
  std::uint32_t last_epoch_{};
  std::uint32_t last_sequence_{};
};

}  // namespace robot
