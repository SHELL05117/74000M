#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "robot/runtime/mode.hpp"
#include "robot/state/controller_snapshot.hpp"
#include "robot/telemetry/log_frame.hpp"
#include "robot/telemetry/spsc_ring.hpp"

namespace robot {

enum class RecordingState : std::uint8_t {
  Idle,
  Opening,
  Recording,
  Closing,
};

enum class RecordingError : std::uint8_t {
  None,
  CardMissing,
  DirectoryCreate,
  FileOpen,
  HeaderWrite,
  DataWrite,
  FooterWrite,
  Flush,
  Close,
  CommitRename,
  Internal,
};

enum RecordingEvent : std::uint32_t {
  kRecordingNoEvent = 0,
  kRecordingStartRequested = 1u << 0,
  kRecordingStarted = 1u << 1,
  kRecordingStopRequested = 1u << 2,
  kRecordingCompleted = 1u << 3,
  kRecordingFailed = 1u << 4,
};

struct RecordingControlConfig {
  TimeUs start_hold_us{3000000};
  TimeUs stop_hold_us{1000000};
  std::uint32_t button{kButtonY};
};

struct RecordingObservation {
  RecordingState state{RecordingState::Idle};
  RecordingError error{RecordingError::None};
  std::uint32_t session_sequence{};
  std::uint32_t event_bits{};
};

class RecordingControl {
 public:
  explicit RecordingControl(RecordingControlConfig config = {}) noexcept
      : config_(config) {}

  RecordingObservation observe(const ControllerSnapshot& controller,
                               const ModeSnapshot& mode) noexcept {
    RecordingObservation result = snapshot();
    const bool valid = controller.connected && controller.api_ok &&
                       controller.h.mode_epoch == mode.epoch;
    if (!valid || controller.h.time_us < last_time_us_ ||
        (epoch_initialized_ && controller.h.mode_epoch != epoch_)) {
      cancelPress();
      epoch_ = controller.h.mode_epoch;
      epoch_initialized_ = true;
      last_time_us_ = controller.h.time_us;
      return snapshot();
    }

    epoch_ = controller.h.mode_epoch;
    epoch_initialized_ = true;
    last_time_us_ = controller.h.time_us;
    const bool down = (controller.buttons & config_.button) != 0;
    const RecordingState current = state();

    if (down && !button_down_) {
      press_valid_ = current == RecordingState::Idle ||
                     current == RecordingState::Recording;
      press_state_ = current;
      press_start_us_ = controller.h.time_us;
    } else if (down && button_down_ && current != press_state_) {
      press_valid_ = false;
    } else if (!down && button_down_) {
      if (press_valid_ && current == press_state_ &&
          controller.h.time_us >= press_start_us_) {
        const TimeUs held_us = controller.h.time_us - press_start_us_;
        if (current == RecordingState::Idle &&
            held_us >= config_.start_hold_us) {
          if (!available_.load(std::memory_order_acquire)) {
            fail(RecordingError::Internal);
            result.event_bits |= kRecordingFailed;
            button_down_ = down;
            result.state = state();
            result.error = error();
            return result;
          }
          const std::uint32_t next =
              session_sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
          start_time_ms_.store(
              static_cast<std::uint32_t>(controller.h.time_us / 1000u),
              std::memory_order_relaxed);
          state_.store(RecordingState::Opening, std::memory_order_release);
          result.session_sequence = next;
          result.event_bits |= kRecordingStartRequested;
        } else if (current == RecordingState::Recording &&
                   held_us >= config_.stop_hold_us) {
          state_.store(RecordingState::Closing, std::memory_order_release);
          result.event_bits |= kRecordingStopRequested;
        }
      }
      press_valid_ = false;
    }
    button_down_ = down;

    result.state = state();
    result.error = error();
    result.session_sequence =
        session_sequence_.load(std::memory_order_relaxed);
    return result;
  }

  RecordingObservation snapshot() const noexcept {
    return {state(), error(),
            session_sequence_.load(std::memory_order_relaxed), 0};
  }

  RecordingState state() const noexcept {
    return state_.load(std::memory_order_acquire);
  }

  RecordingError error() const noexcept {
    return error_.load(std::memory_order_acquire);
  }

  std::uint32_t startTimeMs() const noexcept {
    return start_time_ms_.load(std::memory_order_relaxed);
  }

  std::uint32_t alertSequence() const noexcept {
    return alert_sequence_.load(std::memory_order_relaxed);
  }

  void setAvailable(bool available) noexcept {
    available_.store(available, std::memory_order_release);
  }

  void requestStop() noexcept {
    RecordingState current = state();
    while ((current == RecordingState::Opening ||
            current == RecordingState::Recording) &&
           !state_.compare_exchange_weak(
               current, RecordingState::Closing,
               std::memory_order_acq_rel)) {
    }
    cancelPress();
  }

  bool markRecording() noexcept {
    RecordingState expected = RecordingState::Opening;
    if (!state_.compare_exchange_strong(
            expected, RecordingState::Recording, std::memory_order_acq_rel))
      return false;
    error_.store(RecordingError::None, std::memory_order_release);
    return true;
  }

  bool markIdle() noexcept {
    RecordingState expected = RecordingState::Closing;
    return state_.compare_exchange_strong(
        expected, RecordingState::Idle, std::memory_order_acq_rel);
  }

  void fail(RecordingError error) noexcept {
    error_.store(error == RecordingError::None ? RecordingError::Internal
                                                : error,
                 std::memory_order_release);
    state_.store(RecordingState::Idle, std::memory_order_release);
    alert_sequence_.fetch_add(1, std::memory_order_relaxed);
  }

 private:
  void cancelPress() noexcept {
    button_down_ = false;
    press_valid_ = false;
    press_start_us_ = 0;
  }

  RecordingControlConfig config_{};
  std::atomic<RecordingState> state_{RecordingState::Idle};
  std::atomic<RecordingError> error_{RecordingError::None};
  std::atomic<std::uint32_t> session_sequence_{0};
  std::atomic<std::uint32_t> start_time_ms_{0};
  std::atomic<std::uint32_t> alert_sequence_{0};
  std::atomic<bool> available_{true};
  TimeUs press_start_us_{};
  TimeUs last_time_us_{};
  std::uint32_t epoch_{};
  RecordingState press_state_{RecordingState::Idle};
  bool button_down_{};
  bool press_valid_{};
  bool epoch_initialized_{};
};

class RecordingSessionSink {
 public:
  virtual bool begin(std::uint32_t session_sequence,
                     std::uint32_t start_time_ms) = 0;
  virtual bool write(const LogFrame* frames, std::size_t count) = 0;
  virtual bool finish(std::uint32_t producer_drops) = 0;
  virtual void abort() = 0;
  virtual RecordingError error() const noexcept = 0;
  virtual ~RecordingSessionSink() = default;
};

template <std::size_t RingCapacity, std::size_t BatchSize>
class RecordingWorker {
  static_assert(BatchSize > 0);

 public:
  RecordingWorker(RecordingControl& control,
                  SpscRing<LogFrame, RingCapacity>& ring,
                  RecordingSessionSink& sink)
      : control_(control), ring_(ring), sink_(sink) {}

  void tickOnce() {
    const RecordingState state = control_.state();
    if (state == RecordingState::Opening && !session_open_) {
      const auto snapshot = control_.snapshot();
      if (!sink_.begin(snapshot.session_sequence, control_.startTimeMs())) {
        failSession();
        return;
      }
      session_open_ = true;
      drops_at_open_ = ring_.dropped();
      if (!control_.markRecording()) {
        sink_.abort();
        session_open_ = false;
        control_.fail(RecordingError::Internal);
        return;
      }
    }

    if (session_open_ &&
        (control_.state() == RecordingState::Recording ||
         control_.state() == RecordingState::Closing)) {
      if (!drainBatch()) {
        failSession();
        return;
      }
    }

    if (session_open_ && control_.state() == RecordingState::Closing &&
        ring_.depth() == 0) {
      if (!sink_.finish(ring_.dropped() - drops_at_open_)) {
        failSession();
        return;
      }
      session_open_ = false;
      if (!control_.markIdle()) control_.fail(RecordingError::Internal);
    }
    if (!session_open_ && control_.state() == RecordingState::Closing) {
      discardQueued();
      if (!control_.markIdle()) control_.fail(RecordingError::Internal);
    }
  }

 private:
  bool drainBatch() {
    std::size_t count = 0;
    while (count < BatchSize && ring_.tryPop(batch_[count])) ++count;
    return count == 0 || sink_.write(batch_.data(), count);
  }

  void discardQueued() noexcept {
    LogFrame discarded{};
    while (ring_.tryPop(discarded)) {
    }
  }

  void failSession() {
    const RecordingError cause = sink_.error();
    sink_.abort();
    session_open_ = false;
    discardQueued();
    control_.fail(cause);
  }

  RecordingControl& control_;
  SpscRing<LogFrame, RingCapacity>& ring_;
  RecordingSessionSink& sink_;
  std::array<LogFrame, BatchSize> batch_{};
  std::uint32_t drops_at_open_{};
  bool session_open_{};
};

}  // namespace robot
