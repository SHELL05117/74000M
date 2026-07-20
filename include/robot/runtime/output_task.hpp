#pragma once

#include <cstdint>

#include "robot/drive/output_service.hpp"
#include "robot/platform/io.hpp"
#include "robot/runtime/mailbox.hpp"
#include "robot/runtime/output_status.hpp"

namespace robot {

class OutputTask {
 public:
  OutputTask(Clock& clock, const ModeStore& modes,
             const ActuatorStore& actuators, OutputService& output,
             std::uint32_t period_ms,
             OutputStatusStore* status_store = nullptr)
      : clock_(clock),
        modes_(modes),
        actuators_(actuators),
        output_(output),
        period_ms_(period_ms),
        status_store_(status_store) {}

  OutputResult tickOnce() {
    ActuatorFrame frame{};
    const bool available = actuators_.readLatest(frame);
    const OutputResult result =
        output_.tick(modes_.read(), available ? &frame : nullptr,
                     clock_.nowUs());
    if (available && result.reject_bits == kOutputAccepted && result.io_ok)
      last_written_sequence_ = frame.h.sequence;
    if (status_store_ != nullptr) {
      status_store_->publish(
          {last_written_sequence_, result.reject_bits,
           result.action, true, result.io_ok});
    }
    return result;
  }

  [[noreturn]] void run() {
    std::uint32_t wake_ms = clock_.nowMs();
    while (true) {
      tickOnce();
      clock_.delayUntilMs(wake_ms, period_ms_);
    }
  }

 private:
  Clock& clock_;
  const ModeStore& modes_;
  const ActuatorStore& actuators_;
  OutputService& output_;
  std::uint32_t period_ms_{};
  OutputStatusStore* status_store_{};
  std::uint32_t last_written_sequence_{};
};

}  // namespace robot
