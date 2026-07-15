#pragma once

#include <cstdint>

#include "robot/drive/output_service.hpp"
#include "robot/platform/io.hpp"
#include "robot/runtime/mailbox.hpp"

namespace robot {

class OutputTask {
 public:
  OutputTask(Clock& clock, const ModeStore& modes,
             const ActuatorStore& actuators, OutputService& output,
             std::uint32_t period_ms)
      : clock_(clock),
        modes_(modes),
        actuators_(actuators),
        output_(output),
        period_ms_(period_ms) {}

  OutputResult tickOnce() {
    ActuatorFrame frame{};
    const bool available = actuators_.readLatest(frame);
    return output_.tick(modes_.read(), available ? &frame : nullptr,
                        clock_.nowUs());
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
};

}  // namespace robot
