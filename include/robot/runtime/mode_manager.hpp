#pragma once

#include <cstdint>

#include "robot/runtime/mailbox.hpp"
#include "robot/runtime/mode.hpp"

namespace robot {

inline bool isEnabledMode(CompetitionMode mode) noexcept {
  return mode == CompetitionMode::Driver ||
         mode == CompetitionMode::AutonomousInterface ||
         mode == CompetitionMode::Test;
}

class ModeManager {
 public:
  explicit ModeManager(ModeStore& store) : store_(store) {
    store_.publish(ModeSnapshot{});
  }

  bool transitionTo(CompetitionMode next, TimeUs now_us,
                    std::uint32_t fault_bits = 0,
                    bool field_connected = false) {
    const ModeSnapshot previous = store_.read();
    if (previous.mode == next && previous.fault_bits == fault_bits &&
        previous.field_connected == field_connected)
      return false;

    ModeSnapshot entering{};
    entering.mode = next;
    entering.enabled = false;
    entering.field_connected = field_connected;
    entering.epoch = previous.epoch + 1;
    entering.transition_time_us = now_us;
    entering.fault_bits = fault_bits;
    store_.publish(entering);
    pending_enable_ = isEnabledMode(next);
    boundary_stop_epoch_ = entering.epoch;
    return true;
  }

  bool acknowledgeBoundaryStop(std::uint32_t epoch) {
    ModeSnapshot current = store_.read();
    if (current.epoch != epoch || boundary_stop_epoch_ != epoch) return false;
    current.enabled = pending_enable_;
    store_.publish(current);
    pending_enable_ = false;
    boundary_stop_epoch_ = 0;
    return true;
  }

  ModeSnapshot snapshot() const { return store_.read(); }

  bool boundaryStopPending() const noexcept {
    return boundary_stop_epoch_ != 0;
  }

 private:
  ModeStore& store_;
  bool pending_enable_{};
  std::uint32_t boundary_stop_epoch_{};
};

}  // namespace robot
