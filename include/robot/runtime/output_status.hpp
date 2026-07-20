#pragma once

#include <atomic>
#include <cstdint>

#include "robot/drive/output_service.hpp"

namespace robot {

struct OutputStatus {
  std::uint32_t actuator_sequence{};
  std::uint32_t reject_bits{};
  OutputAction action{OutputAction::Stopped};
  bool write_attempted{};
  bool io_ok{};
};

class OutputStatusStore {
 public:
  virtual void publish(const OutputStatus& status) noexcept = 0;
  virtual bool readLatest(OutputStatus& status) const noexcept = 0;
  virtual ~OutputStatusStore() = default;
};

class AtomicOutputStatusStore final : public OutputStatusStore {
 public:
  void publish(const OutputStatus& status) noexcept override {
    generation_.fetch_add(1, std::memory_order_acq_rel);
    actuator_sequence_.store(status.actuator_sequence,
                             std::memory_order_relaxed);
    reject_bits_.store(status.reject_bits, std::memory_order_relaxed);
    action_.store(static_cast<std::uint8_t>(status.action),
                  std::memory_order_relaxed);
    write_attempted_.store(status.write_attempted,
                           std::memory_order_relaxed);
    io_ok_.store(status.io_ok, std::memory_order_relaxed);
    generation_.fetch_add(1, std::memory_order_release);
  }

  bool readLatest(OutputStatus& status) const noexcept override {
    const std::uint32_t before =
        generation_.load(std::memory_order_acquire);
    if (before == 0 || (before & 1u) != 0) return false;
    status.actuator_sequence =
        actuator_sequence_.load(std::memory_order_relaxed);
    status.reject_bits = reject_bits_.load(std::memory_order_relaxed);
    status.action =
        static_cast<OutputAction>(action_.load(std::memory_order_relaxed));
    status.write_attempted =
        write_attempted_.load(std::memory_order_relaxed);
    status.io_ok = io_ok_.load(std::memory_order_relaxed);
    return before == generation_.load(std::memory_order_acquire);
  }

 private:
  std::atomic<std::uint32_t> generation_{0};
  std::atomic<std::uint32_t> actuator_sequence_{0};
  std::atomic<std::uint32_t> reject_bits_{0};
  std::atomic<std::uint8_t> action_{
      static_cast<std::uint8_t>(OutputAction::Stopped)};
  std::atomic<bool> write_attempted_{false};
  std::atomic<bool> io_ok_{false};
};

}  // namespace robot
