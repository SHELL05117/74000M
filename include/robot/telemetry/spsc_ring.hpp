#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace robot {

template <typename T, std::size_t N>
class SpscRing {
  static_assert(N >= 2, "SPSC ring needs at least two slots");
  static_assert(std::is_trivially_copyable_v<T>,
                "realtime log elements must be trivially copyable");

 public:
  bool tryPush(const T& value) noexcept {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = increment(head);
    const std::size_t tail = tail_.load(std::memory_order_acquire);
    if (next == tail) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    slots_[head] = value;
    head_.store(next, std::memory_order_release);
    const std::size_t current_depth = distance(next, tail);
    std::size_t high = high_watermark_.load(std::memory_order_relaxed);
    while (current_depth > high &&
           !high_watermark_.compare_exchange_weak(
               high, current_depth, std::memory_order_relaxed,
               std::memory_order_relaxed)) {
    }
    return true;
  }

  bool tryPop(T& output) noexcept {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return false;
    output = slots_[tail];
    tail_.store(increment(tail), std::memory_order_release);
    return true;
  }

  std::size_t depth() const noexcept {
    return distance(head_.load(std::memory_order_acquire),
                    tail_.load(std::memory_order_acquire));
  }

  std::size_t highWatermark() const noexcept {
    return high_watermark_.load(std::memory_order_relaxed);
  }

  std::uint32_t dropped() const noexcept {
    return dropped_.load(std::memory_order_relaxed);
  }

  static constexpr std::size_t usableCapacity() noexcept { return N - 1; }

 private:
  static constexpr std::size_t increment(std::size_t index) noexcept {
    return (index + 1U) % N;
  }

  static constexpr std::size_t distance(std::size_t head,
                                        std::size_t tail) noexcept {
    return head >= tail ? head - tail : N - (tail - head);
  }

  std::array<T, N> slots_{};
  std::atomic<std::size_t> head_{0};
  std::atomic<std::size_t> tail_{0};
  std::atomic<std::size_t> high_watermark_{0};
  std::atomic<std::uint32_t> dropped_{0};
};

}  // namespace robot
