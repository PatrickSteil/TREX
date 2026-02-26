#pragma once

#include <atomic>
#include <cstdint>

struct alignas(64) PaddedAtomicBool {
  std::atomic<uint8_t> value;

  constexpr PaddedAtomicBool(bool v = false) noexcept : value(v ? 1 : 0) {}

  inline void set_true() noexcept { value.store(1, std::memory_order_release); }

  inline bool set_true_if_first() noexcept {
    return value.exchange(1, std::memory_order_acq_rel) == 0;
  }

  inline bool cas_false_to_true() noexcept {
    uint8_t expected = 0;
    return value.compare_exchange_strong(expected, 1, std::memory_order_acq_rel,
                                         std::memory_order_relaxed);
  }

  inline bool test() const noexcept {
    return value.load(std::memory_order_acquire) != 0;
  }

  inline void reset() noexcept { value.store(0, std::memory_order_release); }

  inline void atomic_or_true() noexcept {
    value.fetch_or(1, std::memory_order_acq_rel);
  }

  inline bool getValue() const noexcept {
    return value.load(std::memory_order_relaxed) != 0;
  }
};
