// Fixed-capacity FIFO queue with static storage (no heap).
#pragma once

#include <stdint.h>

namespace micromouse::util {

/// FIFO of at most `Capacity` elements. push() fails instead of overwriting.
template <typename T, uint16_t Capacity>
class RingBuffer {
  static_assert(Capacity > 0, "capacity must be positive");

 public:
  /// Appends `value`; returns false (and changes nothing) if full.
  bool push(const T& value) {
    if (count_ == Capacity) return false;
    items_[tail_] = value;
    tail_ = next(tail_);
    ++count_;
    return true;
  }

  /// Removes the oldest element into `out`; returns false if empty.
  bool pop(T& out) {
    if (count_ == 0) return false;
    out = items_[head_];
    head_ = next(head_);
    --count_;
    return true;
  }

  void clear() {
    head_ = 0;
    tail_ = 0;
    count_ = 0;
  }

  bool empty() const { return count_ == 0; }
  bool full() const { return count_ == Capacity; }
  uint16_t size() const { return count_; }
  static constexpr uint16_t capacity() { return Capacity; }

 private:
  static uint16_t next(const uint16_t index) {
    return index + 1u == Capacity ? 0 : static_cast<uint16_t>(index + 1u);
  }

  T items_[Capacity] = {};
  uint16_t head_ = 0;
  uint16_t tail_ = 0;
  uint16_t count_ = 0;
};

}  // namespace micromouse::util
